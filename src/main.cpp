#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "helpers.h"
#include "json.hpp"
#include "map.h"

vector<double> JMT(vector<double> &start, vector<double> &end, double T);

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> path_x;
vector<double> path_y;
vector<double> path_s;
vector<double> path_v;
vector<double> path_a;
vector<double> path_d;

int main() {
  uWS::Hub h;

  Map &map = Map::getInstance();

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);

    map.addWaypoint(x, y, s, d_x, d_y);
  }

  map.buildSplines();

  h.onMessage([&path_d, &path_a, &path_v, &path_s,&path_x, &path_y,&map,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds        
           */

          int pp_size = previous_path_x.size();

          std::cout<<"previous path"<<std::endl;      
          for (int i = 0; i < pp_size; ++i)
          {
            std::cout<<previous_path_x[i]<<" ";
          }
          std::cout<<std::endl;
          std::cout<<std::endl;


          int eaten_points = path_x.size() - pp_size;
          static bool init = true;
          if (init)
          {
            eaten_points = 50;
            init = false;
          }

          static double mph = 10;
          double T = 3.0;
          double v = mph2mps(mph);
          mph +=0.5;
          if (mph>47)
          {
            mph = 47;            
          }

          vector<double> start_s;
          vector<double> end_s;

          vector<double> start_d;
          vector<double> end_d;

          vector<double> ego_s_d;
          if (pp_size == 0)
          {
            std::cout<<"pp size 0"<<std::endl;
            vector<double> xy = map.toXY(car_s, car_d);
            path_x.push_back(xy[0]);
            path_y.push_back(xy[1]);
            path_s.push_back(car_s);
            path_v.push_back(0.0);
            ego_s_d = {car_s, car_d};
            start_s = {car_s, 0.0, 0.0};
            end_s = {car_s + v*T, v, 0.0};
            start_d = {car_d, 0.0, 0.0};
            end_d = {car_d, 0.0, 0.0};
          }
          else
          {
            std::cout<<"erasing "<<eaten_points<<std::endl;
            std::cout<<"path x size "<<path_x.size()<<std::endl;
            path_x.erase(path_x.begin(), path_x.begin() + eaten_points);
            path_y.erase(path_y.begin(), path_y.begin() + eaten_points);
            path_s.erase(path_s.begin(), path_s.begin() + eaten_points);
            path_v.erase(path_v.begin(), path_v.begin() + eaten_points);
            path_a.erase(path_a.begin(), path_a.begin() + eaten_points);
            path_d.erase(path_d.begin(), path_d.begin() + eaten_points);
            std::cout<<"path x size "<<path_x.size()<<std::endl;
            double yaw = atan2(path_y[path_y.size()-1]-path_y[path_y.size()-2],path_x[path_x.size()-1]-path_x[path_x.size()-2]);
            ego_s_d = map.toFrenet(path_x[path_x.size()-1],path_y[path_y.size()-1],yaw);

            //start_s = {ego_s_d[0], v, 0.0};            
            //end_s = {ego_s_d[0] + v*T, v, 0.0};
            start_s = {path_s[path_s.size()-1], path_v[path_v.size()-1], path_a[path_a.size()-1]};
            end_s = {path_s[path_s.size()-1] + v*T, v, 0.0};
            start_d = {path_d[path_d.size()-1], 0.0, 0.0};
            end_d = {path_d[path_d.size()-1], 0.0, 0.0};
          }

          
          vector<double> coeffs_s = JMT(start_s, end_s, T);
          vector<double> coeffs_d = JMT(start_d, end_d, T);

          for (int i = 1; i<=eaten_points; ++i)
          {
            //double next_s = ego_s_d[0] + i*0.5;
            double t = i * 0.02;
            double t_2 = pow(t, 2);
            double t_3 = pow(t, 3);
            double t_4 = pow(t, 4);
            double t_5 = pow(t, 5);

            double next_s =   coeffs_s[0]  + 
                              coeffs_s[1] * t + 
                              coeffs_s[2] * t_2 + 
                              coeffs_s[3] * t_3 + 
                              coeffs_s[4] * t_4 + 
                              coeffs_s[5] * t_5;

            double next_d =   coeffs_d[0]  + 
                              coeffs_d[1] * t + 
                              coeffs_d[2] * t_2 + 
                              coeffs_d[3] * t_3 + 
                              coeffs_d[4] * t_4 + 
                              coeffs_d[5] * t_5;

            double next_v =   coeffs_s[1]  + 
                              2 * coeffs_s[2] * t + 
                              3 * coeffs_s[3] * t_2 + 
                              4 * coeffs_s[4] * t_3 + 
                              5 * coeffs_s[5] * t_4;

            double next_a =   2 * coeffs_s[2] + 
                              6 * coeffs_s[3] * t + 
                              12 * coeffs_s[4] * t_2 + 
                              20 * coeffs_s[5] * t_3;                                                        
                                                                                                   
            vector<double> xy = map.toRealWorldXY(next_s, next_d); 
            path_x.push_back(xy[0]);
            path_y.push_back(xy[1]);
            path_s.push_back(next_s);
            path_v.push_back(next_v);    
            path_a.push_back(next_a);     
            path_d.push_back(next_d);       
          }

          std::cout<<"new path"<<std::endl;  
          for (int i = 1; i<path_x.size(); ++i)
          {
            std::cout<<path_x[i]<<" ";
          }

          std::cout<<std::endl;
          std::cout<<std::endl;

/*


          vector<double> ppx;
          vector<double> ppy;
          for (int i = 0; i < previous_path_x.size(); ++i)
          {
            ppx.push_back(previous_path_x[i]);
            ppy.push_back(previous_path_y[i]);
          }


          vector<double> ego_s_d = {car_s, car_d};
          if (ppx.size()<2)
          {          
            double oldoldppx = ppx[ppx.size()-2];
            double oldoldppy = ppy[ppy.size()-2];
            double oldppx = ppx[ppx.size()-1];
            double oldppy = ppy[ppy.size()-1];
            double new_yaw = atan2(oldppy - oldoldppy, oldppx - oldoldppx);
            ego_s_d = map.toFrenet(oldppx, oldppy, new_yaw);
          }
          
          std::cout<<"new size "<<50-ppx.size()<<std::endl;

          std::cout<<"previous path "<<std::endl;
          for(int i = 0; i<ppx.size();++i)
          {
            std::cout<<ppx[i]<<" ";
          }
          std::cout<<std::endl;
          std::cout<<"previous path "<<std::endl;


          double dist_inc = 0.5;
          for(int i = 1; i<=50-ppx.size();++i)
          {
            double next_s = ego_s_d[0] + (i)*dist_inc;
            double next_d = ego_s_d[1];
            vector<double> xy = map.toXY(next_s, next_d);
            ppx.push_back(xy[0]);
            ppy.push_back(xy[1]);
            std::cout<<xy[0]<<"   ";
          }
          std::cout<<std::endl;
          std::cout<<std::endl;
          std::cout<<std::endl;

          /*
double dist_inc = 0.5;
for (int i = 0; i < 50; ++i) {
  next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
  next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
}
*/
          static bool hola = true;
          if (hola)
          {
            msgJson["next_x"] = path_x;
            msgJson["next_y"] = path_y;

            auto msg = "42[\"control\","+ msgJson.dump()+"]";

            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          //  hola = false;
          }
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}


vector<double> JMT(vector<double> &start, vector<double> &end, double T)
{
    MatrixXd a(3,3);
    double T2 =  T*T, 
           T3 = T2*T, 
           T4 = T3*T,
           T5 = T4*T;
    a <<  T3,    T4,    T5, 
        3*T2,  4*T3,  5*T4, 
         6*T, 12*T2, 20*T3;
    MatrixXd aInv = a.inverse();
    
    VectorXd b(3);
    b << end[0] - (start[0] + start[1]*T + 0.5*start[2]*T2),
         end[1] - (           start[1]   +     start[2]*T),
         end[2] - (                            start[2]);
    VectorXd alpha = aInv * b;
    
    vector<double> output = {start[0], start[1], 0.5*start[2], alpha[0], alpha[1], alpha[2]};
    return output;
}