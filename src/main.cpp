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
#include "trajectory.h"
#include "vehicle.h"

vector<double> JMT(vector<double> &start, vector<double> &end, double T);
vector<double> car_distance_cost(vector<Vehicle> ego_cars, vector<Vehicle> other_cars);
vector<double> front_car_cost(vector<Vehicle> ego_cars, vector<Vehicle> other_cars);
vector<double> lane_cost(vector<Vehicle> ego_cars);
Trajectory generateTrajectory(vector<double> start_s, vector<double> end_s, vector<double> start_d, vector<double> end_d, double T, int traj_size);

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::VectorXd;


int main() {
  uWS::Hub h;

  Trajectory traj;

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

  h.onMessage([&traj,&map,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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

          int pp_size = previous_path_x.size();

          static double mph = 10;

          int in_sov = 0;
          vector<Vehicle> others;
          for (auto v_data : sensor_fusion)
          {
            Vehicle v = Vehicle(v_data[0], v_data[1], v_data[2], v_data[3], v_data[4], v_data[5], v_data[6]);

            if (v.s() < (car_s + 70.0) && v.s() > (car_s - 70.0))
            {
              others.push_back(v);       
              others[in_sov].propagate(70);    
              //std::cout<<"others sov "<<in_sov<<" propagate "<<pp_size<< " init s " << others[in_sov].getTrajectory().s(0)<<" last s "<<others[in_sov].getTrajectory().lastS()<<std::endl;
              in_sov++;
            }
          }

          //std::cout << "pp size " << pp_size << std::endl;

          for (int i=0; i<others.size(); i++)
          {
            Vehicle other = others[i];
            Trajectory other_traj = others[i].getTrajectory();
            double d = other.d();
            if (d < (2+4*1+2) && d>(2+4*1-2))
            {
              //double s = others[i].s() + pp_size*.02*others[i].v();
              double s = other.s();
              static double distance = 30;
              if ((s > car_s) && (s-car_s < distance))
              {

                /*
                std::cout<<"propagated other path "<< other.id() <<" size "<<other_traj.size()<<std::endl;      
                for (int e = 0; e < other_traj.size(); e++)
                {
                  std::cout<<other_traj.s(e)<<" ";
                }
                std::cout<<std::endl;
                std::cout<<std::endl;
                */

                //std::cout<<"CAR IS CLOSE and it is moving at"<<others[i].v()<<std::endl;
                mph = mps2mph(other.v()-1);
                distance = 40;
              }
              else
              {
                distance = 30;
                mph +=0.3;
              }
            }
          }

          /*
          std::cout<<"previous path"<<std::endl;      
          for (int i = 0; i < pp_size; ++i)
          {
            std::cout<<previous_path_x[i]<<" ";
          }
          std::cout<<std::endl;
          std::cout<<std::endl;
          */


          int eaten_points = traj.size() - pp_size;
          int cpy_idx = 0;
          static bool init = true;
          if (init)
          {
            eaten_points = 49;
            init = false;
          }


          double T = 3.0;
          double v = mph2mps(mph);
          
          if (mph>47)
          {
            mph = 47;            
          }

          Trajectory new_traj;

          vector<double> start_s;
          vector<double> end_s;

          vector<double> start_d;
          vector<double> end_d;

          vector<double> ego_s_d;

          vector<Vehicle> ego_cars; 


          if (pp_size == 0)
          {
            start_s = {car_s, 0.0, 0.0};
            end_s = {car_s + v*T, v, 0.0};
            start_d = {car_d, 0.0, 0.0};
            end_d = {car_d, 0.0, 0.0};

            //std::cout<<"pp size 0"<<std::endl;
            vector<double> xy = map.toXY(car_s, car_d);
            traj.add(xy, start_s, start_d, 0.0);
            new_traj.add(xy, start_s, start_d, 0.0);

            Trajectory traj0 = generateTrajectory(start_s, end_s, start_d, end_d, T, eaten_points+cpy_idx);           
            Trajectory candidate0 = Trajectory::appendTrajectories(new_traj, traj0);   
            Vehicle ego0;
            ego0.addTrajectory(candidate0);
            ego_cars.push_back(ego0); 

            traj = candidate0; 
          }
          else
          {
            traj.removeN(eaten_points);

            cpy_idx = 40;            

            new_traj = traj.copyUpTo(traj.size()-cpy_idx);            

            std::cout<<"new traj size"<<new_traj.size()<<std::endl;

            start_s = {new_traj.lastS(), new_traj.lastSV(), new_traj.lastSA()};
            end_s = {new_traj.lastS() + v * T, v, 0.0};
            start_d = {new_traj.lastD(), new_traj.lastDV(), new_traj.lastDA()};
            end_d = {new_traj.lastD(), 0.0, 0.0};

            Trajectory traj1 = generateTrajectory(start_s, end_s, start_d, end_d, T, eaten_points+cpy_idx);
            Trajectory candidate1 = Trajectory::appendTrajectories(new_traj, traj1);  
            Vehicle ego1;
            ego1.addTrajectory(candidate1);
            ego_cars.push_back(ego1);  
            std::cout<<"candidate1 size "<<candidate1.size()<<" init lane "<<getLane(candidate1.d(0),4,2)<<" last lane "<<getLane(candidate1.lastD(),4,2)<<" init s "<<candidate1.s(0)<<" last s "<<candidate1.lastS()<<std::endl;

            start_s = {new_traj.lastS(), new_traj.lastSV(), new_traj.lastSA()};
            end_s = {new_traj.lastS() + v * T, v, 0.0};
            start_d = {new_traj.lastD(), new_traj.lastDV(), new_traj.lastDA()};
            end_d = {new_traj.lastD() + 4.0, 0.0, 0.0};

            Trajectory traj2 = generateTrajectory(start_s, end_s, start_d, end_d, T, eaten_points+cpy_idx);
            Trajectory candidate2 = Trajectory::appendTrajectories(new_traj, traj2);  
            Vehicle ego2;
            ego2.addTrajectory(candidate2);
            ego_cars.push_back(ego2);
            std::cout<<"candidate2 size "<<candidate2.size()<<" init lane "<<getLane(candidate2.d(0),4,2)<<" last lane "<<getLane(candidate2.lastD(),4,2)<<" init s "<<candidate2.s(0)<<" last s "<<candidate2.lastS()<<std::endl;
            start_s = {new_traj.lastS(), new_traj.lastSV(), new_traj.lastSA()};
            end_s = {new_traj.lastS() + v * T, v, 0.0};
            start_d = {new_traj.lastD(), new_traj.lastDV(), new_traj.lastDA()};
            end_d = {new_traj.lastD() - 4.0, 0.0, 0.0};

            Trajectory traj3 = generateTrajectory(start_s, end_s, start_d, end_d, T, eaten_points+cpy_idx);
            Trajectory candidate3 = Trajectory::appendTrajectories(new_traj, traj3);  
            Vehicle ego3;
            ego3.addTrajectory(candidate3);
            ego_cars.push_back(ego3);
            std::cout<<"candidate3 size "<<candidate2.size()<<" init lane "<<getLane(candidate3.d(0),4,2)<<" last lane "<<getLane(candidate3.lastD(),4,2)<<" init s "<<candidate3.s(0)<<" last s "<<candidate3.lastS()<<std::endl;

            traj = candidate1;             
          }

                  

/*

          unsinged seed = std::chrono::system_clock::now().time_since_epoch().count();
          std::default_random_engine generator(seed);
          std::normal_distribution<double> s_distrib(target_s, 2.0);

          for (int i = 0; i<3; i++)
          {
            double new_target_d = 2 + 4*tarj.lastD() * i;
            for (int e = 0; e < 5; e++)
            {
              Trajectory candidate;
              double new_target_s = s_distrib(generator);
              start_s = {traj.lastS(), traj.lastSV(), traj.lastSA()};

              ego_trajs.push_back(traj);
            }            
          }
*/

          
     /*     
          vector<double> coeffs_s = JMT(start_s, end_s, T);
          vector<double> coeffs_d = JMT(start_d, end_d, T);

          for (int i = 1; i<=(eaten_points+cpy_idx); ++i)
          {
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

            double next_sv =      coeffs_s[1]  + 
                              2 * coeffs_s[2] * t + 
                              3 * coeffs_s[3] * t_2 + 
                              4 * coeffs_s[4] * t_3 + 
                              5 * coeffs_s[5] * t_4;

            double next_dv =      coeffs_d[1]  + 
                              2 * coeffs_d[2] * t + 
                              3 * coeffs_d[3] * t_2 + 
                              4 * coeffs_d[4] * t_3 + 
                              5 * coeffs_d[5] * t_4;

            double next_sa =  2 *  coeffs_s[2] + 
                              6 *  coeffs_s[3] * t + 
                              12 * coeffs_s[4] * t_2 + 
                              20 * coeffs_s[5] * t_3;  

            double next_da =  2 *  coeffs_d[2] + 
                              6 *  coeffs_d[3] * t + 
                              12 * coeffs_d[4] * t_2 + 
                              20 * coeffs_d[5] * t_3;                                                           
                                                                                                   
            vector<double> xy = map.toRealWorldXY(next_s, next_d); 

            double yaw = atan2(xy[1]-new_traj.lastY(), xy[0]-new_traj.lastX());          
            
            new_traj.add(xy, {next_s, next_sv, next_sa}, {next_d, next_dv, next_da}, yaw);               
          }        

          traj = new_traj;

*/

 

        /*
         vector<double> cdc = car_distance_cost(ego_cars, others);
         std::cout<<"car distance cost"<<std::endl;
         for (int i = 0; i<cdc.size(); i++)
         {
          if (cdc[i]>0.0)
          {
            std::cout<<"Trajectory "<<i+1<<" "<<cdc[i]<<std::endl;
          }
         }
         std::cout<<std::endl;
         vector<double> lc = lane_cost(ego_cars);
         std::cout<<"lane_cost"<<std::endl;
         for (int i = 0; i<lc.size(); i++)
         {
          std::cout<<"Trajectory "<<i+1<<" "<<lc[i]<<std::endl;
         }
         std::cout<<std::endl;
      */
             

          msgJson["next_x"] = traj.x();
          msgJson["next_y"] = traj.y();

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

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

vector<double> car_distance_cost(vector<Vehicle> ego_cars, vector<Vehicle> other_cars)
{
  vector<double> cost;
  double min_dist = 99999.0;
  bool car_found = false;
  for (Vehicle ego : ego_cars)
  {
    Trajectory ego_tj = ego.getTrajectory();
    if (isLaneValid(getLane(ego_tj.lastD(), 2, 4)))
    {      
      for (Vehicle other : other_cars)
      {  
        Trajectory other_tj = other.getTrajectory();
        if (other_tj.lastS() > ego_tj.lastS() &&
          getLane(other_tj.lastD(),4,2) == getLane(ego_tj.lastD(),4,2))
        {    
          double dist = other_tj.lastS() - ego_tj.lastS();
          if (dist<min_dist)
          {
            min_dist = dist;
            car_found = true;
            std::cout<<"car found"<<std::endl;
          }        
        }       
      }

      std::cout<<"min dist "<<min_dist<<std::endl;
      double c = 1.0 * (2.0 - exp(min_dist/43.3));

      if (!car_found)
        c = 0.0;    
      else if (c > 1.0)
        c = 1.0;
      else if (c < 0.0)
        c = 0.0;
      cost.push_back(c);
      min_dist = 99999.0;
      car_found = false;
    }
    else
    {
      std::cout<<"invalid lane"<<std::endl;
      cost.push_back(1);
    }
  }
  return cost;
}

vector<double> front_car_cost(vector<Vehicle> ego_cars, vector<Vehicle> other_cars)
{
  vector<double> cost;  
  double min_dist = 99999.0;
  bool car_found = false;
  for (Vehicle ego : ego_cars)
  {    
    Trajectory ego_tj = ego.getTrajectory();
    for (Vehicle other : other_cars)
    {  
      Trajectory other_tj = other.getTrajectory();    
      for (int i = 0; i<ego_tj.size(); i++)
      {        
        if ((other_tj.s(i) > ego_tj.s(i)) && 
          (getLane(other_tj.d(i), 4 ,2) == getLane(ego_tj.d(i), 4, 2)))
        {          
          double dist = other_tj.s(i) - ego_tj.s(i);
          if (dist < min_dist)
          {
            min_dist = dist;
            car_found = true;
          }
        }
      }
      
      double c = 1.0 * (2.0 - exp(min_dist/43.3));
    
      if (!car_found)
        c = 0.0;
      else if (c > 1.0)
        c = 1.0;
      else if (c < 0.0)
        c = 0.0;


      cost.push_back(c);      

      min_dist = 99999.0;
      car_found = false;
    }
    
  }  
  
  return cost;
}

vector<double> lane_cost(vector<Vehicle> ego_cars)
{
  vector<double> cost;
  for (Vehicle ego : ego_cars)
  {
    Trajectory ego_tj = ego.getTrajectory();
    if (isLaneValid(getLane(ego_tj.lastD(), 4, 2)))
    {
      if (ego_tj.d(0) != ego_tj.lastD())
      {
        cost.push_back(0.2);
      }
      else
      {
        cost.push_back(0);
      }
    }
    else
    {
      cost.push_back(1);
    }
  }
  return cost;
}

Trajectory generateTrajectory(vector<double> start_s, vector<double> end_s, vector<double> start_d, vector<double> end_d, double T, int traj_size)
{
  Trajectory gen;

  Map &map = Map::getInstance();

  vector<double> coeffs_s = JMT(start_s, end_s, T);
  vector<double> coeffs_d = JMT(start_d, end_d, T);

  for (int i = 1; i<=traj_size; ++i)
  {
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

    double next_sv =      coeffs_s[1]  + 
                      2 * coeffs_s[2] * t + 
                      3 * coeffs_s[3] * t_2 + 
                      4 * coeffs_s[4] * t_3 + 
                      5 * coeffs_s[5] * t_4;

    double next_dv =      coeffs_d[1]  + 
                      2 * coeffs_d[2] * t + 
                      3 * coeffs_d[3] * t_2 + 
                      4 * coeffs_d[4] * t_3 + 
                      5 * coeffs_d[5] * t_4;

    double next_sa =  2 *  coeffs_s[2] + 
                      6 *  coeffs_s[3] * t + 
                      12 * coeffs_s[4] * t_2 + 
                      20 * coeffs_s[5] * t_3;  

    double next_da =  2 *  coeffs_d[2] + 
                      6 *  coeffs_d[3] * t + 
                      12 * coeffs_d[4] * t_2 + 
                      20 * coeffs_d[5] * t_3;                                                           
                                                                                           
    vector<double> xy = map.toRealWorldXY(next_s, next_d); 

    //double yaw = atan2(xy[1]-gen.lastY(), xy[0]-gen.lastX());          
    
    gen.add(xy, {next_s, next_sv, next_sa}, {next_d, next_dv, next_da}, 0.0);               
  }  

  return gen;

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