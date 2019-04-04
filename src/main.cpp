#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <random>
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
double final_velocity_cost(Trajectory ego_tj);
double max_velocity_cost(Trajectory ego_tj);
double acceleration_peak_cost(Trajectory ego_tj);
double acceleration_avg_cost(Trajectory ego_tj);
double car_ahead_cost(Trajectory ego_tj, vector<Vehicle> other_cars);
double driving_forward_cost(Trajectory ego_tj);
double change_lane_cost(Trajectory ego_tj);
double prohibited_lane_cost(Trajectory ego_tj);
double distance_to_vehicle(Trajectory ego_tj, vector<Vehicle> other_cars);
double inside_lane_cost(Trajectory ego_tj);
double nocars_ahead_cost(Trajectory ego_tj, vector<Vehicle> other_cars);
double change_safe_cost(Trajectory ego_tj, vector<Vehicle> other_cars);

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


          int in_sov = 0;
          vector<Vehicle> others;
          for (auto v_data : sensor_fusion)
          {
            Vehicle v = Vehicle(v_data[0], v_data[1], v_data[2], v_data[3], v_data[4], v_data[5], v_data[6]);

            if (v.s() < (car_s + 70.0) && v.s() > (car_s - 30.0) && v.d() > 0.0)
            {
              others.push_back(v);       
              others[in_sov].propagate(100);    
              // std::cout<<"car id "<<v.id()<<" propagate "<<pp_size<< " init s " << others[in_sov].getTrajectory().s(0)<<" last s "<<others[in_sov].getTrajectory().lastS();
              //std::cout<<"id: "<<v.id()<<" lane "<< v.d()<< " " << getLane(v.d(),4,2)<<"\t";              
              in_sov++;
            }
          }
          if (others.size()>0)
          {
          //std::cout<<std::endl;
          //std::cout<<std::endl;
        }

          //std::cout << "pp size " << pp_size << std::endl;

          /*
          std::cout<<"previous path"<<std::endl;      
          for (int i = 0; i < pp_size; ++i)
          {
            std::cout<<previous_path_x[i]<<" ";
          }
          std::cout<<std::endl;
          std::cout<<std::endl;
          */


          double T = 2.0;


          int eaten_points = traj.size() - pp_size;
          int cpy_idx = 0;
          static bool init = true;
          static double acceleration_peak_cost_k = 1;
          if (init)
          {
            eaten_points = T/0.02;
            init = false;
            acceleration_peak_cost_k = 15;
          }

          if (acceleration_peak_cost_k > 1.0)
          {
            acceleration_peak_cost_k -= 0.1;
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
            end_s = {car_s + mph2mps(10)*T, mph2mps(10), 0.0};
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
            static double chaning_lane_cost_k = 30;
            traj.removeN(eaten_points);

            cpy_idx = 30;            

            new_traj = traj.copyUpTo(cpy_idx);                       

            int current_lane = getLane(car_d,4,2);

            
            if ( getLane(new_traj.lastD(),4,2) != current_lane)
            {
              std::cout<< "CHANGING LANE"<<std::endl;
              chaning_lane_cost_k = 5.0;
            }

            // if (chaning_lane_cost_k >0.5)
            // {
            //   chaning_lane_cost_k -= 0.01;
            // }

            double target_s = new_traj.lastS() + new_traj.lastSV() * T;
            double target_d = getLaneCenterFrenet(getLane(car_d,4,2));
            
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine generator(seed);
            std::normal_distribution<double> s_distrib(target_s, target_s-car_s);
            //std::cout<<target_s-car_s<<std::endl;
            std::normal_distribution<double> d_distrib(target_d, 0.2);
            vector<Trajectory> candidates;

            //std::cout<<target_s<<" "<<new_traj.lastS()<<" "<<ceil((target_s-new_traj.lastS())*0.5)<<std::endl;

            //std::cout<<"last S "<<new_traj.lastS()<<" and final S "<<target_s<<std::endl;
            for (int i = 0; i<50; i++)
            {                          
              double new_target_s = s_distrib(generator);
              double new_target_sv = (new_target_s -  new_traj.lastS())/T;
              if (new_target_sv > 49.9)
              {
                new_target_sv = 49.9;
              }
              double new_target_d = d_distrib(generator);
             // std::cout<<" new target s "<<new_target_s<<std::endl;
              Trajectory generated = generateTrajectory(  {new_traj.lastS(), new_traj.lastSV(), new_traj.lastSA()},
                                                          {new_target_s, new_target_sv, 0.0},
                                                          {new_traj.lastD(), new_traj.lastDV(), new_traj.lastDA()},
                                                          {new_target_d, 0.0, 0.0},
                                                          T, 
                                                          round(T/0.02)-cpy_idx);
              candidates.push_back(Trajectory::appendTrajectories(new_traj, generated));
          
              generated = generateTrajectory(   {new_traj.lastS(), new_traj.lastSV(), new_traj.lastSA()},
                                                {new_target_s, new_target_sv, 0.0},
                                                {new_traj.lastD(), new_traj.lastDV(), new_traj.lastDA()},
                                                {new_target_d+4, 0.0, 0.0},
                                                T, 
                                                round(T/0.02)-cpy_idx);
              candidates.push_back(Trajectory::appendTrajectories(new_traj, generated));
            
              generated = generateTrajectory(   {new_traj.lastS(), new_traj.lastSV(), new_traj.lastSA()},
                                                {new_target_s, new_target_sv, 0.0},
                                                {new_traj.lastD(), new_traj.lastDV(), new_traj.lastDA()},
                                                {new_target_d-4, 0.0, 0.0},
                                                T, 
                                                round(T/0.02)-cpy_idx);
              candidates.push_back(Trajectory::appendTrajectories(new_traj, generated));
          
            }

            Trajectory selected;
            double total_cost = 9999;
            double sel_fvc;  
            double sel_mvc;  
            double sel_apc;  
            double sel_avc;  
            double sel_cac;  
            double sel_dfc;  
            double sel_clc;  
            double sel_plc;  
            double sel_dvc;  
            double sel_ilc; 
            double sel_ncc;  
            double sel_csc;  
            for(Trajectory tr : candidates)
            {
              double fvc = 1.0*final_velocity_cost(tr);
              double mvc = 5.0*max_velocity_cost(tr);
              double apc = acceleration_peak_cost_k*acceleration_peak_cost(tr);
              double avc = 1.0*acceleration_avg_cost(tr);
              double cac = 2.0*car_ahead_cost(tr, others);
              double dfc = 100.0*driving_forward_cost(tr);
              double clc = 0.01*change_lane_cost(tr);
              double plc = 100.0*prohibited_lane_cost(tr);
              double dvc = 50.0*distance_to_vehicle(tr, others);
              double ilc = 0.1*inside_lane_cost(tr);
              double ncc = 0.5*nocars_ahead_cost(tr, others);
              double csc = 10.0*change_safe_cost(tr, others);
              double sum =  
                            0
                            +fvc
                            +mvc
                            +apc
                            //+avc
                            +cac
                            +dfc
                            +clc
                            +plc
                            //+dvc
                            +ilc
                            //+ncc
                            +csc
                            +0;                            
              //std::cout<<"cost cac: "<<cac<<std::endl;
              //std::cout<<"cost fvc: "<<fvc<<"   cost apc: "<<apc<<"   cost avc: "<<avc <<"   SUM: "<<sum<<std::endl;
              if (sum < total_cost)
              {
                total_cost = sum;
                selected = tr;  
                sel_fvc = fvc;
                sel_mvc = mvc;
                sel_apc = apc;
                sel_avc = avc;
                sel_cac = cac;
                sel_dfc = dfc;
                sel_clc = clc;    
                sel_plc = plc; 
                sel_dvc = dvc;   
                sel_ilc = ilc;   
                sel_ncc = ncc;  
                sel_csc = csc;   
              }
            }

            if(sel_csc>0.0)
            {
               std::cout<<std::fixed;
             std::cout<<std::setprecision(3);
             std::cout<<
                     "sel_csc:  " <<sel_csc;
                   std::cout<<std::endl;
            }
            // if (mph2mps(car_speed)*1.1<(selected.lastSV()))
            // {              
            //   std::cout<<"accelerating "<<mph2mps(car_speed) << " " << selected.lastSV()<<std::endl;
            // }
            // else if (mph2mps(car_speed)/1.1>(selected.lastSV()))
            // {
            //  std::cout<<"decelerating "<<mph2mps(car_speed) << " " << selected.lastSV()<<std::endl;
            // }
            // if (sel_dvc > 0.0)
            // {
            //      std::cout<<std::fixed;
            //  std::cout<<std::setprecision(3);
            //  std::cout<<
            //          "\tsel_dvc:  " <<sel_dvc;
            //        std::cout<<std::endl;
            // }
            
            // std::cout<<"selected cost "<<total_cost<<std::endl;
             // std::cout<<std::fixed;
             // std::cout<<std::setprecision(3);
             // std::cout<<
             // "vel:  "          <<sel_fvc<<
            // "\tacc peak:  "   <<sel_apc<<
            // "\tacc avg:  "    <<sel_avc<<
            // "\tcar ahead:  "  <<sel_cac<<
            // "\tforward:  "    <<sel_dfc<<
             // "\tchange lane:  "<<sel_clc<<
            // "\tprohibited:  " <<sel_plc<<
             // "\tany car dis:  " <<sel_dvc
             // <<std::endl;
            // std::cout<<std::endl;

            traj = selected;

          }

    

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
    if (getLane(ego_tj.lastD(), 4, 2)>=0 && getLane(ego_tj.lastD(), 4, 2)<=2)
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
            //std::cout<<"car found"<<std::endl;
          }        
        }       
      }

      //std::cout<<"min dist "<<min_dist<<std::endl;
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
    if (getLane(ego_tj.lastD(), 4, 2)>=0 && getLane(ego_tj.lastD(), 4, 2)<=2)
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

double final_velocity_cost(Trajectory ego_tj)
{
  double target_vel = mph2mps(48.5);
  double v_lim = mph2mps(49.9);
  double final_vel = ego_tj.lastSV();
  double cost = 1.0;
  //std::cout<<"final velocity cost "<<final_vel<<std::endl;
  if (final_vel < target_vel)
  {
    cost = 1.0 - final_vel/target_vel;
    //cost = -exp((final_vel-target_vel)/10.0) + 1.0;
  }
  else if(final_vel >target_vel && final_vel< v_lim)
  {
    double m = 1.0/(v_lim-target_vel);
    double b = 1.0 - m*v_lim;
    cost = (m*final_vel+b);
    //cost = exp(7*(-v_lim+final_vel));
  }
  return cost;
}

double max_velocity_cost(Trajectory ego_tj)
{
  double target_vel = mph2mps(47.0);
  double v_lim = mph2mps(49.5);
  double vel = 0.0;
  double cost = 0.0;

  for (int i = 0; i<ego_tj.size(); i++)
  {
    double cost_temp = 1.0;
    vel = ego_tj.sv(i);
       //std::cout<<"final velocity cost "<<final_vel<<std::endl;
    if (vel < target_vel)
    {
      cost_temp = 0.0;
      //cost = -exp((final_vel-target_vel)/10.0) + 1.0;
    }
    else
    {
      double m = 1.0/(v_lim-target_vel);
      double b = 1.0 - m*v_lim;
      cost_temp = (m*vel+b);
      //cost = exp(7*(-v_lim+final_vel));
    }
    if (cost_temp > 1.0)
      cost_temp = 1.0;

    if (cost_temp > cost)
    {
      cost = cost_temp;
    }
  }
 
  return cost;
}

double acceleration_peak_cost(Trajectory ego_tj)
{
  double cost = 0;
  //std::cout<<"acc"<<std::endl;
  for (int i = 0; i<ego_tj.size();i++)
  {
    double acc = abs(ego_tj.sa(i));
   // std::cout<<" "<<acc;
    double cost_temp =  1.0 - exp(-(acc)/10.0);
    if (cost_temp>1.0)
    {
      cost_temp = 1.0;
    }  
    else if (cost_temp<0.0)
    {
      cost_temp = 0.0;
    }

    if (cost_temp > cost)
    {
      cost = cost_temp;
    } 
  }
  //std::cout<<std::endl;
  return cost;
}

double acceleration_avg_cost(Trajectory ego_tj)
{
  double cost = 0;
  double avg = 0;
  //std::cout<<"acc"<<std::endl;
  for (int i = 0; i<ego_tj.size();i++)
  {
    double acc = abs(ego_tj.sa(i));
   // std::cout<<" "<<acc;
    avg += acc;   
  }
  cost = avg/ego_tj.size()/20.0;

  if (cost > 1.0)
  {
    cost = 1.0;
  }
  //std::cout<<std::endl;
  return cost;
}

double driving_forward_cost(Trajectory ego_tj)
{
  double cost = 0.0;

  if (ego_tj.lastS()<ego_tj.s(0))
  {
    //std::cout<<"driving backwards!!"<<std::endl;
    cost = 1.0;
  }

  return cost;
}

double car_ahead_cost(Trajectory ego_tj, vector<Vehicle> other_cars)
{
  double cost = 0.0;
  for (Vehicle other : other_cars)
  {  
    Trajectory other_tj = other.getTrajectory();
    int other_lane = getLane(other_tj.lastD(), 4, 2);
      for (int i = 0; i<ego_tj.size(); i++)
      {        
        if (getLane(ego_tj.d(i),4,2) == other_lane)      
        {
          double dist = abs(other_tj.s(i) - ego_tj.s(i));
          double cost_temp = exp(-0.3 * dist);
          if (cost_temp > 1.0)
          {
            cost_temp = 1.0;
          }
          else if (cost_temp < 0.0)
          {
            cost_temp = 0.0;
          }
          if (cost_temp > cost)
          {
            cost = cost_temp;
          }
        }
     }    
  }        
  return cost;
}

double change_lane_cost(Trajectory ego_tj)
{
  double cost = 0.0;
  if(getLane(ego_tj.lastD(),4,2) != getLane(ego_tj.d(0),4,2))
  {
    cost = 1.0;
  }
  return cost;
}

double prohibited_lane_cost(Trajectory ego_tj)
{
  double cost = 0.0;
  if(getLane(ego_tj.lastD(),4,1.8)<0 || getLane(ego_tj.lastD(),4,1.8)>2)
  {
    cost = 1.0;
  }
  return cost;
}

double distance_to_vehicle(Trajectory ego_tj, vector<Vehicle> other_cars)
{
  double cost = 0.0;
  double max_dist = 3.0;
  for (Vehicle other : other_cars)
  {     
    Trajectory other_tj = other.getTrajectory(); 
    for (int i = 0; i<other_tj.size();i++)
    {
      double o_s = other_tj.s(i);
      double o_d = other_tj.d(i);
      double e_s = ego_tj.s(i);
      double e_d = ego_tj.d(i);

      double cost_temp = 0.0;
      if(abs(o_d - e_d)<1 && abs(o_s - e_s)<2)
      {
        cost_temp = 1.0;
      }

      //double dist = sqrt((o_x-e_x)*(o_x-e_x) + (o_y-e_y)*(o_y-e_y));

      //double cost_temp = 1.0 - exp((dist - max_dist)/1.0);
      //if (cost_temp>1.0)
      //{
      //   cost_temp = 1.0;
      // }
      // else if (cost_temp < 0.0)
      // {
      //   cost_temp = 0.0;
      // }

      if (cost_temp>cost)
      {
        cost = cost_temp;
      }
    
    }      
  }
  return cost;
}

double inside_lane_cost(Trajectory ego_tj)
{
  double cost = 0.0;

  for (int i = 0; i<ego_tj.size();i++)
  {
    double d = ego_tj.d(i);
    double lane_d = getLaneCenterFrenet(getLane(d, 4, 2));
    double error = abs(lane_d - d);
    double cost_temp = error/4.0;

    if (cost_temp > 1.0)
    {
      cost_temp = 1.0;
    }
    if (cost_temp > cost)
    {
      cost = cost_temp;
    }
  }
  return cost;
}

double nocars_ahead_cost(Trajectory ego_tj, vector<Vehicle> other_cars)
{
  double cost = 0.0;
  double max_dist = 10.0;
  for (Vehicle other : other_cars)
  {  
    Trajectory other_tj = other.getTrajectory();
    int other_lane = getLane(other_tj.lastD(), 4, 2);
    int ego_lane = getLane(ego_tj.lastD(), 4, 2);
    if (other_lane == ego_lane &&
        other_tj.lastS() > ego_tj.lastS())
    {
     cost = 1.0;
    }      
  }
  return cost;
}

double change_safe_cost(Trajectory ego_tj, vector<Vehicle> other_cars)
{
  double cost = 0.0;
  int ego_lane_1 = getLane(ego_tj.d(0), 4, 2);
  int ego_lane_2 = getLane(ego_tj.lastD(), 4, 2);
  if (ego_lane_1 != ego_lane_2)
  {
    for (Vehicle other : other_cars)
    {  
      Trajectory other_tj = other.getTrajectory();
      int other_lane = getLane(other_tj.lastD(), 4, 2);
      if (ego_lane_2 == other_lane)
      {
        double ego_s = ego_tj.lastS();
        double other_s = other_tj.lastS();
        double ego_d = ego_tj.lastD();
        double other_d = other_tj.lastD();
        double dist = sqrt((ego_s - other_s)*(ego_s - other_s)+(ego_d - other_d)*(ego_d - other_d));
        double cost_temp = -dist/20 + 1;
        if (cost_temp<0.0)
        {
          cost_temp = 0.0;
        }
        else if (cost_temp>1.0)
        {
          cost_temp = 1.0;
        }
        if (cost_temp>cost)
        {
          cost = cost_temp;
        }
      }
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