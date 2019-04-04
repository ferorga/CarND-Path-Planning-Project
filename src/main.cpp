#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <random>
#include "helpers.h"
#include "json.hpp"
#include "map.h"
#include "trajectory.h"
#include "vehicle.h"
#include "costs.h"

#define DEF_FURTHEST_CAR_AHEAD          70
#define DEF_FURTHEST_CAR_BEHIND         30
#define DEF_TRAJECTORY_TIME_HORIZON     2
#define DEF_SIM_DT                      0.02
#define DEF_START_TRAJECTORY_FROM_IDX   30
#define DEF_NUM_CANDIDATE_TRAJECTORIES  50
#define DEF_TARGET_VELOCITY             49.9
#define DEF_D_GAUSS                     0.2

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


int main() {
  uWS::Hub h;

  Trajectory traj; // Main trajectory to keep track of previous points

  Map &map = Map::getInstance();

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

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

          // Other cars detection and trajectory propagation
          int in_sov = 0;
          vector<Vehicle> others;
          for (auto v_data : sensor_fusion)
          {
            Vehicle v = Vehicle(v_data[0], v_data[1], v_data[2], v_data[3], v_data[4], v_data[5], v_data[6]);

            if (v.s() < (car_s + DEF_FURTHEST_CAR_AHEAD) && v.s() > (car_s - DEF_FURTHEST_CAR_BEHIND) && v.d() > 0.0)
            {
              others.push_back(v);       
              others[in_sov].propagate((int)round(DEF_TRAJECTORY_TIME_HORIZON/DEF_SIM_DT));       
              in_sov++;
            }
          }

          double T = DEF_TRAJECTORY_TIME_HORIZON;
          int eaten_points = traj.size() - pp_size;
          int cpy_idx = 0;

          // Initialization
          static bool init = true;
          static double acceleration_peak_cost_k = 1;
          if (init)
          {
            eaten_points = T/DEF_SIM_DT;
            init = false;
            acceleration_peak_cost_k = 15; // Higher weight to the acceleration cost to avoid increasing speed too fast from 0 to "target"
          }

          // Decreasing the acceleration weight cost as times pases
          if (acceleration_peak_cost_k > 1.0)
          {
            acceleration_peak_cost_k -= 0.1;
          }      


          Trajectory new_traj;
          vector<double> start_s, end_s, start_d, end_d, ego_s_d;

          if (pp_size == 0) // Initialization
          {
            // Create a starting trajectory to start moving
            start_s = {car_s, 0.0, 0.0};
            end_s = {car_s + mph2mps(10)*T, mph2mps(10), 0.0};
            start_d = {car_d, 0.0, 0.0};
            end_d = {car_d, 0.0, 0.0};

            vector<double> xy = map.toXY(car_s, car_d);
            traj.add(xy, start_s, start_d, 0.0);
            new_traj.add(xy, start_s, start_d, 0.0);

            Trajectory traj0 = Trajectory::generateTrajectory(start_s, end_s, start_d, end_d, T, eaten_points+cpy_idx);           
            Trajectory candidate0 = Trajectory::appendTrajectories(new_traj, traj0); 

            // Here do not evaluate costs, we just start with this first trajectory
            traj = candidate0; 
          }
          else
          {        
            traj.removeN(eaten_points); // Remove already "eaten" points

            cpy_idx = DEF_START_TRAJECTORY_FROM_IDX; // We generate a new trajectory from this point of the previous one to avoid big jumps and keep continuity             

            new_traj = traj.copyUpTo(cpy_idx);                       

            double target_s = new_traj.lastS() + new_traj.lastSV() * T; // we use a constant velocity model to estimate the target S
            double target_d = getLaneCenterFrenet(getLane(car_d,4,2)); // The target will be the center of the current lane
            
            // Gauss generators to create a bunch of candidate trajectories
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine generator(seed);
            std::normal_distribution<double> s_distrib(target_s, target_s-car_s); // All candidates will be in between the current car S and the target S
            std::normal_distribution<double> d_distrib(target_d, DEF_D_GAUSS);
            
            // Create the vector of candidates that we will evaluate
            vector<Trajectory> candidates;

            for (int i = 0; i<DEF_NUM_CANDIDATE_TRAJECTORIES; i++)
            {                          
              double new_target_s = s_distrib(generator);
              double new_target_sv = (new_target_s -  new_traj.lastS())/T;
              if (new_target_sv > DEF_TARGET_VELOCITY)
              {
                new_target_sv = DEF_TARGET_VELOCITY;
              }
              double new_target_d = d_distrib(generator);
              
              Trajectory generated;

              // Same lane trajectory
              generated = Trajectory::generateTrajectory(   {new_traj.lastS(), new_traj.lastSV(), new_traj.lastSA()},
                                                            {new_target_s, new_target_sv, 0.0},
                                                            {new_traj.lastD(), new_traj.lastDV(), new_traj.lastDA()},
                                                            {new_target_d, 0.0, 0.0},
                                                            T, 
                                                            round(T/DEF_SIM_DT)-cpy_idx);
              candidates.push_back(Trajectory::appendTrajectories(new_traj, generated));
          
              // Right lane trajectory
              generated = Trajectory::generateTrajectory(   {new_traj.lastS(), new_traj.lastSV(), new_traj.lastSA()},
                                                            {new_target_s, new_target_sv, 0.0},
                                                            {new_traj.lastD(), new_traj.lastDV(), new_traj.lastDA()},
                                                            {new_target_d+4, 0.0, 0.0},
                                                            T, 
                                                            round(T/DEF_SIM_DT)-cpy_idx);
              candidates.push_back(Trajectory::appendTrajectories(new_traj, generated));
            
              // Left lane trajectory
              generated = Trajectory::generateTrajectory(   {new_traj.lastS(), new_traj.lastSV(), new_traj.lastSA()},
                                                            {new_target_s, new_target_sv, 0.0},
                                                            {new_traj.lastD(), new_traj.lastDV(), new_traj.lastDA()},
                                                            {new_target_d-4, 0.0, 0.0},
                                                            T, 
                                                            round(T/DEF_SIM_DT)-cpy_idx);
              candidates.push_back(Trajectory::appendTrajectories(new_traj, generated));        
            }

            Trajectory selected;
            double total_cost = 9999;
            double sel_fvc, sel_mvc, sel_apc, sel_avc ,sel_cac, sel_dfc, sel_clc, sel_plc, sel_dvc, sel_ilc, sel_ncc, sel_csc;  
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
        
            // Select the one with the least costs
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



