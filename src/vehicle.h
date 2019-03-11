#ifndef VEHICLE_H
#define VEHICLE_H
#include <vector>
#include "helpers.h"
#include "spline.h"

using std::vector;

class Vehicle {
public:
  // Constructors
  Vehicle();

  // Destructor
  virtual ~Vehicle();

  void Run(double x, double y, double s, double d, double yaw, double speed, vector<double> prev_x, vector<double> prev_y, double end_s, double end_d, vector<vector<double>> sensor);
  void SetMapWaypoints(vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double>map_waypoints_y, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy);
  vector<double> GetNextX();
  vector<double> GetNextY(); 

private:
  vector<double> _next_x;
  vector<double> _next_y;
  vector<double> _map_wp_s;
  vector<double> _map_wp_x;
  vector<double> _map_wp_y;
  vector<double> _map_wp_dx;
  vector<double> _map_wp_dy;

};

#endif  // VEHICLE_H