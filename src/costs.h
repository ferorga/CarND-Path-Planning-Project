#ifndef COSTS_H
#define COSTS_H

#include <math.h>
#include "trajectory.h"
#include "vehicle.h"
#include "helpers.h"

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

#endif