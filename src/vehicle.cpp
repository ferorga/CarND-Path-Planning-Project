#include "vehicle.h"


Vehicle::Vehicle()
{
}

Vehicle::~Vehicle() 
{
}

void Vehicle::SetMapWaypoints(vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double>map_waypoints_y, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy)
{
	_map_wp_s = map_waypoints_s;
	_map_wp_x = map_waypoints_x;
	_map_wp_y = map_waypoints_y;
	_map_wp_dx = map_waypoints_dx;
	_map_wp_dy = map_waypoints_dy;
}

void Vehicle::Run(double x, double y, double s, double d, double yaw, double speed, vector<double> prev_x, vector<double> prev_y, double end_s, double end_d, vector<vector<double>> sensor)
{
	int lane = 1;
	double ref_vel = 49.5;

	int prev_size = prev_x.size();

	vector<double> ptsx;
	vector<double> ptsy;

	double ref_x = x;
	double ref_y = y;
	double ref_yaw = deg2rad(yaw);

	if (prev_size < 2)
	{
		double prev_car_x = x - cos(yaw);
		double prev_car_y = y - sin(yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(x);
		ptsy.push_back(prev_car_y);
		ptsy.push_back(y);
	}
	else
	{
		ref_x = prev_x[prev_size-1];
		ref_y = prev_y[prev_size-1];

		double ref_x_prev = prev_x[prev_size-2];
		double ref_y_prev = prev_y[prev_size-2];
		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);
		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}

	vector<double> next_wp0 = getXY(s+30, (2+4*lane), _map_wp_s, _map_wp_x, _map_wp_y);
	vector<double> next_wp1 = getXY(s+60, (2+4*lane), _map_wp_s, _map_wp_x, _map_wp_y);
	vector<double> next_wp2 = getXY(s+90, (2+4*lane), _map_wp_s, _map_wp_x, _map_wp_y);

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);
	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	for (int i = 0; i < ptsx.size(); i++)
	{
		double shift_x = ptsx[i]-ref_x;
		double shift_y = ptsy[i]-ref_y;

		ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y * sin(0-ref_yaw));
		ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y * cos(0-ref_yaw));
	}

	vector<double> next_x_vals;
	vector<double> next_y_vals;

	tk::spline trajectory;

	trajectory.set_points(ptsx, ptsy);

	for (int i = 0; i < prev_x.size(); i++)
	{
		next_x_vals.push_back(prev_x[i]);
		next_y_vals.push_back(prev_y[i]);
	}

	double target_x = 30.0;
	double target_y = trajectory(target_x);
	double target_dist = sqrt(target_x*target_x + target_y*target_y);

	double x_add_on = 0;

	for (int i = 1; i<=50-prev_x.size(); i++)
	{
		double N = target_dist/(.02*ref_vel/2.24);
		double x_point = x_add_on + target_x/N;
		double y_point = trajectory(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
		y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

		x_point += ref_x;
		y_point += ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);

		_next_x = next_x_vals;
		_next_y = next_y_vals;
	}
}

vector<double> Vehicle::GetNextX(void)
{
	return _next_x;
}

vector<double> Vehicle::GetNextY(void)
{
	return _next_y;
}