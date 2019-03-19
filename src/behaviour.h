#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

#include "trajectory.h"
#include "vehicle.h"
#include "map.h"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"

#include <iostream>
#include <vector>
#include <math.h>

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class Behaviour
{

public:
	
	Behaviour();

	~Behaviour();

	Trajectory nextTrajectory(const Vehicle &ego, const vector<Vehicle>& others, vector<double> &ppx, vector<double> &ppy);

	void appendPath(vector<double> start_s, vector<double> end_s, vector<double> start_d, vector<double> end_d, Trajectory &traj, double T);
	vector<double> JMT(vector<double> &start, vector<double> &end, double T);


private:

	bool _init;
	Trajectory _traj;

};

#endif
