#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <iostream>
#include <vector>

using std::vector;

class Trajectory
{

public:

	Trajectory();

	~Trajectory();

	void add(vector<double> xy, vector<double> s_state, vector<double> d_state, double yaw);
	void removeN(int n);
	Trajectory copyUpTo(int idx);
	
	static Trajectory appendTrajectories(Trajectory ap1, Trajectory ap2);

	vector<double> x(void);
	vector<double> y(void);
	double x(int idx);
	double y(int idx);
	double s(int idx);
	double d(int idx);
	double sv(int idx);		
	double dv(int idx);
	double sa(int idx);		
	double da(int idx);
	double yaw(int idx);

	int size(void);

	double lastX(void);
	double lastY(void);	
	double lastS(void);
	double lastD(void);	
	double lastSV(void);
	double lastDV(void);
	double lastSA(void);
	double lastDA(void);

private:

	vector<double> _x_s;
	vector<double> _y_s;
	vector<double> _s_s;
	vector<double> _s_v;
	vector<double> _s_a;
	vector<double> _s_j;
	vector<double> _d_s;
	vector<double> _d_v;
	vector<double> _d_a;
	vector<double> _d_j;
	vector<double> _yaws;
};

#endif
