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

	double getX(int idx);
	double getY(int idx);
	double getS(int idx);
	double getD(int idx);
	double getSV(int idx);		
	double getDV(int idx);
	double getSA(int idx);		
	double getDA(int idx);

	int getSize(void);

	double getLastX(void);
	double getLastY(void);	
	double getLastS(void);
	double getLastD(void);
	double getLastSV(void);
	double getLastDV(void);
	double getLastSA(void);
	double getLastDA(void);

private:

	vector<double> _xs;
	vector<double> _ys;
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
