#include "trajectory.h"

Trajectory::Trajectory()
{

}

Trajectory::~Trajectory() 
{

}

void Trajectory::add(vector<double> xy, vector<double> s_state, vector<double> d_state, double yaw)
{
	_xs.push_back(xy[0]);
	_ys.push_back(xy[1]);

	_s_s.push_back(s_state[0]);
	_s_v.push_back(s_state[1]);
	_s_a.push_back(s_state[2]);
	_s_j.push_back(s_state[3]);

	_d_s.push_back(d_state[0]);
	_d_v.push_back(d_state[1]);
	_d_a.push_back(d_state[2]);
	_d_j.push_back(d_state[3]);

	_yaws.push_back(yaw);
}

void Trajectory::removeN(int n)
{
	_xs.erase(_xs.begin(), _xs.begin()+n);
	_ys.erase(_ys.begin(), _ys.begin()+n);

	_s_s.erase(_s_s.begin(), _s_s.begin()+n);
	_s_v.erase(_s_v.begin(), _s_v.begin()+n);
	_s_a.erase(_s_a.begin(), _s_a.begin()+n);
	_s_j.erase(_s_j.begin(), _s_j.begin()+n);

	_d_s.erase(_s_s.begin(), _s_s.begin()+n);
	_d_v.erase(_s_v.begin(), _s_v.begin()+n);
	_d_a.erase(_s_a.begin(), _s_a.begin()+n);
	_d_j.erase(_s_j.begin(), _s_j.begin()+n);

	_yaws.erase(_yaws.begin(), _yaws.begin()+n);
}

double Trajectory::getX(int idx)
{
	return _xs[idx];
}

double Trajectory::getY(int idx)
{
	return _ys[idx];
}

double Trajectory::getS(int idx)
{
	return _s_s[idx];
}

double Trajectory::getD(int idx)
{
	return _d_s[idx];
}

double Trajectory::getSV(int idx)
{
	return _s_v[idx];
}

double Trajectory::getDV(int idx)
{
	return _d_v[idx];
}

int Trajectory::getSize(void)
{
	return _xs.size();
}

double Trajectory::getLastX(void)
{
	return _xs[_xs.size()-1];
}

double Trajectory::getLastY(void)
{
	return _ys[_ys.size()-1];
}

double Trajectory::getLastS(void)
{
	return _s_s[_s_s.size()-1];
}

double Trajectory::getLastD(void)
{
	return _d_s[_d_s.size()-1];
}

double Trajectory::getLastSV(void)
{
	return _s_v[_s_v.size()-1];
}

double Trajectory::getLastDV(void)
{
	return _d_v[_d_v.size()-1];
}

double Trajectory::getLastSA(void)
{
	return _s_a[_s_a.size()-1];
}

double Trajectory::getLastDA(void)
{
	return _d_a[_d_a.size()-1];
}