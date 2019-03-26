#include "trajectory.h"

Trajectory::Trajectory()
{

}

Trajectory::~Trajectory() 
{

}

void Trajectory::add(vector<double> xy, vector<double> s_state, vector<double> d_state, double yaw)
{
	_x_s.push_back(xy[0]);
	_y_s.push_back(xy[1]);

	_s_s.push_back(s_state[0]);
	_s_v.push_back(s_state[1]);
	_s_a.push_back(s_state[2]);
	//_s_j.push_back(s_state[3]);

	_d_s.push_back(d_state[0]);
	_d_v.push_back(d_state[1]);
	_d_a.push_back(d_state[2]);
	//_d_j.push_back(d_state[3]);

	_yaws.push_back(yaw);
}

void Trajectory::removeN(int n)
{
	_x_s.erase(_x_s.begin(), _x_s.begin()+n);
	_y_s.erase(_y_s.begin(), _y_s.begin()+n);

	_s_s.erase(_s_s.begin(), _s_s.begin()+n);
	_s_v.erase(_s_v.begin(), _s_v.begin()+n);
	_s_a.erase(_s_a.begin(), _s_a.begin()+n);
	//_s_j.erase(_s_j.begin(), _s_j.begin()+n);

	_d_s.erase(_d_s.begin(), _d_s.begin()+n);
	_d_v.erase(_d_v.begin(), _d_v.begin()+n);
	_d_a.erase(_d_a.begin(), _d_a.begin()+n);
	//_d_j.erase(_d_j.begin(), _d_j.begin()+n);

	_yaws.erase(_yaws.begin(), _yaws.begin()+n-1);
}

Trajectory Trajectory::copyUpTo(int idx)
{
	Trajectory ret;
	for (int i = 0; i<idx; i++)
	{		
		ret.add({_x_s[i], _y_s[i]}, {_s_s[i], _s_v[i], _s_a[i]}, {_d_s[i], _d_v[i], _d_a[i]}, _yaws[i]);
	}

	return ret;
}

Trajectory Trajectory::appendTrajectories(Trajectory ap1, Trajectory ap2)
{
	Trajectory ret = ap1;

	for (int i = 0; i<ap2.size(); i++)
	{
		ret.add({ap2.x(i), ap2.y(i)},
				{ap2.s(i), ap2.sv(i), ap2.sa(i)},
				{ap2.d(i), ap2.dv(i), ap2.da(i)},
				ap2.yaw(i));
	}

	return ret;
}

int Trajectory::size(void)
{
	return _x_s.size();
}

vector<double> Trajectory::x(void)
{
	return _x_s;
}

vector<double> Trajectory::y(void)
{
	return _y_s;
}

double Trajectory::x(int idx)
{
	double ret = 0.0;
	if (_x_s.size()>0)
		ret =_x_s[idx]; 
	return ret;
}

double Trajectory::y(int idx)
{
	double ret = 0.0;
	if (_y_s.size()>0)
		ret =_y_s[idx]; 
	return ret;
}

double Trajectory::s(int idx)
{
	double ret = 0.0;
	if (_s_s.size()>0)
		ret =_s_s[idx]; 
	return ret;
}

double Trajectory::d(int idx)
{
	double ret = 0.0;
	if (_d_s.size()>0)
		ret =_d_s[idx]; 
	return ret;
}

double Trajectory::sv(int idx)
{
	double ret = 0.0;
	if (_s_v.size()>0)
		ret =_s_v[idx]; 
	return ret;
}

double Trajectory::dv(int idx)
{
	double ret = 0.0;
	if (_d_v.size()>0)
		ret =_d_v[idx]; 
	return ret;
}

double Trajectory::sa(int idx)
{
	double ret = 0.0;
	if (_s_a.size()>0)
		ret =_s_a[idx]; 
	return ret;
}


double Trajectory::da(int idx)
{
	double ret = 0.0;
	if (_d_a.size()>0)
		ret =_d_a[idx]; 
	return ret;
}

double Trajectory::yaw(int idx)
{
	double ret = 0.0;
	if (_yaws.size()>0)
		ret =_yaws[idx]; 
	return ret;
}

double Trajectory::lastX(void)
{
	double ret = 0.0;
	if (_x_s.size()>0)
		ret =_x_s[_x_s.size()-1];
	return ret;
}

double Trajectory::lastY(void)
{
	double ret = 0.0;
	if (_y_s.size()>0)
		ret =_y_s[_y_s.size()-1];
	return ret;
}

double Trajectory::lastS(void)
{
	double ret = 0.0;
	if (_s_s.size()>0)
		ret =_s_s[_s_s.size()-1];
	return ret;
}

double Trajectory::lastD(void)
{
	double ret = 0.0;
	if (_d_s.size()>0)
		ret =_d_s[_d_s.size()-1];
	return ret;
}

double Trajectory::lastSV(void)
{
	double ret = 0.0;
	if (_s_v.size()>0)
		ret =_s_v[_s_v.size()-1];
	return ret;
}

double Trajectory::lastDV(void)
{
	double ret = 0.0;
	if (_d_v.size()>0)
		ret =_d_v[_d_v.size()-1];
	return ret;
}

double Trajectory::lastSA(void)
{
	double ret = 0.0;
	if (_s_a.size()>0)
		ret =_s_a[_s_a.size()-1];
	return ret;
}

double Trajectory::lastDA(void)
{
	double ret = 0.0;
	if (_d_a.size()>0)
		ret =_d_a[_d_a.size()-1];
	return ret;
}