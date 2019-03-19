#include "behaviour.h"

static const double CONTROLLER_UPDATE_RATE_SECONDS = 0.02;

Behaviour::Behaviour()
{
	_init = false;
	_traj = Trajectory();
}

Behaviour::~Behaviour()
{

}


Trajectory Behaviour::nextTrajectory(const Vehicle &ego, const vector<Vehicle>& others, vector<double> &ppx, vector<double> &ppy)
{
	
	int idx = 0;

	if (_init)
	{
		_init = true;

//		_traj.add({ego.x, ego.y}, {ego.s, 0.0, 0.0, 0.0}, {ego.d, 0.0, 0.0, 0.0}, ego.theta);

	}

	if (ppx.size() > 0)
	{
		_traj.removeN(ppx.size() - _traj.getSize());
	}


	double speed_at_index = _traj.getSV(idx);
	double target_s_vel = 1.05*speed_at_index;
	if (target_s_vel > 21)
	{
		target_s_vel = 21;
	}
	double target_s = _traj.getS(idx) + target_s_vel * 1.7;

	double target_d = _traj.getD(idx);

	double target_d_vel = 0.0;
	double target_s_acc = 0.0;
	double target_d_acc = 0.0;


    vector<double> start_s = {_traj.getLastS(), _traj.getLastSV(), _traj.getLastSA()};
    vector<double> end_s = {target_s, target_d_vel, target_s_acc};

    vector<double> start_d = {_traj.getLastD(), _traj.getLastDV(), _traj.getLastDA()};
    vector<double> end_d = {target_d, target_d_vel, target_d_acc};

    vector<double> coeffs_s = this->JMT(start_s, end_s, 1.7);
    vector<double> coeffs_d = this->JMT(start_d, end_d, 1.7);

	//appendPath(start_s, end_s, start_d, end_s, _traj, 1.7);

	return _traj;
}


void Behaviour::appendPath(vector<double> start_s, vector<double> end_s, vector<double> start_d, vector<double> end_d, Trajectory &traj, double T)
{
	vector<double> coeffs_s = JMT(start_s, end_s, T);
	vector<double> coeffs_d = JMT(start_d, end_d, T);

	int total_points = T/ CONTROLLER_UPDATE_RATE_SECONDS;

	int points_remaining = total_points - traj.getSize();
	Map &map = Map::getInstance();

	double last_x = traj.getLastX();
	double last_y = traj.getLastY();

	double last_s = traj.getLastS();
	double last_d = traj.getLastD();

	for (int i = 0; i < points_remaining; ++i)
	{
		double t = CONTROLLER_UPDATE_RATE_SECONDS * (i + 1);
		double t_2 = pow(t, 2);
		double t_3 = pow(t, 3);
		double t_4 = pow(t, 4);
		double t_5 = pow(t, 5);

		double s_t = start_s[0] + start_s[1] * t + 0.5 * start_s[2] * t_2 + coeffs_s[3] * t_3 + coeffs_s[4] * t_4 + coeffs_s[5] * t_5;
        double s_t_dot = start_s[1] + start_s[2] * t + 3 * coeffs_s[3] * t_2 + 4 * coeffs_s[4] * t_3 + 5 * coeffs_s[5] * t_4;
        double s_t_dot_dot = start_s[2] + 6 * coeffs_s[3] * t + 12 * coeffs_s[4] * t_2 + 20 * coeffs_s[5] * t_3;
        double s_jerk = 6 * coeffs_s[3] + 24 * coeffs_s[4] * t + 60 * coeffs_s[5] * t_2;

        double d_t = start_d[0] + start_d[1] * t + start_d[2] * 0.5 * t_2 + coeffs_d[3] * t_3 + coeffs_d[4] * t_4 + coeffs_d[5] * t_5;
        double d_t_dot = start_d[1] + start_d[2] * t + 3 * coeffs_d[3] * t_2 + 4 * coeffs_d[4] * t_3 + 5 * coeffs_d[5] * t_4;
        double d_t_dot_dot = start_d[2] + 6 * coeffs_d[3] * t + 12 * coeffs_d[4] * t_2 + 20 * coeffs_d[5] * t_3;
        double d_jerk = 6 * coeffs_d[3] + 24 * coeffs_d[4] * t + 60 * coeffs_d[5] * t_2;

        vector<double> xy = map.toRealWorldXY(s_t, d_t);
        double x = xy[0];
        double y = xy[1];
        double theta = atan2(y - last_y, x - last_x);

        traj.add(xy, {s_t, s_t_dot, s_t_dot_dot, s_jerk},
                     {d_t, d_t_dot, d_t_dot_dot, d_jerk},
                     theta);
	}
}

vector<double> Behaviour::JMT(vector<double> &start, vector<double> &end, double T)
{
	MatrixXd A = MatrixXd(3,3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
	   3*T*T, 4*T*T*T,5*T*T*T*T,
	   6*T, 12*T*T, 20*T*T*T;

	 std::cout<<A<<std::endl;

	MatrixXd B = MatrixXd(3,1);     
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
	   end[1]-(start[1]+start[2]*T),
	   end[2]-start[2];

	   std::cout<<B<<std::endl;
	      
	MatrixXd Ai = A.inverse();

	MatrixXd C = Ai*B;

	vector <double> result = {start[0], start[1], .5*start[2]};

	for(int i = 0; i < C.size(); ++i) 
	{
		result.push_back(C.data()[i]);
	}

	return result;
}
