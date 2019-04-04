#include "vehicle.h"
#include "map.h"
#include <math.h>

using namespace std;

Vehicle::Vehicle() {}

Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s, double d)
{
    _id = id;
    _x = x;
    _y = y;
    _vx = vx;
    _vy = vy;
    _s = s;
    _d = d;    
    _v = sqrt(vx*vx + vy*vy);
}

void Vehicle::propagate(int n)
{  
    Trajectory prop;
    Map &map = Map::getInstance();
    for( int i = 0; i < n; i++)
    {
        double next_s = _s + _d * 0.02*i;       
        vector<double> xy = map.toRealWorldXY(next_s, _d);    
        prop.add(xy, {next_s, _v, 0.0}, {_d, 0.0, 0.0}, 0.0);        
    }
    //std::cout<<"init prop s "<<prop.s(0)<<" last prop s "<<prop.lastS()<<std::endl;
    //std::cout<<"init prop sv "<<prop.sv(0)<<" last prop sv "<<prop.lastSV()<<std::endl;
    _traj = prop;
}

int Vehicle::id(void)
{
    return _id;
}

double Vehicle::d(void)
{
    return _d;
}

double Vehicle::s(void)
{
    return _s;
}

double Vehicle::x(void)
{
    return _x;
}

double Vehicle::y(void)
{
    return _y;
}

double Vehicle::vx(void)
{
    return _vx;
}

double Vehicle::vy(void)
{
    return _vy;
}

double Vehicle::v(void)
{
    return _v;
}

void Vehicle::addTrajectory(Trajectory traj)
{
    _traj = traj;
}

Trajectory Vehicle::getTrajectory(void)
{
    return _traj;
}

int Vehicle::trajectorySize(void)
{
    return _traj.size();
}

Vehicle::~Vehicle() {}