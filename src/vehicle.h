#ifndef VEHICLE_H
#define VEHICLE_H
#include <vector>
#include "trajectory.h"

using namespace std;

class Vehicle : public Trajectory
{
    public:
        int _id;
        
        double _x;
        double _y;
        
        double _vx;
        double _vy;
        
        double _s;
        double _d;

        double _v;

        Trajectory _traj;

        /**
         * Constructors
         */
        Vehicle(); 
        Vehicle(int id, double x, double y, double vx, double vy, double s, double d);

        void propagate(int n);

        int id(void);
        double d(void);           
        double s(void);      
        double x(void);      
        double y(void);      
        double vx(void);     
        double vy(void);   
        double v(void);     

        void addTrajectory(Trajectory traj);
        Trajectory getTrajectory(void);
        int trajectorySize(void);

        /**
        * Destructor
        */
        virtual ~Vehicle();
};
#endif