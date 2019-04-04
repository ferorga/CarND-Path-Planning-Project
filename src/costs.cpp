#include "costs.h"

double final_velocity_cost(Trajectory ego_tj)
{
  double target_vel = mph2mps(48.5);
  double v_lim = mph2mps(49.9);
  double final_vel = ego_tj.lastSV();
  double cost = 1.0;
  //std::cout<<"final velocity cost "<<final_vel<<std::endl;
  if (final_vel < target_vel)
  {
    cost = 1.0 - final_vel/target_vel;
    //cost = -exp((final_vel-target_vel)/10.0) + 1.0;
  }
  else if(final_vel >target_vel && final_vel< v_lim)
  {
    double m = 1.0/(v_lim-target_vel);
    double b = 1.0 - m*v_lim;
    cost = (m*final_vel+b);
    //cost = exp(7*(-v_lim+final_vel));
  }
  return cost;
}

double max_velocity_cost(Trajectory ego_tj)
{
  double target_vel = mph2mps(47.0);
  double v_lim = mph2mps(49.5);
  double vel = 0.0;
  double cost = 0.0;

  for (int i = 0; i<ego_tj.size(); i++)
  {
    double cost_temp = 1.0;
    vel = ego_tj.sv(i);
       //std::cout<<"final velocity cost "<<final_vel<<std::endl;
    if (vel < target_vel)
    {
      cost_temp = 0.0;
      //cost = -exp((final_vel-target_vel)/10.0) + 1.0;
    }
    else
    {
      double m = 1.0/(v_lim-target_vel);
      double b = 1.0 - m*v_lim;
      cost_temp = (m*vel+b);
      //cost = exp(7*(-v_lim+final_vel));
    }
    if (cost_temp > 1.0)
      cost_temp = 1.0;

    if (cost_temp > cost)
    {
      cost = cost_temp;
    }
  }
 
  return cost;
}

double acceleration_peak_cost(Trajectory ego_tj)
{
  double cost = 0;
  //std::cout<<"acc"<<std::endl;
  for (int i = 0; i<ego_tj.size();i++)
  {
    double acc = abs(ego_tj.sa(i));
   // std::cout<<" "<<acc;
    double cost_temp =  1.0 - exp(-(acc)/10.0);
    if (cost_temp>1.0)
    {
      cost_temp = 1.0;
    }  
    else if (cost_temp<0.0)
    {
      cost_temp = 0.0;
    }

    if (cost_temp > cost)
    {
      cost = cost_temp;
    } 
  }
  //std::cout<<std::endl;
  return cost;
}

double acceleration_avg_cost(Trajectory ego_tj)
{
  double cost = 0;
  double avg = 0;
  //std::cout<<"acc"<<std::endl;
  for (int i = 0; i<ego_tj.size();i++)
  {
    double acc = abs(ego_tj.sa(i));
   // std::cout<<" "<<acc;
    avg += acc;   
  }
  cost = avg/ego_tj.size()/20.0;

  if (cost > 1.0)
  {
    cost = 1.0;
  }
  //std::cout<<std::endl;
  return cost;
}

double driving_forward_cost(Trajectory ego_tj)
{
  double cost = 0.0;

  if (ego_tj.lastS()<ego_tj.s(0))
  {
    //std::cout<<"driving backwards!!"<<std::endl;
    cost = 1.0;
  }

  return cost;
}

double car_ahead_cost(Trajectory ego_tj, vector<Vehicle> other_cars)
{
  double cost = 0.0;
  for (Vehicle other : other_cars)
  {  
    Trajectory other_tj = other.getTrajectory();
    int other_lane = getLane(other_tj.lastD(), 4, 2);
      for (int i = 0; i<ego_tj.size(); i++)
      {        
        if (getLane(ego_tj.d(i),4,2) == other_lane)      
        {
          double dist = abs(other_tj.s(i) - ego_tj.s(i));
          double cost_temp = exp(-0.3 * dist);
          if (cost_temp > 1.0)
          {
            cost_temp = 1.0;
          }
          else if (cost_temp < 0.0)
          {
            cost_temp = 0.0;
          }
          if (cost_temp > cost)
          {
            cost = cost_temp;
          }
        }
     }    
  }        
  return cost;
}

double change_lane_cost(Trajectory ego_tj)
{
  double cost = 0.0;
  if(getLane(ego_tj.lastD(),4,2) != getLane(ego_tj.d(0),4,2))
  {
    cost = 1.0;
  }
  return cost;
}

double prohibited_lane_cost(Trajectory ego_tj)
{
  double cost = 0.0;
  if(getLane(ego_tj.lastD(),4,1.8)<0 || getLane(ego_tj.lastD(),4,1.8)>2)
  {
    cost = 1.0;
  }
  return cost;
}

double distance_to_vehicle(Trajectory ego_tj, vector<Vehicle> other_cars)
{
  double cost = 0.0;
  double max_dist = 3.0;
  for (Vehicle other : other_cars)
  {     
    Trajectory other_tj = other.getTrajectory(); 
    for (int i = 0; i<other_tj.size();i++)
    {
      double o_s = other_tj.s(i);
      double o_d = other_tj.d(i);
      double e_s = ego_tj.s(i);
      double e_d = ego_tj.d(i);

      double cost_temp = 0.0;
      if(abs(o_d - e_d)<1 && abs(o_s - e_s)<2)
      {
        cost_temp = 1.0;
      }

      //double dist = sqrt((o_x-e_x)*(o_x-e_x) + (o_y-e_y)*(o_y-e_y));

      //double cost_temp = 1.0 - exp((dist - max_dist)/1.0);
      //if (cost_temp>1.0)
      //{
      //   cost_temp = 1.0;
      // }
      // else if (cost_temp < 0.0)
      // {
      //   cost_temp = 0.0;
      // }

      if (cost_temp>cost)
      {
        cost = cost_temp;
      }
    
    }      
  }
  return cost;
}

double inside_lane_cost(Trajectory ego_tj)
{
  double cost = 0.0;

  for (int i = 0; i<ego_tj.size();i++)
  {
    double d = ego_tj.d(i);
    double lane_d = getLaneCenterFrenet(getLane(d, 4, 2));
    double error = abs(lane_d - d);
    double cost_temp = error/4.0;

    if (cost_temp > 1.0)
    {
      cost_temp = 1.0;
    }
    if (cost_temp > cost)
    {
      cost = cost_temp;
    }
  }
  return cost;
}

double nocars_ahead_cost(Trajectory ego_tj, vector<Vehicle> other_cars)
{
  double cost = 0.0;
  double max_dist = 10.0;
  for (Vehicle other : other_cars)
  {  
    Trajectory other_tj = other.getTrajectory();
    int other_lane = getLane(other_tj.lastD(), 4, 2);
    int ego_lane = getLane(ego_tj.lastD(), 4, 2);
    if (other_lane == ego_lane &&
        other_tj.lastS() > ego_tj.lastS())
    {
     cost = 1.0;
    }      
  }
  return cost;
}

double change_safe_cost(Trajectory ego_tj, vector<Vehicle> other_cars)
{
  double cost = 0.0;
  int ego_lane_1 = getLane(ego_tj.d(0), 4, 2);
  int ego_lane_2 = getLane(ego_tj.lastD(), 4, 2);
  if (ego_lane_1 != ego_lane_2)
  {
    for (Vehicle other : other_cars)
    {  
      Trajectory other_tj = other.getTrajectory();
      int other_lane = getLane(other_tj.lastD(), 4, 2);
      if (ego_lane_2 == other_lane)
      {
        double ego_s = ego_tj.lastS();
        double other_s = other_tj.lastS();
        double ego_d = ego_tj.lastD();
        double other_d = other_tj.lastD();
        double dist = sqrt((ego_s - other_s)*(ego_s - other_s)+(ego_d - other_d)*(ego_d - other_d));
        double cost_temp = -dist/20 + 1;
        if (cost_temp<0.0)
        {
          cost_temp = 0.0;
        }
        else if (cost_temp>1.0)
        {
          cost_temp = 1.0;
        }
        if (cost_temp>cost)
        {
          cost = cost_temp;
        }
      }
    }
  }
  return cost;
}
