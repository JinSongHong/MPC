#include "Trajectory.hpp"

Trajectory::Trajectory(int n)
{
    Leg_num = n;   
}

Trajectory::~Trajectory()
{
}

Vector3d Trajectory::Leg_vel_trajectory(double t)
{
    if(Leg_num == 0)
    {
        // ref_vel[0] = 0.05*sin(4*t);  // theta
        // ref_vel[1] = 0.05*sin(4*t);  // pi
        // ref_vel[2] = 0.05*sin(4*t);     // r
    }
    else if(Leg_num == 1)
    {
        ref_vel[0] = 0.1; //0.05*sin(4*t);  // theta
        // ref_vel[1] = 0.05*sin(4*t);  // pi
        // ref_vel[2] = 0.1;     // r
    }
    else if(Leg_num == 2)
    {
        ref_vel[0] = 0;
        ref_vel[1] = 0;
        ref_vel[2] = 0;
    }
    else
    {
        ref_vel[0] = 0;
        ref_vel[1] = 0;
        ref_vel[2] = 0;
    }

    
    return ref_vel;
}