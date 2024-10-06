#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "globals.hpp"
class Trajectory
{
private:
    int Leg_num;
    Vector3d ref_vel; //x,y,z

public:
    Trajectory(int n);
    ~Trajectory();
    Vector3d Leg_vel_trajectory(double t);
};


#endif

