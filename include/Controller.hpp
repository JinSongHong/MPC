#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "globals.hpp"
#include "Mathtool.hpp"
class Mathtool;

class Controller
{
private:
    // filtertool filtere;
    Mathtool* Math;

    int Leg_num;
    double cutoff_freq = 10;
    double tau = 0.01;

    Vector3d P_term;
    Vector3d P_term_old;
    Vector3d I_term;
    Vector3d I_term_old;
    Vector3d D_term;
    Vector3d D_term_old;
    Vector3d PID_output = Vector3d::Zero();
    


    // Gain //
    double KP;
    double KI;
    double KD;

public:



    Controller(int n);  
    ~Controller();
    Vector3d FB_controller(Vector3d error, Vector3d error_old);
    void update_old_state();

    
};



#endif