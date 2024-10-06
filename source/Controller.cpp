#include "Controller.hpp"

Controller::Controller(int n)
{
    Leg_num = n;
    cutoff_freq = 75;
// Gain set //
    if(Leg_num == 0)
    {
        KP = 1000;
        KI = 5000;
        KD = 0;
    }
    else if(Leg_num == 1)
    {
        KP = 1000;
        KI = 5000;
        KD = 0;
    }
    else if(Leg_num == 2)
    {
        KP = 1000;
        KI = 5000;
        KD = 0;
    }
    else
    {
        KP = 1000;
        KI = 5000;
        KD = 0;
    }
    // Filter->lowpassfilter();
}

Controller::~Controller()
{

}


Vector3d Controller::FB_controller(Vector3d error, Vector3d error_old)
{
    tau = 1 / (2 * PI * cutoff_freq);

    P_term = KP * error;
    I_term = KI * Ts / 2 * (error + error_old) + I_term_old;
    D_term = 2 * KD / (2 * tau + Ts) * (error - error_old) -
                       (Ts - 2 * tau) / (2 * tau + Ts) * D_term_old;

    PID_output = P_term + I_term + D_term;
    // cout << error << endl;    
    return PID_output;
}


void Controller::update_old_state()
{
    P_term_old = P_term;
    I_term_old = I_term;
    D_term_old = D_term;
}