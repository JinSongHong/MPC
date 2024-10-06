#include "Mathtool.hpp"

Mathtool::Mathtool(/* args */)
{
}

Mathtool::~Mathtool()
{
}

MatrixXd Mathtool::pinv(const MatrixXd &A)
{
    return A.completeOrthogonalDecomposition().pseudoInverse();
}

double Mathtool::tustin_derivative(double input, double input_old, double output_old, double cutoff_freq)
{
    double time_const = 1 / (2 * pi * cutoff_freq);
    double output = 0;

    output = (2 * (input - input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);

    return output;
}

double Mathtool::lowpassfilter(double input, double input_old, double output_old, double cutoff_freq) 
{
    double time_const = 1 / (2 * pi * cutoff_freq);
    double output = 0;

    output = (Ts * (input + input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);

    return output;
}

double* Mathtool::Ori2Quat(double* Ori)
{
    roll = Ori[0];
    pitch = Ori[1];
    yaw = Ori[2];
    
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
 
    Quat_out[0] = cy * cr * cp + sy * sr * sp;  //qw
    Quat_out[1] = cy * sr * cp - sy * cr * sp;  //qx
    Quat_out[2] = cy * cr * sp + sy * sr * cp;  //qy
    Quat_out[3] = sy * cr * cp - cy * sr * sp;  //qz

    return Quat_out;
}
 
double* Mathtool::Quat2Ori(double* Quat)
{
    
}