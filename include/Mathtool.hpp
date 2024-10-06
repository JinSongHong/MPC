#ifndef MATHTOOL_H_
#define MATHTOOL_H_

#include "globals.hpp"


// using namespace Eigen;

class Mathtool
{
private:
    double pi = M_PI;
    double* Quat_out;
    double* Ori_out;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    
public:
    Mathtool(/* args */);
    ~Mathtool();
    MatrixXd pinv(const MatrixXd &A);

    double tustin_derivative(double input, double input_old, double output_old, double cutoff_freq);
    double lowpassfilter(double input, double input_old, double output_old, double cutoff_freq);
    double* Ori2Quat(double* Ori);
    double* Quat2Ori(double* Quat);

};




#endif