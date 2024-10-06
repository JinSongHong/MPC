// #include "filtertool.hpp"


// filtertool::filtertool(){}

// filtertool::~filtertool(){}

// double filtertool::tustin_derivative(double input, double input_old, double output_old, double cutoff_freq)
// {
//     double time_const = 1 / (2 * PI * cutoff_freq);
//     double output = 0;

//     output = (2 * (input - input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);

//     return output;
// }

// double filtertool::lowpassfilter(double input, double input_old, double output_old, double cutoff_freq) 
// {
//     double time_const = 1 / (2 * PI * cutoff_freq);
//     double output = 0;

//     output = (Ts * (input + input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);

//     return output;
// }