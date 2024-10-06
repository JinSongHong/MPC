#include "globals.hpp"

const double Ts = 0.001; // sampling period
const double g = 9.81;    // gravitational accel.
// const double PI = 3.141592;


/* Trunk Dimension */
const double width_trunk = 0.25;
const double length_trunk = 0.66;
const double length_front = 0.34;
const double length_rear = 0.32;
const double height_trunk = 0.083;

double jacb_trunk_pos[NUM_LEG * NUM_LEG];
double jacb_trunk_pos_inv[NUM_LEG * NUM_LEG];
double jacb_trunk_vel[NUM_LEG * NDOF_TRUNK];

/* Trunk States */
// double pos_trunk_act[6];
// double vel_trunk_act[6];
double rot_mat_trunk[9];
double rot_mat_trunk_act[9];

double pos_trunk_des[NDOF_TRUNK];
double pos_trunk_des_old[NDOF_TRUNK];
double pos_trunk_est[NDOF_TRUNK];
double pos_trunk_est_old[NDOF_TRUNK];

double vel_trunk_des[NDOF_TRUNK];
double vel_trunk_des_old[NDOF_TRUNK];
double vel_trunk_est[NDOF_TRUNK];
double vel_trunk_est_old[NDOF_TRUNK];

double error_pos_trunk[NDOF_TRUNK];
double error_pos_trunk_old[NDOF_TRUNK];
double error_dot_pos_trunk[NDOF_TRUNK];
double error_dot_pos_trunk_old[NDOF_TRUNK];

double error_vel_trunk[NDOF_TRUNK];
double error_vel_trunk_old[NDOF_TRUNK];

double ctrl_input_RW_from_trunk[NUM_LEG];