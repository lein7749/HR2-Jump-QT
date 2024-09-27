#ifndef ALL_TRJ_H
#define ALL_TRJ_H
#include <Eigen/Dense>
float linear_trajectory(float point_ini,float point_tar,float current_time,float time_all);

float **Trj_ploy5th_plus(float th00, float th11,
                         float thv00, float thv11,
                         float tha00, float tha11,
                         float T, float T_step);

float landing_trj(float resist_Land_T_all,float resist_Land_T_step,
                  float position_z_pos_first,float position_z_pos_last,
                  float position_z_vel_first,float position_z_vel_last,
                  float position_z_acc_first,float position_z_acc_last);

float *Landing_resist(float Fan_F_percent,float Fan_B_percent,float HR2_mass,
                      float Landing_z_pos_p, float Landing_z_vel_p,
                      float z_position_err,float z_position_err_temp,float z_position_err_v,
                      float z_velocity_err,float z_velocity_err_temp,float z_velocity_err_v);

float *x_trj(float desire_x_steptime,
             float desire_x_pos_1,float desire_x_pos_2,float desire_x_pos_3,
             float desire_x_time_1,float desire_x_time_2,float desire_x_time_3);

float *y_trj(float desire_y_steptime,
             float desire_y_pos_1,float desire_y_pos_2,float desire_y_pos_3,
             float desire_y_time_1,float desire_y_time_2,float desire_y_time_3);

float *z_trj(float desire_z_steptime,
             float desire_z_pos_1,float desire_z_pos_2,float desire_z_pos_3,
             float desire_z_time_1,float desire_z_time_2,float desire_z_time_3);

float *square_trj(float desire_x_steptime,
                  float desire_x_pos_1,float desire_y_pos_1,float desire_first_rotation_angle,
                  float desire_x_time_1,float desire_y_time_1,float desire_rotation_time_1);

float *linear_trajectory_plus(double point_ini,float point_tar,float time_all,float step_time);

Eigen::Matrix<double,5,6> trj_w_to_b();

double *worldpos_to_bodypos(double x,double y,double z);
double *worldvel_to_bodyvel(double x,double y,double z);


#endif // ALL_TRJ_H
