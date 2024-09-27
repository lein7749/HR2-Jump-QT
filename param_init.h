#ifndef PARAM_INIT_H
#define PARAM_INIT_H
#include <stdio.h>
#include <iostream>
#include <QWidget>
#include <Eigen/Dense>
#define delta_t 0.01

extern float point_current;
extern QString str_upping_time;

extern QString str_downing_time;

extern QString str_mpc_time;

extern int zero_flag ;
extern int fan_flag  ;
extern int Cal_Fan_F_percent ;
extern int Cal_Fan_B_percent ;
extern int Cal_Fan_L_percent ;
extern int Cal_Fan_R_percent ;

extern int Cal_R_Hipx_degree ;
extern int Cal_R_Hipz_degree ;
extern int Cal_R_Hipy_degree ;
extern int Cal_R_Knee_degree ;
extern int Cal_R_Ankle_degree ;
extern int Cal_L_Hipx_degree ;
extern int Cal_L_Hipz_degree ;
extern int Cal_L_Hipy_degree ;
extern int Cal_L_Knee_degree ;
extern int Cal_L_Ankle_degree ;

extern int Cal_R_Hipx_velocity ;
extern int Cal_R_Hipz_velocity ;
extern int Cal_R_Hipy_velocity ;
extern int Cal_R_Knee_velocity ;
extern int Cal_R_Ankle_velocity ;
extern int Cal_L_Hipx_velocity ;
extern int Cal_L_Hipz_velocity ;
extern int Cal_L_Hipy_velocity ;
extern int Cal_L_Knee_velocity ;
extern int Cal_L_Ankle_velocity ;

//Motor kp
extern int Cal_R_Hipx_kp;
extern int Cal_R_Hipz_kp;
extern int Cal_R_Hipy_kp;
extern int Cal_R_Knee_kp;
extern int Cal_R_Ankle_kp;
extern int Cal_L_Hipx_kp;
extern int Cal_L_Hipz_kp;
extern int Cal_L_Hipy_kp;
extern int Cal_L_Knee_kp;
extern int Cal_L_Ankle_kp;


//Motor kd
extern int Cal_R_Hipx_kd;
extern int Cal_R_Hipz_kd;
extern int Cal_R_Hipy_kd;
extern int Cal_R_Knee_kd;
extern int Cal_R_Ankle_kd;
extern int Cal_L_Hipx_kd;
extern int Cal_L_Hipz_kd;
extern int Cal_L_Hipy_kd;
extern int Cal_L_Knee_kd;
extern int Cal_L_Ankle_kd;

//Motor torque
extern int Cal_R_Hipx_torque;
extern int Cal_R_Hipz_torque;
extern int Cal_R_Hipy_torque;
extern int Cal_R_Knee_torque;
extern int Cal_R_Ankle_torque;
extern int Cal_L_Hipx_torque;
extern int Cal_L_Hipz_torque;
extern int Cal_L_Hipy_torque;
extern int Cal_L_Knee_torque;
extern int Cal_L_Ankle_torque;

//Motor extra_torque
extern int Cal_R_Hipx_extra_torque ;
extern int Cal_R_Hipz_extra_torque ;
extern int Cal_R_Hipy_extra_torque ;
extern int Cal_R_Knee_extra_torque ;
extern int Cal_R_Ankle_extra_torque ;
extern int Cal_L_Hipx_extra_torque ;
extern int Cal_L_Hipz_extra_torque ;
extern int Cal_L_Hipy_extra_torque ;
extern int Cal_L_Knee_extra_torque ;
extern int Cal_L_Ankle_extra_torque ;

//servo_theta
extern int	servo_degree;
extern int Cal_servo_degree;
extern int send_mode;

extern float imu_euler_roll ;
extern float imu_euler_pitch ;
extern float imu_euler_yaw ;

extern float imu_euler_roll_v ;
extern float imu_euler_pitch_v ;
extern float imu_euler_pitch_v_nn ;
extern float imu_euler_yaw_v ;
extern float imu_euler_pitch_v_new;

// extern float position_x;
// extern float position_y;
// extern float position_z;
// extern float velocity_x;
// extern float velocity_y;
// extern float velocity_z;

extern float R_Hipx_degree ;
extern float R_Hipz_degree ;
extern float R_Hipy_degree ;
extern float R_Knee_degree ;
extern float R_Ankle_degree ;
extern float L_Hipx_degree ;
extern float L_Hipz_degree ;
extern float L_Hipy_degree ;
extern float L_Knee_degree ;
extern float L_Ankle_degree ;

extern float R_Hipx_velocity ;
extern float R_Hipz_velocity ;
extern float R_Hipy_velocity ;
extern float R_Knee_velocity ;
extern float R_Ankle_velocity ;
extern float L_Hipx_velocity ;
extern float L_Hipz_velocity ;
extern float L_Hipy_velocity ;
extern float L_Knee_velocity ;
extern float L_Ankle_velocity ;

extern float R_Hipx_current ;
extern float R_Hipz_current ;
extern float R_Hipy_current ;
extern float R_Knee_current ;
extern float R_Ankle_current ;
extern float L_Hipx_current ;
extern float L_Hipz_current ;
extern float L_Hipy_current ;
extern float L_Knee_current ;
extern float L_Ankle_current ;

extern float R_Hipx_connect_flag ;
extern float R_Hipz_connect_flag ;
extern float R_Hipy_connect_flag ;
extern float R_Knee_connect_flag ;
extern float R_Ankle_connect_flag ;
extern float L_Hipx_connect_flag ;
extern float L_Hipz_connect_flag ;
extern float L_Hipy_connect_flag ;
extern float L_Knee_connect_flag ;
extern float L_Ankle_connect_flag ;

extern float L_Hipx_target    ;
extern float L_Hipy_target    ;
extern float L_Hipz_target    ;
extern float L_Knee_target    ;
extern float L_Ankle_target    ;

extern float R_Hipx_target    ;
extern float R_Hipy_target    ;
extern float R_Hipz_target    ;
extern float R_Knee_target    ;
extern float R_Ankle_target    ;

extern float servo_target    ;

extern float R_Hipz_start    ;
extern float R_Hipy_start    ;
extern float R_Hipx_start    ;
extern float R_Knee_start    ;
extern float R_Ankle_start    ;

extern float L_Hipz_start    ;
extern float L_Hipy_start    ;
extern float L_Hipx_start    ;
extern float L_Knee_start    ;
extern float L_Ankle_start    ;

extern float servo_start ;

extern int motor_move_count_times  ;
extern float time_all;

extern Eigen::VectorXd body_p;
extern Eigen::MatrixXd body_R;

extern Eigen::VectorXd left_p;
extern float Lfoot_yaw;
extern float Lfoot_pitch;

extern Eigen::VectorXd right_p;
extern float Rfoot_yaw;
extern float Rfoot_pitch;

extern float motor_move_time ;

extern float  fan_text_count;
extern bool fan_text_flag;

extern bool fly_flag ;

extern bool rectify_imu_flag;

extern float rectify_imu_count;

extern QVector<float> imu_euler_roll_array;
extern QVector<float> imu_euler_pitch_array;
extern QVector<float> imu_euler_yaw_array;

extern float imu_euler_roll_ref ;
extern float imu_euler_pitch_ref ;
extern float imu_euler_yaw_ref ;

extern float kp_servo;
extern float kd_servo;
extern float ki_servo;
extern float base_fan;
extern float kp_fan;
extern float kd_fan;
extern float ki_fan;
extern float jump_time_count;
extern float jump_flag;
extern float Contact_test_flag;
extern float Collision_flag_R;
extern float Collision_flag_L;
extern float robot_state_R;
extern float robot_state_L;
extern float robot_state;
extern float excel_time_count;

extern float jump_start_flag;

extern float Kp_Roll_Support;
extern float kd_Roll_Support;
extern float h_max;
extern float kp_hip_Support;
extern float kd_hip_Support;
extern float hip_Supporting_ecpect;
extern float K_Roll_air;
extern float kd_Roll_air;

extern double F_f1, F_b1, F_x, F_f2, F_b2, F_f, F_b, F_thta;
extern int Movement_mode;

// MPC 参数设置
extern float Qi_x;
extern float Qi_dx;
extern float Qi_thta;
extern float Qi_dthta;
extern float Ri_f;

extern int take_off_flag;

extern QVector<double> buf_upping_time;
extern QVector<double> buf_imu_roll;
extern QVector<double> buf_imu_pitch;
extern QVector<double> buf_imu_yaw;

extern QVector<double> buf_imu_rollV;
extern QVector<double> buf_imu_pitchV;
extern QVector<double> buf_imu_euler_pitch_v_new;
extern QVector<double> buf_imu_yawV;

extern QVector<double> buf_imu_rollV_raw;
extern QVector<double> buf_imu_pitchV_raw;
extern QVector<double> buf_imu_yawV_raw;

extern QVector<double> buf_imu_phi_desire;
extern QVector<double> buf_imu_xita_desire;
extern QVector<double> buf_imu_psid_desire;

extern QVector<double> buf_imu_phi_vel_desire;
extern QVector<double> buf_imu_xita_vel_desire;
extern QVector<double> buf_imu_psid_vel_desire;

extern QVector<double> buf_L_Hipx_degree;
extern QVector<double> buf_L_Hipz_degree;
extern QVector<double> buf_L_Hipy_degree;
extern QVector<double> buf_L_Knee_degree;
extern QVector<double> buf_L_Ankle_degree;
extern QVector<double> buf_R_Hipx_degree;
extern QVector<double> buf_R_Hipz_degree;
extern QVector<double> buf_R_Hipy_degree;
extern QVector<double> buf_R_Knee_degree;
extern QVector<double> buf_R_Ankle_degree;

extern QVector<double> buf_L_Hipx_velocity;
extern QVector<double> buf_L_Hipz_velocity;
extern QVector<double> buf_L_Hipy_velocity;
extern QVector<double> buf_L_Knee_velocity;
extern QVector<double> buf_L_Ankle_velocity;
extern QVector<double> buf_R_Hipx_velocity;
extern QVector<double> buf_R_Hipz_velocity;
extern QVector<double> buf_R_Hipy_velocity;
extern QVector<double> buf_R_Knee_velocity;
extern QVector<double> buf_R_Ankle_velocity;

extern QVector<double> buf_L_Hipx_current;
extern QVector<double> buf_L_Hipz_current;
extern QVector<double> buf_L_Hipy_current;
extern QVector<double> buf_L_Knee_current;
extern QVector<double> buf_L_Ankle_current;
extern QVector<double> buf_R_Hipx_current;
extern QVector<double> buf_R_Hipz_current;
extern QVector<double> buf_R_Hipy_current;
extern QVector<double> buf_R_Knee_current;
extern QVector<double> buf_R_Ankle_current;

extern QVector<double> buf_motive_time;
extern QVector<double> filter_used_time;
extern QVector<double> buf_x;
extern QVector<double> buf_y;
extern QVector<double> buf_z;
extern QVector<double> buf_dx;
extern QVector<double> buf_dy;
extern QVector<double> buf_dz;
extern QVector<double> buf_ddx;
extern QVector<double> buf_ddy;
extern QVector<double> buf_ddz;
extern QVector<double> buf_target_x_pos;
extern QVector<double> buf_target_y_pos;
extern QVector<double> buf_target_z_pos;
extern QVector<double> buf_target_x_vel;
extern QVector<double> buf_target_y_vel;
extern QVector<double> buf_target_z_vel;

extern QVector<double> buf_downing_time;
extern QVector<double> buf_fan_flag;
extern QVector<double> buf_Fan_F_percent;
extern QVector<double> buf_Fan_B_percent;
extern QVector<double> buf_Fan_L_percent;
extern QVector<double> buf_Fan_R_percent;
extern QVector<double> buf_Cal_R_Hipx_degree;
extern QVector<double> buf_Cal_R_Hipz_degree;
extern QVector<double> buf_Cal_R_Hipy_degree;
extern QVector<double> buf_Cal_R_Knee_degree;
extern QVector<double> buf_Cal_R_Ankle_degree;
extern QVector<double> buf_Cal_L_Hipx_degree;
extern QVector<double> buf_Cal_L_Hipz_degree;
extern QVector<double> buf_Cal_L_Hipy_degree;
extern QVector<double> buf_Cal_L_Knee_degree;
extern QVector<double> buf_Cal_L_Ankle_degree;

extern QVector<double> buf_robot_state_R;
extern QVector<double> buf_robot_state_L;
extern QVector<double> buf_robot_state;

extern QVector<double> buf_servo_degree;

extern QVector<QVector<double>> buf_downing;

extern int str_filter_upping_time;

extern QVector<QVector<double>> buf_motive;
extern QVector<QVector<double>> buf_upping;

extern QString excel_time;

extern float angle_jump;
extern float time_jump;
extern float tor_jump ;
extern float ankle_vel;
extern float jump_num;
extern QVector<double> ankle_angle_tra;

///////////////////////////////////////////////
/////////////on_timer_timeout_init/////////////
///////////////////////////////////////////////

extern int rectify_motive_count  ;
extern bool rectify_motive_flag ;
extern bool i_open_flag_button  ;

extern double position_x_desire;
extern double position_y_desire;
extern double position_z_desire;

extern double velocity_x_desire;
extern double velocity_y_desire;
extern double velocity_z_desire;

extern double position_x  ;
extern double position_x_ ;
extern double position_x__ ;
extern double position_y  ;
extern double position_y_ ;
extern double position_y__ ;
extern double position_z  ;
extern double position_z_ ;
extern double position_z__ ;

extern double velocity_x ;
extern double velocity_x_ ;
extern double velocity_x__ ;
extern double velocity_y ;
extern double velocity_y_ ;
extern double velocity_y__ ;
extern double velocity_z ;
extern double velocity_z_ ;
extern double velocity_z__ ;

extern double acceleration_x ;
extern double acceleration_x_ ;
extern double acceleration_x__ ;
extern double acceleration_y ;
extern double acceleration_y_ ;
extern double acceleration_y__ ;
extern double acceleration_z ;
extern double acceleration_z_ ;
extern double acceleration_z__ ;

extern double position_ref_x ;
extern double position_ref_y ;
extern double position_ref_z ;

extern double qx_ref ;
extern double qy_ref ;
extern double qz_ref ;
extern double qw_ref ;
extern double qx  ;
extern double qy  ;
extern double qz  ;
extern double qw  ;

extern float r_x_err   ;//P
extern float r_y_err   ;
extern float r_z_err   ;
extern float r_x_err_temp   ;//I
extern float r_y_err_temp   ;
extern float r_z_err_temp   ;
extern float r_x_errV   ;//D
extern float r_y_errV   ;
extern float r_z_errV   ;
extern float r_x_err_   ;//last data
extern float r_y_err_   ;
extern float r_z_err_   ;
extern float world_yaw  ;

extern float vel_x_err   ;//P
extern float vel_y_err   ;
extern float vel_z_err   ;
extern float vel_x_err_temp   ;//I
extern float vel_y_err_temp   ;
extern float vel_z_err_temp   ;
extern float vel_x_errV   ;//D
extern float vel_y_errV   ;
extern float vel_z_errV   ;
extern float vel_x_err_   ;//last data
extern float vel_y_err_   ;
extern float vel_z_err_   ;

extern QVector<float> motive_position_x_array;
extern QVector<float> motive_position_y_array;
extern QVector<float> motive_position_z_array;
extern QVector<float> motive_roll_array;
extern QVector<float> motive_pitch_array;
extern QVector<float> motive_yaw_array;


#endif // PARAM_INIT_H
