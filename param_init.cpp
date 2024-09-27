#include "param_init.h"
#include <stdlib.h>

float point_current=0;
QString str_upping_time;
QString str_downing_time;
QString str_mpc_time = "None";

int zero_flag=0;
int fan_flag = 0 ;
int Cal_Fan_F_percent=0;
int Cal_Fan_B_percent=0;
int Cal_Fan_L_percent=0;
int Cal_Fan_R_percent=0;

int Cal_R_Hipx_degree=0;
int Cal_R_Hipz_degree=0;
int Cal_R_Hipy_degree=0;
int Cal_R_Knee_degree=0;
int Cal_R_Ankle_degree=0;
int Cal_L_Hipx_degree=0;
int Cal_L_Hipz_degree=0;
int Cal_L_Hipy_degree=0;
int Cal_L_Knee_degree=0;
int Cal_L_Ankle_degree=0;
int Cal_R_Hipx_velocity=0;
int Cal_R_Hipz_velocity=0;
int Cal_R_Hipy_velocity=0;
int Cal_R_Knee_velocity=0;
int Cal_R_Ankle_velocity=0;
int Cal_L_Hipx_velocity=0;
int Cal_L_Hipz_velocity=0;
int Cal_L_Hipy_velocity=0;
int Cal_L_Knee_velocity=0;
int Cal_L_Ankle_velocity=0;

//Motor kp
int Cal_R_Hipy_kp=110*100;
int Cal_R_Hipz_kp=70*100;
int Cal_R_Hipx_kp=120*100;
int Cal_R_Knee_kp=120*100;
int Cal_R_Ankle_kp=120*100;

int Cal_L_Hipy_kp=110*100;
int Cal_L_Hipz_kp=100*100;
int Cal_L_Hipx_kp=120*100;
int Cal_L_Knee_kp=120*100;
int Cal_L_Ankle_kp=120*100;

// int Cal_R_Hipy_kp=30*100;
// int Cal_R_Hipz_kp=30*100;
// int Cal_R_Hipx_kp=30*100;
// int Cal_R_Knee_kp=30*100;
// int Cal_R_Ankle_kp=30*100;

// int Cal_L_Hipy_kp=30*100;
// int Cal_L_Hipz_kp=30*100;
// int Cal_L_Hipx_kp=30*100;
// int Cal_L_Knee_kp=30*100;
// int Cal_L_Ankle_kp=30*100;


//Motor kd
int Cal_R_Hipx_kd=1*100;
int Cal_R_Hipz_kd=1*100;
int Cal_R_Hipy_kd=1*100;
int Cal_R_Knee_kd=1*100;
int Cal_R_Ankle_kd=1*100;

int Cal_L_Hipx_kd=1*100;
int Cal_L_Hipz_kd=1*100;
int Cal_L_Hipy_kd=1*100;
int Cal_L_Knee_kd=1*100;
int Cal_L_Ankle_kd=1*100;

//Motor torque
int Cal_R_Hipx_torque=0;
int Cal_R_Hipz_torque=0;
int Cal_R_Hipy_torque=0;
int Cal_R_Knee_torque=0;
int Cal_R_Ankle_torque=0;
int Cal_L_Hipx_torque=0;
int Cal_L_Hipz_torque=0;
int Cal_L_Hipy_torque=0;
int Cal_L_Knee_torque=0;
int Cal_L_Ankle_torque=0;

//Motor extra_torque
int Cal_R_Hipx_extra_torque=0;
int Cal_R_Hipz_extra_torque=0;
int Cal_R_Hipy_extra_torque=0;
int Cal_R_Knee_extra_torque=0;
int Cal_R_Ankle_extra_torque=0;
int Cal_L_Hipx_extra_torque=0;
int Cal_L_Hipz_extra_torque=0;
int Cal_L_Hipy_extra_torque=0;
int Cal_L_Knee_extra_torque=0;
int Cal_L_Ankle_extra_torque=0;

//servo_theta
int	servo_degree=90*100;
int	Cal_servo_degree=90*100;
int send_mode = 0;

float imu_euler_roll=0;
float imu_euler_pitch=0;
float imu_euler_yaw=0;

float imu_euler_roll_v=0;
float imu_euler_pitch_v_nn=0;
float imu_euler_pitch_v=0;
float imu_euler_yaw_v=0;
float imu_euler_pitch_v_new = 0;

// float position_x=0.0;
// float position_y=0.0;
// float position_z=0.0;
// float velocity_x=0.0;
// float velocity_y=0.0;
// float velocity_z=0.0;

float R_Hipx_degree=0;
float R_Hipz_degree=0;
float R_Hipy_degree=0;
float R_Knee_degree=0;
float R_Ankle_degree=0;
float L_Hipx_degree=0;
float L_Hipz_degree=0;
float L_Hipy_degree=0;
float L_Knee_degree=0;
float L_Ankle_degree=0;

float R_Hipx_velocity=0;
float R_Hipz_velocity=0;
float R_Hipy_velocity=0;
float R_Knee_velocity=0;
float R_Ankle_velocity=0;
float L_Hipx_velocity=0;
float L_Hipz_velocity=0;
float L_Hipy_velocity=0;
float L_Knee_velocity=0;
float L_Ankle_velocity=0;

float R_Hipx_current=0;
float R_Hipz_current=0;
float R_Hipy_current=0;
float R_Knee_current=0;
float R_Ankle_current=0;
float L_Hipx_current=0;
float L_Hipz_current=0;
float L_Hipy_current=0;
float L_Knee_current=0;
float L_Ankle_current=0;

float R_Hipx_connect_flag=0;
float R_Hipz_connect_flag=0;
float R_Hipy_connect_flag=0;
float R_Knee_connect_flag=0;
float R_Ankle_connect_flag=0;
float L_Hipx_connect_flag=0;
float L_Hipz_connect_flag=0;
float L_Hipy_connect_flag=0;
float L_Knee_connect_flag=0;
float L_Ankle_connect_flag=0;

float L_Hipx_target = 0.0;
float L_Hipz_target = 0.0;
float L_Hipy_target = 0.0;
float L_Knee_target = 0.0;
float L_Ankle_target = 0.0;

float R_Hipx_target = 0.0;
float R_Hipz_target = 0.0;
float R_Hipy_target = 0.0;
float R_Knee_target = 0.0;
float R_Ankle_target = 0.0;

float servo_target = 90;

float R_Hipy_start = 0.0;
float R_Hipx_start = 0.0;
float R_Hipz_start = 0.0;
float R_Knee_start = 0.0;
float R_Ankle_start = 0.0;

float L_Hipy_start = 0.0;
float L_Hipx_start = 0.0;
float L_Hipz_start = 0.0;
float L_Knee_start = 0.0;
float L_Ankle_start = 0.0;

float servo_start = 0.0;

int motor_move_count_times = 0;
float time_all = 0;

Eigen::VectorXd body_p = Eigen::MatrixXd::Zero(3, 1);
Eigen::MatrixXd body_R = Eigen::MatrixXd::Zero(3, 3);

Eigen::VectorXd left_p = Eigen::MatrixXd::Zero(3, 1);
float Lfoot_yaw = 0;
float Lfoot_pitch = 0;

Eigen::VectorXd right_p = Eigen::MatrixXd::Zero(3, 1);
float Rfoot_yaw = 0;
float Rfoot_pitch = 0;

float motor_move_time = 0.0;

float  fan_text_count = 0;
bool fan_text_flag = 0;

bool fly_flag = 0;

bool rectify_imu_flag = 0;

float rectify_imu_count = 0;

QVector<float> imu_euler_roll_array(100);
QVector<float> imu_euler_pitch_array(100);
QVector<float> imu_euler_yaw_array(100);


float imu_euler_roll_ref=0;
float imu_euler_pitch_ref=0;
float imu_euler_yaw_ref=0;

float kp_servo = 0;
float kd_servo = 0;
float ki_servo = 0;
float base_fan = 0;
float kp_fan = 0;
float kd_fan = 0;
float ki_fan = 0;
float jump_time_count = 0;
float jump_flag = 0;
float Contact_test_flag = 0;
float Collision_flag_R = 0;
float Collision_flag_L = 0;
float robot_state_R = 0;
float robot_state_L = 0;
float robot_state = 0;

float jump_start_flag = 0;

float excel_time_count=0.0;

float Kp_Roll_Support = 0.0;
float kd_Roll_Support = 0.0;
float h_max = 0.0;
float kp_hip_Support = 0;
float kd_hip_Support = 0;
float hip_Supporting_ecpect = 0;
float K_Roll_air = 0;
float kd_Roll_air = 0;

double F_f1, F_b1, F_x, F_f2, F_b2, F_f, F_b, F_thta;
int Movement_mode = 0;

// MPC 初始参数设置
float Qi_x = 0;
float Qi_dx = 10;
float Qi_thta = 20000;
float Qi_dthta = 0;
float Ri_f = 0.001;

int take_off_flag = 1;

QVector<double> buf_upping_time;
QVector<double> buf_imu_roll;
QVector<double> buf_imu_pitch;
QVector<double> buf_imu_yaw;

QVector<double> buf_imu_rollV;
QVector<double> buf_imu_pitchV;
QVector<double> buf_imu_euler_pitch_v_new;
QVector<double> buf_imu_yawV;

QVector<double> buf_imu_rollV_raw;
QVector<double> buf_imu_pitchV_raw;
QVector<double> buf_imu_yawV_raw;

QVector<double> buf_imu_phi_desire;
QVector<double> buf_imu_xita_desire;
QVector<double> buf_imu_psid_desire;

QVector<double> buf_imu_phi_vel_desire;
QVector<double> buf_imu_xita_vel_desire;
QVector<double> buf_imu_psid_vel_desire;

QVector<double> buf_L_Hipx_degree;
QVector<double> buf_L_Hipz_degree;
QVector<double> buf_L_Hipy_degree;
QVector<double> buf_L_Knee_degree;
QVector<double> buf_L_Ankle_degree;
QVector<double> buf_R_Hipx_degree;
QVector<double> buf_R_Hipz_degree;
QVector<double> buf_R_Hipy_degree;
QVector<double> buf_R_Knee_degree;
QVector<double> buf_R_Ankle_degree;

QVector<double> buf_L_Hipx_velocity;
QVector<double> buf_L_Hipz_velocity;
QVector<double> buf_L_Hipy_velocity;
QVector<double> buf_L_Knee_velocity;
QVector<double> buf_L_Ankle_velocity;
QVector<double> buf_R_Hipx_velocity;
QVector<double> buf_R_Hipz_velocity;
QVector<double> buf_R_Hipy_velocity;
QVector<double> buf_R_Knee_velocity;
QVector<double> buf_R_Ankle_velocity;

QVector<double> buf_L_Hipx_current;
QVector<double> buf_L_Hipz_current;
QVector<double> buf_L_Hipy_current;
QVector<double> buf_L_Knee_current;
QVector<double> buf_L_Ankle_current;
QVector<double> buf_R_Hipx_current;
QVector<double> buf_R_Hipz_current;
QVector<double> buf_R_Hipy_current;
QVector<double> buf_R_Knee_current;
QVector<double> buf_R_Ankle_current;

QVector<double> buf_motive_time;
QVector<double> filter_used_time;
QVector<double> buf_x;
QVector<double> buf_y;
QVector<double> buf_z;
QVector<double> buf_dx;
QVector<double> buf_dy;
QVector<double> buf_dz;
QVector<double> buf_ddx;
QVector<double> buf_ddy;
QVector<double> buf_ddz;
QVector<double> buf_target_x_pos;
QVector<double> buf_target_y_pos;
QVector<double> buf_target_z_pos;
QVector<double> buf_target_x_vel;
QVector<double> buf_target_y_vel;
QVector<double> buf_target_z_vel;


QVector<double> buf_downing_time;
QVector<double> buf_fan_flag;
QVector<double> buf_Fan_F_percent;
QVector<double> buf_Fan_B_percent;
QVector<double> buf_Fan_L_percent;
QVector<double> buf_Fan_R_percent;
QVector<double> buf_Cal_R_Hipx_degree;
QVector<double> buf_Cal_R_Hipz_degree;
QVector<double> buf_Cal_R_Hipy_degree;
QVector<double> buf_Cal_R_Knee_degree;
QVector<double> buf_Cal_R_Ankle_degree;
QVector<double> buf_Cal_L_Hipx_degree;
QVector<double> buf_Cal_L_Hipz_degree;
QVector<double> buf_Cal_L_Hipy_degree;
QVector<double> buf_Cal_L_Knee_degree;
QVector<double> buf_Cal_L_Ankle_degree;

QVector<double> buf_robot_state_R;
QVector<double> buf_robot_state_L;
QVector<double> buf_robot_state;

QVector<double> buf_servo_degree;

QVector<QVector<double>> buf_downing;

int str_filter_upping_time;

QVector<QVector<double>> buf_motive;
QVector<QVector<double>> buf_upping;

QString excel_time;

float angle_jump = 35;
float time_jump  = 0.1;
float tor_jump   = 18;
float ankle_vel = 0;
float jump_num = 0;
QVector<double> ankle_angle_tra(500);


///////////////////////////////////////////////
/////////////on_timer_timeout_init/////////////
///////////////////////////////////////////////

int rectify_motive_count = 0;
bool rectify_motive_flag = 1;
bool i_open_flag_button = 0;

double position_x_desire =0;
double position_y_desire =0;
double position_z_desire =0;

double velocity_x_desire =0;
double velocity_y_desire =0;
double velocity_z_desire =0;

double position_x =0;
double position_x_=0;
double position_x__=0;
double position_y =0;
double position_y_=0;
double position_y__=0;
double position_z =0;
double position_z_=0;
double position_z__=0;

double velocity_x=0;
double velocity_x_=0;
double velocity_x__=0;
double velocity_y=0;
double velocity_y_=0;
double velocity_y__=0;
double velocity_z=0;
double velocity_z_=0;
double velocity_z__=0;

double acceleration_x=0;
double acceleration_x_=0;
double acceleration_x__=0;
double acceleration_y=0;
double acceleration_y_=0;
double acceleration_y__=0;
double acceleration_z=0;
double acceleration_z_=0;
double acceleration_z__=0;

double position_ref_x=0;
double position_ref_y=0;
double position_ref_z=0;

double qx_ref=0;
double qy_ref=0;
double qz_ref=0;
double qw_ref=0;
double qx = 0;
double qy = 0;
double qz = 0;
double qw = 0;

float r_x_err=0.0;//P
float r_y_err=0.0;
float r_z_err=0.0;
float r_x_err_temp=0.0;//I
float r_y_err_temp=0.0;
float r_z_err_temp=0.0;
float r_x_errV=0.0;//D
float r_y_errV=0.0;
float r_z_errV=0.0;
float r_x_err_=0.0;//last data
float r_y_err_=0.0;
float r_z_err_=0.0;
float world_yaw = 0;

float vel_x_err=0.0;//P
float vel_y_err=0.0;
float vel_z_err=0.0;
float vel_x_err_temp=0.0;//I
float vel_y_err_temp=0.0;
float vel_z_err_temp=0.0;
float vel_x_errV=0.0;//D
float vel_y_errV=0.0;
float vel_z_errV=0.0;
float vel_x_err_=0.0;//last data
float vel_y_err_=0.0;
float vel_z_err_=0.0;

QVector<float> motive_position_x_array(100);
QVector<float> motive_position_y_array(100);
QVector<float> motive_position_z_array(100);
QVector<float> motive_roll_array(100);
QVector<float> motive_pitch_array(100);
QVector<float> motive_yaw_array(100);
