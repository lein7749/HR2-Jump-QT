#include "data_init.h"

HC2JCData Data_send;

void dataforsend()
{
    Data_send.data_len = 0x7C;      //124字节

    Data_send.zero_flag = 0x00;
    Data_send.fan_flag = 0x00;

    Data_send.Cal_Fan_F_percent = 0x00;     //Fan percent
    Data_send.Cal_Fan_B_percent = 0x00;
    Data_send.Cal_Fan_L_percent = 0x00;
    Data_send.Cal_Fan_R_percent = 0x00;

    Data_send.Cal_R_Hipx_degree_flag = 0x00;
    Data_send.Cal_R_Hipx_degree_high = 0x00;
    Data_send.Cal_R_Hipx_degree_low = 0x00;

    Data_send.Cal_R_Hipz_degree_flag = 0x00;
    Data_send.Cal_R_Hipz_degree_high = 0x00;
    Data_send.Cal_R_Hipz_degree_low = 0x00;

    Data_send.Cal_R_Hipy_degree_flag = 0x00;
    Data_send.Cal_R_Hipy_degree_high = 0x00;
    Data_send.Cal_R_Hipy_degree_low = 0x00;

    Data_send.Cal_R_Knee_degree_flag = 0x00;
    Data_send.Cal_R_Knee_degree_high = 0x00;
    Data_send.Cal_R_Knee_degree_low = 0x00;

    Data_send.Cal_R_Ankle_degree_flag = 0x00;
    Data_send.Cal_R_Ankle_degree_high = 0x00;
    Data_send.Cal_R_Ankle_degree_low = 0x00;

    Data_send.Cal_L_Hipx_degree_flag = 0x00;
    Data_send.Cal_L_Hipx_degree_high = 0x00;
    Data_send.Cal_L_Hipx_degree_low = 0x00;

    Data_send.Cal_L_Hipz_degree_flag = 0x00;
    Data_send.Cal_L_Hipz_degree_high = 0x00;
    Data_send.Cal_L_Hipz_degree_low = 0x00;

    Data_send.Cal_L_Hipy_degree_flag = 0x00;
    Data_send.Cal_L_Hipy_degree_high = 0x00;
    Data_send.Cal_L_Hipy_degree_low = 0x00;

    Data_send.Cal_L_Knee_degree_flag = 0x00;
    Data_send.Cal_L_Knee_degree_high = 0x00;
    Data_send.Cal_L_Knee_degree_low = 0x00;

    Data_send.Cal_L_Ankle_degree_flag = 0x00;
    Data_send.Cal_L_Ankle_degree_high = 0x00;
    Data_send.Cal_L_Ankle_degree_low = 0x00;

    Data_send.Cal_R_Hipx_velocity_flag = 0x00;
    Data_send.Cal_R_Hipx_velocity_high = 0x00;
    Data_send.Cal_R_Hipx_velocity_low = 0x00;

    Data_send.Cal_R_Hipz_velocity_flag = 0x00;
    Data_send.Cal_R_Hipz_velocity_high = 0x00;
    Data_send.Cal_R_Hipz_velocity_low = 0x00;

    Data_send.Cal_R_Hipy_velocity_flag = 0x00;
    Data_send.Cal_R_Hipy_velocity_high = 0x00;
    Data_send.Cal_R_Hipy_velocity_low = 0x00;

    Data_send.Cal_R_Knee_velocity_flag = 0x00;
    Data_send.Cal_R_Knee_velocity_high = 0x00;
    Data_send.Cal_R_Knee_velocity_low = 0x00;

    Data_send.Cal_R_Ankle_velocity_flag = 0x00;
    Data_send.Cal_R_Ankle_velocity_high = 0x00;
    Data_send.Cal_R_Ankle_velocity_low = 0x00;

    Data_send.Cal_L_Hipx_velocity_flag = 0x00;
    Data_send.Cal_L_Hipx_velocity_high = 0x00;
    Data_send.Cal_L_Hipx_velocity_low = 0x00;

    Data_send.Cal_L_Hipz_velocity_flag = 0x00;
    Data_send.Cal_L_Hipz_velocity_high = 0x00;
    Data_send.Cal_L_Hipz_velocity_low = 0x00;

    Data_send.Cal_L_Hipy_velocity_flag = 0x00;
    Data_send.Cal_L_Hipy_velocity_high = 0x00;
    Data_send.Cal_L_Hipy_velocity_low = 0x00;

    Data_send.Cal_L_Knee_velocity_flag = 0x00;
    Data_send.Cal_L_Knee_velocity_high = 0x00;
    Data_send.Cal_L_Knee_velocity_low = 0x00;

    Data_send.Cal_L_Ankle_velocity_flag = 0x00;
    Data_send.Cal_L_Ankle_velocity_high = 0x00;
    Data_send.Cal_L_Ankle_velocity_low = 0x00;



    Data_send.Cal_L_Hipx_kp_high= 0x00;
    Data_send.Cal_L_Hipx_kp_low= 0x00;


    Data_send.Cal_L_Hipz_kp_high= 0x00;
    Data_send.Cal_L_Hipz_kp_low= 0x00;


    Data_send.Cal_L_Hipy_kp_high= 0x00;
    Data_send.Cal_L_Hipy_kp_low= 0x00;

    Data_send.Cal_L_Knee_kp_high= 0x00;
    Data_send.Cal_L_Knee_kp_low= 0x00;


    Data_send.Cal_L_Ankle_kp_high= 0x00;
    Data_send.Cal_L_Ankle_kp_low= 0x00;

    //

    Data_send.Cal_L_Hipx_kd_high= 0x00;
    Data_send.Cal_L_Hipx_kd_low= 0x00;


    Data_send.Cal_L_Hipz_kd_high= 0x00;
    Data_send.Cal_L_Hipz_kd_low= 0x00;


    Data_send.Cal_L_Hipy_kd_high= 0x00;
    Data_send.Cal_L_Hipy_kd_low= 0x00;

    Data_send.Cal_L_Knee_kd_high= 0x00;
    Data_send.Cal_L_Knee_kd_low= 0x00;


    Data_send.Cal_L_Ankle_kd_high= 0x00;
    Data_send.Cal_L_Ankle_kd_low= 0x00;

    Data_send.Cal_R_Hipx_extra_torque_flag = 0x00;
    Data_send.Cal_R_Hipx_extra_torque_high = 0x00;
    Data_send.Cal_R_Hipx_extra_torque_low = 0x00;

    Data_send.Cal_R_Hipz_extra_torque_flag = 0x00;
    Data_send.Cal_R_Hipz_extra_torque_high = 0x00;
    Data_send.Cal_R_Hipz_extra_torque_low = 0x00;

    Data_send.Cal_R_Hipy_extra_torque_flag = 0x00;
    Data_send.Cal_R_Hipy_extra_torque_high = 0x00;
    Data_send.Cal_R_Hipy_extra_torque_low = 0x00;

    Data_send.Cal_R_Knee_extra_torque_flag = 0x00;
    Data_send.Cal_R_Knee_extra_torque_high = 0x00;
    Data_send.Cal_R_Knee_extra_torque_low = 0x00;

    Data_send.Cal_R_Ankle_extra_torque_flag = 0x00;
    Data_send.Cal_R_Ankle_extra_torque_high = 0x00;
    Data_send.Cal_R_Ankle_extra_torque_low = 0x00;

    Data_send.Cal_L_Hipx_extra_torque_flag = 0x00;
    Data_send.Cal_L_Hipx_extra_torque_high = 0x00;
    Data_send.Cal_L_Hipx_extra_torque_low = 0x00;

    Data_send.Cal_L_Hipz_extra_torque_flag = 0x00;
    Data_send.Cal_L_Hipz_extra_torque_high = 0x00;
    Data_send.Cal_L_Hipz_extra_torque_low = 0x00;

    Data_send.Cal_L_Hipy_extra_torque_flag = 0x00;
    Data_send.Cal_L_Hipy_extra_torque_high = 0x00;
    Data_send.Cal_L_Hipy_extra_torque_low = 0x00;

    Data_send.Cal_L_Knee_extra_torque_flag = 0x00;
    Data_send.Cal_L_Knee_extra_torque_high = 0x00;
    Data_send.Cal_L_Knee_extra_torque_low = 0x00;

    Data_send.Cal_L_Ankle_extra_torque_flag = 0x00;
    Data_send.Cal_L_Ankle_extra_torque_high = 0x00;
    Data_send.Cal_L_Ankle_extra_torque_low = 0x00;

    Data_send.servo_theta_flag = 0x00;
    Data_send.servo_theta_high = 0x00;
    Data_send.servo_theta_low = 0x00;


    Data_send.servo_flag = 0x00;

    Data_send.motor_last_flag = 0x00;


    Data_send.m_ucCRCHigh = 0x00;
    Data_send.m_ucCRCLow = 0x00;

}
