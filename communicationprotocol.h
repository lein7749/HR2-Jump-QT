#ifndef COMMUNICATIONPROTOCOL_H
#define COMMUNICATIONPROTOCOL_H

#include "type.h"

//#pragma pack (1)					/*指定按1字节对齐*/
struct HC2JCData
{
    u8	data_len;
    u8	zero_flag;
    u8	fan_flag;

    u8	Cal_Fan_F_percent;     //Fan percent

    u8	Cal_Fan_B_percent;
    u8	Cal_Fan_L_percent;
    u8	Cal_Fan_R_percent;

    u8	Cal_R_Hipx_degree_flag;
    u8	Cal_R_Hipx_degree_high;
    u8	Cal_R_Hipx_degree_low;

    u8	Cal_R_Hipz_degree_flag;
    u8	Cal_R_Hipz_degree_high;
    u8	Cal_R_Hipz_degree_low;

    u8	Cal_R_Hipy_degree_flag;
    u8	Cal_R_Hipy_degree_high;
    u8	Cal_R_Hipy_degree_low;

    u8	Cal_R_Knee_degree_flag;
    u8	Cal_R_Knee_degree_high;
    u8	Cal_R_Knee_degree_low;

    u8	Cal_R_Ankle_degree_flag;
    u8	Cal_R_Ankle_degree_high;
    u8	Cal_R_Ankle_degree_low;

    u8	Cal_L_Hipx_degree_flag;
    u8	Cal_L_Hipx_degree_high;
    u8	Cal_L_Hipx_degree_low;

    u8	Cal_L_Hipz_degree_flag;
    u8	Cal_L_Hipz_degree_high;
    u8	Cal_L_Hipz_degree_low;

    u8	Cal_L_Hipy_degree_flag;
    u8	Cal_L_Hipy_degree_high;
    u8	Cal_L_Hipy_degree_low;

    u8	Cal_L_Knee_degree_flag;
    u8	Cal_L_Knee_degree_high;
    u8	Cal_L_Knee_degree_low;

    u8	Cal_L_Ankle_degree_flag;
    u8	Cal_L_Ankle_degree_high;
    u8	Cal_L_Ankle_degree_low;

    u8	Cal_R_Hipx_velocity_flag;
    u8	Cal_R_Hipx_velocity_high;
    u8	Cal_R_Hipx_velocity_low;

    u8	Cal_R_Hipz_velocity_flag;
    u8	Cal_R_Hipz_velocity_high;
    u8	Cal_R_Hipz_velocity_low;

    u8	Cal_R_Hipy_velocity_flag;
    u8	Cal_R_Hipy_velocity_high;
    u8	Cal_R_Hipy_velocity_low;

    u8	Cal_R_Knee_velocity_flag;
    u8	Cal_R_Knee_velocity_high;
    u8	Cal_R_Knee_velocity_low;

    u8	Cal_R_Ankle_velocity_flag;
    u8	Cal_R_Ankle_velocity_high;
    u8	Cal_R_Ankle_velocity_low;

    u8	Cal_L_Hipx_velocity_flag;
    u8	Cal_L_Hipx_velocity_high;
    u8	Cal_L_Hipx_velocity_low;

    u8	Cal_L_Hipz_velocity_flag;
    u8	Cal_L_Hipz_velocity_high;
    u8	Cal_L_Hipz_velocity_low;

    u8	Cal_L_Hipy_velocity_flag;
    u8	Cal_L_Hipy_velocity_high;
    u8	Cal_L_Hipy_velocity_low;

    u8	Cal_L_Knee_velocity_flag;
    u8	Cal_L_Knee_velocity_high;
    u8	Cal_L_Knee_velocity_low;

    u8	Cal_L_Ankle_velocity_flag;
    u8	Cal_L_Ankle_velocity_high;
    u8	Cal_L_Ankle_velocity_low;



    u8	Cal_L_Hipx_kp_high;
    u8	Cal_L_Hipx_kp_low;


    u8	Cal_L_Hipz_kp_high;
    u8	Cal_L_Hipz_kp_low;


    u8	Cal_L_Hipy_kp_high;
    u8	Cal_L_Hipy_kp_low;

    u8	Cal_L_Knee_kp_high;
    u8	Cal_L_Knee_kp_low;

    u8	Cal_L_Ankle_kp_high;
    u8	Cal_L_Ankle_kp_low;


    u8	Cal_L_Hipx_kd_high;
    u8	Cal_L_Hipx_kd_low;

    u8	Cal_L_Hipz_kd_high;
    u8	Cal_L_Hipz_kd_low;


    u8	Cal_L_Hipy_kd_high;
    u8	Cal_L_Hipy_kd_low;


    u8	Cal_L_Knee_kd_high;
    u8	Cal_L_Knee_kd_low;


    u8	Cal_L_Ankle_kd_high;
    u8	Cal_L_Ankle_kd_low;

    //
    u8	Cal_R_Hipx_extra_torque_flag;
    u8	Cal_R_Hipx_extra_torque_high;
    u8	Cal_R_Hipx_extra_torque_low;

    u8	Cal_R_Hipz_extra_torque_flag;
    u8	Cal_R_Hipz_extra_torque_high;
    u8	Cal_R_Hipz_extra_torque_low;

    u8	Cal_R_Hipy_extra_torque_flag;
    u8	Cal_R_Hipy_extra_torque_high;
    u8	Cal_R_Hipy_extra_torque_low;

    u8	Cal_R_Knee_extra_torque_flag;
    u8	Cal_R_Knee_extra_torque_high;
    u8	Cal_R_Knee_extra_torque_low;

    u8	Cal_R_Ankle_extra_torque_flag;
    u8	Cal_R_Ankle_extra_torque_high;
    u8	Cal_R_Ankle_extra_torque_low;

    u8	Cal_L_Hipx_extra_torque_flag;
    u8	Cal_L_Hipx_extra_torque_high;
    u8	Cal_L_Hipx_extra_torque_low;

    u8	Cal_L_Hipz_extra_torque_flag;
    u8	Cal_L_Hipz_extra_torque_high;
    u8	Cal_L_Hipz_extra_torque_low;

    u8	Cal_L_Hipy_extra_torque_flag;
    u8	Cal_L_Hipy_extra_torque_high;
    u8	Cal_L_Hipy_extra_torque_low;

    u8	Cal_L_Knee_extra_torque_flag;
    u8	Cal_L_Knee_extra_torque_high;
    u8	Cal_L_Knee_extra_torque_low;

    u8	Cal_L_Ankle_extra_torque_flag;
    u8	Cal_L_Ankle_extra_torque_high;
    u8	Cal_L_Ankle_extra_torque_low;


    u8	servo_theta_flag;
    u8	servo_theta_high;
    u8	servo_theta_low;

    u8	servo_flag;

    u8	motor_last_flag;


    u8	m_ucCRCHigh;				//*CRC16校验高8位
    u8	m_ucCRCLow;					//*CRC16校验低8位
};
#endif // COMMUNICATIONPROTOCOL_H
//#endif // COMMUNICATIONPROTOCOL_H
//
