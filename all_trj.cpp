#include "all_trj.h"
#include <math.h>
#include <math.h>
#include <stdlib.h>
#include <param_init.h>


float trj_attitude_end = 0;
//landing_trj
float** trj_Land_5th_data;
//int trj_land_5th_count_times = 0;
//x_trj
float **trj_X_trj_5th_data;
int trj_X_trj_5th_count_times = 0;
float trj_x_trj_start_point = 0;
float trj_x_trj_end_point = 0;
float trj_x_trj_time = 0;
//y_trj
float** trj_Y_trj_5th_data;
int trj_Y_trj_5th_count_times = 0;
float trj_y_trj_start_point = 0;
float trj_y_trj_end_point = 0;
float trj_y_trj_time = 0;
//z_trj
float** trj_Z_trj_5th_data;
int trj_Z_trj_5th_count_times = 0;
float trj_z_trj_start_point = 0;
float trj_z_trj_end_point = 0;
float trj_z_trj_time = 0;
//square_trj
float trj_square_time = 0;
float trj_pos_x_end = 0;
float trj_pos_y_end = 0;
double p_body[4];
double v_body[3];

float linear_trajectory(float point_ini,float point_tar,float current_time,float time_all)
{
    //    float point_current;


    point_current = point_ini + (point_tar-point_ini)*(current_time/time_all);

    return  point_current;
}

float *linear_trajectory_plus(double point_ini,float point_tar,float time_all,float step_time)
{

    float *trj;

    for(int i = 0; i < int(time_all/step_time) ; i++)
    {
        trj[i] = point_ini + (point_tar-point_ini)*((i*step_time)/(time_all));
    }


    return  trj;
}


float **Trj_ploy5th_plus(float th00, float th11,
                         float thv00, float thv11,
                         float tha00, float tha11,
                         float T, float T_step)
{

    /*
    th00  起始位置    th11   终止位置
    thv00 起始速度    thv11  终止速度
    tha00 起始加速度  tha11  终止加速度
    T     总时间      T_step 每步时间
    */

    int time_s;
    float h;
    float a0,a1,a2,a3,a4,a5;
    float tt;

    time_s = T/T_step;

    float **trj_all = (float**)malloc(sizeof(float*)*3);

    for (int i = 0; i < 3; i++)
        trj_all[i] = (float*)malloc(sizeof(float)*time_s);

    //    trj_all[0] = new float(T/T_step);
    //    trj_all[1] = new float(T/T_step);
    //    trj_all[2] = new float(T/T_step);

    h = th11 - th00;
    a0 = th00;
    a1 = thv00;
    a2 = tha00/2;
    a3 = (20*h - (8*thv11 + 12*thv00)*T - (3*tha00 - tha11)*pow(T,2)) / (2*pow(T,3));
    a4 = (-30*h + (14*thv11 + 16*thv00)*T + (3*tha00 - 2*tha11)*pow(T,2)) / (2*pow(T,4));
    a5 = (12*h - (thv11 + thv00)*6*T + (tha11 - tha00)*pow(T,2)) / (2*pow(T,5));

    for(int i = 0; i < time_s; i++)
    {
        tt = i*T_step;
        trj_all[0][i] = a0 + a1*tt + a2*pow(tt,2) + a3*pow(tt,3) + a4*pow(tt,4) + a5*pow(tt,5);
        trj_all[1][i] = a1 + 2*a2*tt + 3*a3*pow(tt,2) + 4*a4*pow(tt,3) + 5*a5*pow(tt,4);
        trj_all[2][i] = 2*a2 + 6*a3*tt + 12*a4*pow(tt,2) + 20*pow(tt,3);
    }

    return trj_all;
}


