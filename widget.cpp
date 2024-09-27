#include "widget.h"
#include "ui_widget.h"
#include <QCloseEvent>
#include <QElapsedTimer>
#include <QString>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <QVector>
#include <string.h>
#include <all_trj.h>
#include <param_init.h>
#include "deal_msg.h"
#include "xlsxdocument.h"
#include "windows.h"
#include "filter.h"
#include <fan_MPC.h>

#define pi acos(-1)
#define toDeg 180/pi
#define toRad pi/180

double *QuaternionToEulerAngles(double qx, double qy, double qz, double qw);
void array_zero();
////motive biaoji
bool QThreadDAQ::controll_flag;
char QThreadDAQ::controll_comman;

void Cal_d_dd(double p_x,double p_y,double p_z);


Eigen::VectorXd new_IK(Eigen::VectorXd body_p,
                       Eigen::VectorXd left_p_sole,float Lfoot_yaw,float Lfoot_pitch,
                       Eigen::VectorXd right_p_sole,float Rfoot_yaw,float Rfoot_pitch);

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    setWindowTitle("Data_Display");
    move(100,50);
    // this->resize(QSize(1300,700));
    //Excel data timer
    fTimer_excel=new QTimer(this);
    fTimer_excel->stop();
    fTimer_excel->setInterval(10);
    connect(fTimer_excel,SIGNAL(timeout()),this,SLOT(on_timer_excel_timeout()));

    fTimer_move=new QTimer(this);
    fTimer_move->stop();
    fTimer_move->setInterval(5);
    connect(fTimer_move,SIGNAL(timeout()),this,SLOT(on_timer_move_timeout()));

    // ui->lineEdit_ip->setText("192.168.1.30");
    ui->lineEdit_ip->setText("192.168.1.30");
    //ui->lineEdit_ip->setText("169.254.146.133");
    ui->lineEdit_port->setText("8089");

    //UI data show timer
    fTimer_data_show=new QTimer(this);
    fTimer_data_show->stop();
    fTimer_data_show->setInterval(100);
    connect(fTimer_data_show,SIGNAL(timeout()),this,SLOT(on_timer_data_show_timeout()));

    fTimer_control=new QTimer(this);
    fTimer_control->stop();
    fTimer_control->setInterval(2);
    connect(fTimer_control,SIGNAL(timeout()),this,SLOT(on_timer_control_timeout()));

    //motive data timer
    fTimer=new QTimer(this);
    fTimer->stop();
    fTimer->setInterval(10);
    connect(fTimer,SIGNAL(timeout()),this,SLOT(on_timer_timeout()));


    dataforsend();
    // setsize

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Cal_d_dd(double p_x,double p_y,double p_z)
{   //calculate the motive data vel and acc
    double world_X_err = 0;
    double world_Y_err = 0;
    acceleration_x__ = acceleration_x_;
    acceleration_x_  = acceleration_x;
    acceleration_x   = (velocity_x-velocity_x_)/delta_t;
    acceleration_y__ = acceleration_y_;
    acceleration_y_  = acceleration_y;
    acceleration_y   = (velocity_y-velocity_y_)/delta_t;
    acceleration_z__ = acceleration_z_;
    acceleration_z_  = acceleration_z;
    acceleration_z   = (velocity_z-velocity_z_)/delta_t;

    velocity_x__ = velocity_x_;
    velocity_x_  = velocity_x;
    velocity_x   = low_past_filter(velocity_x_,(p_x-position_x_)/delta_t);//use the filter
    velocity_y__ = velocity_y_;
    velocity_y_  = velocity_y;
    velocity_y   = low_past_filter(velocity_y_,(p_y-position_y_)/delta_t);//use the filter;
    velocity_z__ = velocity_z_;
    velocity_z_  = velocity_z;
    velocity_z   = limit_low_past_filter(velocity_z_,(p_z-position_z_)/delta_t);//use the filter

    position_x__ = position_x_;
    position_x_  = p_x;
    position_y__ = position_y_;
    position_y_  = p_y;
    position_z__ = position_z;
    position_z_  = p_z;

    //calculate the position error
    world_X_err = -position_x + position_x_desire;//p
    world_Y_err = -position_y + position_y_desire;//p

    r_x_err = world_X_err*cos(world_yaw*toDeg) - world_Y_err*sin(world_yaw*toDeg);
    r_x_errV = (r_x_err - r_x_err_)*100; //d
    r_x_err_ = r_x_err;

    r_y_err = world_X_err*sin(world_yaw*toDeg) + world_Y_err*cos(world_yaw*toDeg);
    r_y_errV = (r_y_err - r_y_err_)*100; //d
    r_y_err_ = r_y_err;
    r_z_err = -position_z + position_z_desire;//p
    r_z_errV = (r_z_err - r_z_err_)*100; //d
    r_z_err_ = r_z_err;

    r_x_err_temp = r_x_err_temp + r_x_err;
    r_y_err_temp = r_y_err_temp + r_y_err;
    r_z_err_temp = r_z_err_temp + r_z_err;

    if(r_x_err_temp > 10)
    {
        r_x_err_temp = 10;
    }
    if(r_x_err_temp < -10)
    {
        r_x_err_temp = -10;
    }
    if(r_y_err_temp > 10)
    {
        r_y_err_temp = 10;
    }
    if(r_y_err_temp < -10)
    {
        r_y_err_temp = -10;
    }
    if(r_z_err_temp > 5)
    {
        r_z_err_temp = 5;
    }
    if(r_z_err_temp < -5)
    {
        r_z_err_temp = -5;
    }
    //calculate the velocity err
    vel_x_err = -velocity_x + velocity_x_desire;//p
    vel_x_errV = (vel_x_err - vel_x_err_)*100; //d
    vel_x_err_ = vel_x_err;
    vel_y_err = -velocity_y + velocity_y_desire;//p
    vel_y_errV = (vel_y_err - vel_y_err_)*100; //d
    vel_y_err_ = vel_y_err;
    vel_z_err = -velocity_z + velocity_z_desire;//p
    vel_z_errV = (vel_z_err - vel_z_err_)*100; //d
    vel_z_err_ = vel_z_err;
    if(i_open_flag_button == 1)
    {
        vel_x_err_temp = vel_x_err_temp + vel_x_err;
        vel_y_err_temp = vel_y_err_temp + vel_y_err;
        vel_z_err_temp = vel_z_err_temp + vel_z_err;
    }//i
    else
    {
        vel_x_err_temp = 0;
        vel_y_err_temp = 0;
        vel_z_err_temp = 0;
    }
    if(vel_x_err_temp > 10)
    {
        vel_x_err_temp = 10;
    }
    if(vel_x_err_temp < -10)
    {
        vel_x_err_temp = -10;
    }
    if(vel_y_err_temp > 10)
    {
        vel_y_err_temp = 10;
    }
    if(vel_y_err_temp < -10)
    {
        vel_y_err_temp = -10;
    }
    if(vel_z_err_temp > 5)
    {
        vel_z_err_temp = 5;
    }
    if(vel_z_err_temp < -5)
    {
        vel_z_err_temp = -5;
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////    IK         /////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int sign(int x) {
    if (x > 0) {
        return 1;
    } else if (x < 0) {
        return -1;
    } else {
        return 0;
    }
}



// 定义 Body 结构体
struct Body {
    Eigen::Vector3d p;
    Eigen::Matrix3d R;
};

// 定义 foot 结构体
struct Foot {
    Eigen::Vector3d p_sole;
    float yaw;
    float pitch;
};

// 定义 Rroll 函数
Eigen::Matrix3d Rroll(float angle) {
    Eigen::Matrix3d rotation;
    rotation << 1, 0, 0,
        0, cos(angle), -sin(angle),
        0, sin(angle), cos(angle);
    return rotation;
}

// 定义 Ryaw 函数
Eigen::Matrix3d Rpitch(float angle) {
    Eigen::Matrix3d rotation;
    rotation << cos(angle), 0, sin(angle),
        0, 1, 0,
        -sin(angle), 0, cos(angle);
    return rotation;
}

// 定义 Rpitch 函数
Eigen::Matrix3d Ryaw(float angle) {
    Eigen::Matrix3d rotation;
    rotation << cos(angle), -sin(angle), 0,
        sin(angle), cos(angle), 0,
        0, 0, 1;
    return rotation;
}

// 定义 IK_L 函数
Eigen::VectorXd IK_L(Body& body, Foot& foot) {
    float D = 116.8;      //两髋距离一半 // 新二号机116.8   旧二号机100
    float A = 105;
    float B = 272;
    float E = -82;      //第二到第三关节高度  // 新二号机-82  旧二号机-91
    float T = 100;      //脚趾长度
    float H = 100;      //脚踝到脚底高度ff
    float q2, q3,foot_pitch,i;

    Eigen::Vector3d toe_vector = Ryaw(foot.yaw/180*pi) * Rpitch(foot.pitch/180*pi) * Eigen::Vector3d(1, 0, 0);    //相对变换，先yaw后pitch
    toe_vector = toe_vector/toe_vector.norm();
    for(i=0;i<3;i++){
        toe_vector[i] = toe_vector[i]*100;
    }
    Eigen::Vector3d toe_pos = foot.p_sole + toe_vector;
    Eigen::Vector3d p2 = body.p + body.R * Eigen::Vector3d(0, D, 0);
    Eigen::Vector3d vTs = toe_pos - foot.p_sole;
    Eigen::Vector3d vTh = toe_pos - p2;
    Eigen::Vector3d nV = vTs.cross(vTh);
    Eigen::Vector3d shadowYoz = nV.cwiseProduct(Eigen::Vector3d(0, 1, 1));

    Eigen::Vector3d ay(0, 1, 0);
    Eigen::Vector3d ax(1, 0, 0);

    if (shadowYoz.isApproxToConstant(0.0)) {
        q2 = 0;
    } else if (shadowYoz.isApproxToConstant(1.0)) {
        q2 = pi / 2;
    } else {
        q2 = sign(shadowYoz(2)) * acos(shadowYoz.dot(ay) / (shadowYoz.norm() * ay.norm()));
    }

    Eigen::Vector3d nV1 = Rroll(q2).transpose() * nV;
    q3 = -sign(nV1(0))*acos(nV1.dot(ay) / nV1.norm());
    Eigen::Vector3d vTs_pitch = body.R.transpose() * Ryaw(q3).transpose() * Rroll(q2).transpose() * vTs;
    foot_pitch = -sign(vTs_pitch(2)) * acos(vTs_pitch.dot(ax)/vTs_pitch.norm());
    Eigen::Matrix3d foot_R = Rroll(q2) * Ryaw(q3) * Rpitch(foot_pitch) * body.R;
    Eigen::Vector3d v_s2a = vTs.cross(nV);
    v_s2a = v_s2a / v_s2a.norm() * H;
    Eigen::Vector3d p_ankle = foot.p_sole + v_s2a;
    Eigen::Vector3d p4 = p2 + Rroll(q2) * Ryaw(q3) * body.R * Eigen::Vector3d(0, 0, E);
    Eigen::Vector3d r = foot_R.transpose() * (p4 - p_ankle);
    float C = r.norm();
    float c5 = (C * C - A * A - B * B) / (2.0 * A * B);

    float q5;
    if (c5 >= 1.0) {
        q5 = 0.0;
    } else if (c5 <= -1.0) {
        q5 = pi;
    } else {
        q5 = acos(c5);
    }

    float q6a = asin((A / C) * sin(pi - q5));
    float q6 = -atan2(r(0), sign(r(2)) * sqrt(r(1) * r(1) + r(2) * r(2))) - q6a;

    Eigen::Matrix3d R = Ryaw(-q3) * Rroll(-q2) * body.R.transpose() * foot_R * Rpitch(-q6 - q5);
    float q4 = atan2(R(0, 2), R(0, 0));

    q2 = q2*toDeg;
    q3 = q3*toDeg;
    q4 = q4*toDeg;
    q5 = q5*toDeg;
    q6 = q6*toDeg;
    Eigen::VectorXd q(5);
    q << q2, q3, q4, q5, q6;

    return q;
}

// 定义 IK_R 函数
Eigen::VectorXd IK_R(Body& body, Foot& foot) {
    float D = -116.8;      //两髋距离一半 // 新二号机-116.8   旧二号机-100
    float A = 105;
    float B = 272;
    float E = -82;      //第二到第三关节高度  // 新二号机-82  旧二号机-91
    float T = 100;      //脚趾长度
    float H = 100;      //脚踝到脚底高度
    float q2, q3,foot_pitch,i;
    Eigen::Vector3d toe_vector = Ryaw(foot.yaw/180*pi) * Rpitch(foot.pitch/180*pi) * Eigen::Vector3d(1, 0, 0);    //相对变换，先yaw后pitch
    toe_vector = toe_vector/toe_vector.norm();
    for(i=0;i<3;i++){
        toe_vector[i] = toe_vector[i]*100;
    }
    Eigen::Vector3d toe_pos = foot.p_sole + toe_vector;
    Eigen::Vector3d p2 = body.p + body.R * Eigen::Vector3d(0, D, 0);
    Eigen::Vector3d vTs = toe_pos - foot.p_sole;
    Eigen::Vector3d vTh = toe_pos - p2;
    Eigen::Vector3d nV = vTs.cross(vTh);
    Eigen::Vector3d shadowYoz = nV.cwiseProduct(Eigen::Vector3d(0, 1, 1));

    Eigen::Vector3d ay(0, 1, 0);
    Eigen::Vector3d ax(1, 0, 0);

    if (shadowYoz.isApproxToConstant(0.0)) {
        q2 = 0;
    } else if (shadowYoz.isApproxToConstant(1.0)) {
        q2 = pi / 2;
    } else {
        q2 = sign(shadowYoz(2)) * acos(shadowYoz.dot(ay) / (shadowYoz.norm() * ay.norm()));
    }

    Eigen::Vector3d nV1 = Rroll(q2).transpose() * nV;
    q3 = -sign(nV1(0))*acos(nV1.dot(ay) / nV1.norm());
    Eigen::Vector3d vTs_pitch = body.R.transpose() * Ryaw(q3).transpose() * Rroll(q2).transpose() * vTs;
    foot_pitch = -sign(vTs_pitch(2)) * acos(vTs_pitch.dot(ax)/vTs_pitch.norm());
    Eigen::Matrix3d foot_R = Rroll(q2) * Ryaw(q3) * Rpitch(foot_pitch) * body.R;
    Eigen::Vector3d v_s2a = vTs.cross(nV);
    v_s2a = v_s2a / v_s2a.norm() * H;
    Eigen::Vector3d p_ankle = foot.p_sole + v_s2a;
    Eigen::Vector3d p4 = p2 + Rroll(q2) * Ryaw(q3) * body.R * Eigen::Vector3d(0, 0, E);
    Eigen::Vector3d r = foot_R.transpose() * (p4 - p_ankle);
    float C = r.norm();
    float c5 = (C * C - A * A - B * B) / (2.0 * A * B);

    float q5;
    if (c5 >= 1.0) {
        q5 = 0.0;
    } else if (c5 <= -1.0) {
        q5 = pi;
    } else {
        q5 = acos(c5);
    }

    float q6a = asin((A / C) * sin(pi - q5));
    float q6 = -atan2(r(0), sign(r(2)) * sqrt(r(1) * r(1) + r(2) * r(2))) - q6a;

    Eigen::Matrix3d R = Ryaw(-q3) * Rroll(-q2) * body.R.transpose() * foot_R * Rpitch(-q6 - q5);
    float q4 = atan2(R(0, 2), R(0, 0));

    q2 = q2*toDeg;
    q3 = q3*toDeg;
    q4 = q4*toDeg;
    q5 = q5*toDeg;
    q6 = q6*toDeg;
    Eigen::VectorXd q(5);
    q << q2, q3, q4, q5, q6;

    return q;
}

Eigen::VectorXd new_IK(Eigen::VectorXd body_p,
                       Eigen::VectorXd left_p_sole,float Lfoot_yaw,float Lfoot_pitch,
                       Eigen::VectorXd right_p_sole,float Rfoot_yaw,float Rfoot_pitch)
{
    // 创建 Body 和 foot 对象
    Body body;
    body.p = body_p;  // 设置适当的初始位置
    // body.R = body_R;  // 单位矩阵
    body.R << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;

    Foot foot_L, foot_R;
    foot_L.p_sole = left_p_sole;  // 设置适当的初始位置
    foot_L.yaw = Lfoot_yaw;  // 角度deg
    foot_L.pitch = Lfoot_pitch;  // 角度deg

    foot_R.p_sole = right_p_sole;  // 设置适当的初始位置
    foot_R.yaw = Rfoot_yaw;  // 角度deg
    foot_R.pitch = Rfoot_pitch;  // 角度deg

    // 调用 IK_L 函数
    Eigen::VectorXd q_L = IK_L(body, foot_L);
    Eigen::VectorXd q_R = IK_R(body, foot_R);
    Eigen::VectorXd motor_degree(10);
    for(int i = 0; i<10; i++)
    {
        if(i < 5)
            motor_degree(i) = q_L(i);
        else
            motor_degree(i) = q_R(i-5);
    }
    //    std::cout << "q:"<< motor_degree <<"\n";
    return motor_degree;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Widget::on_timer_move_timeout()
{
    float** L_Hip_x_trj;
    float** L_Hip_z_trj;
    float** L_Hip_y_trj;
    float** L_Knee_trj;
    float** L_Ankle_trj;

    float** R_Hip_x_trj;
    float** R_Hip_z_trj;
    float** R_Hip_y_trj;
    float** R_Knee_trj;
    float** R_Ankle_trj;

    float** servo_trj;

    // Cal_R_Hipx_velocity=0*100;
    // Cal_R_Hipz_velocity=7*100;
    // Cal_R_Hipy_velocity=7*100;
    // Cal_R_Knee_velocity=7*100;
    // Cal_R_Ankle_velocity=7*100;

    // Cal_L_Hipx_velocity=0*100;
    // Cal_L_Hipz_velocity=7*100;
    // Cal_L_Hipy_velocity=7*100;
    // Cal_L_Knee_velocity=7*100;
    // Cal_L_Ankle_velocity=7*100;
    if (Movement_mode == 0)
    {
        Eigen::VectorXd motor_theta_target;
        //float time_all = 2;

        //Calculate for the target motor degree
        motor_theta_target = new_IK(body_p,left_p,Lfoot_yaw,Lfoot_pitch,right_p,Rfoot_yaw,Rfoot_pitch);

        L_Hipx_target = motor_theta_target(0);
        L_Hipz_target = motor_theta_target(1);
        L_Hipy_target = motor_theta_target(2);
        L_Knee_target = motor_theta_target(3);
        L_Ankle_target = motor_theta_target(4);

        R_Hipx_target = motor_theta_target(5);
        R_Hipz_target = motor_theta_target(6);
        R_Hipy_target = motor_theta_target(7);
        R_Knee_target = motor_theta_target(8);
        R_Ankle_target = motor_theta_target(9);

        //速度符号和角度一样
        // Cal_R_Hipx_velocity=sign(R_Hipx_target)*0*100;
        // Cal_R_Hipz_velocity=sign(R_Hipz_target)*7*100;
        // Cal_R_Hipy_velocity=sign(R_Hipy_target)*7*100;
        // Cal_R_Knee_velocity=sign(R_Knee_target)*7*100;
        // Cal_R_Ankle_velocity=sign(R_Ankle_target)*7*100;

        // Cal_L_Hipx_velocity=sign(L_Hipx_target)*0*100;
        // Cal_L_Hipz_velocity=sign(L_Hipz_target)*7*100;
        // Cal_L_Hipy_velocity=sign(L_Hipy_target)*7*100;
        // Cal_L_Knee_velocity=sign(L_Knee_target)*7*100;
        // Cal_L_Ankle_velocity=sign(L_Ankle_target)*7*100;
    }
    else if (Movement_mode == 1)
    {
        L_Hipx_target  = ui->lineEdit_L_Hipx->text().toFloat();
        L_Hipz_target  = ui->lineEdit_L_Hipz->text().toFloat();
        L_Hipy_target  = ui->lineEdit_L_Hipy->text().toFloat();
        L_Knee_target  = ui->lineEdit_L_Knee->text().toFloat();
        L_Ankle_target = ui->lineEdit_L_Ankle->text().toFloat();

        R_Hipx_target  = ui->lineEdit_R_Hipx->text().toFloat();
        R_Hipz_target  = ui->lineEdit_R_Hipz->text().toFloat();
        R_Hipy_target  = ui->lineEdit_R_Hipy->text().toFloat();
        R_Knee_target  = ui->lineEdit_R_Knee->text().toFloat();
        R_Ankle_target = ui->lineEdit_R_Ankle->text().toFloat();
    }



    //    move from current degree to the target degree

    L_Hip_x_trj = Trj_ploy5th_plus(L_Hipx_start, L_Hipx_target,
                                   0, 0,
                                   0, 0,
                                   time_all, 0.005);

    L_Hip_z_trj = Trj_ploy5th_plus(L_Hipz_start, L_Hipz_target,
                                   0, 0,
                                   0, 0,
                                   time_all, 0.005);


    L_Hip_y_trj = Trj_ploy5th_plus(L_Hipy_start, L_Hipy_target,
                                   0, 0,
                                   0, 0,
                                   time_all, 0.005);



    L_Knee_trj = Trj_ploy5th_plus(L_Knee_start, L_Knee_target,
                                  0, 0,
                                  0, 0,
                                  time_all, 0.005);



    L_Ankle_trj = Trj_ploy5th_plus(L_Ankle_start, L_Ankle_target,
                                   0, 0,
                                   0, 0,
                                   time_all, 0.005);


    //R
    R_Hip_x_trj = Trj_ploy5th_plus(R_Hipx_start, R_Hipx_target,
                                   0, 0,
                                   0, 0,
                                   time_all, 0.005);

    R_Hip_z_trj = Trj_ploy5th_plus(R_Hipz_start, R_Hipz_target,
                                   0, 0,
                                   0, 0,
                                   time_all, 0.005);


    R_Hip_y_trj = Trj_ploy5th_plus(R_Hipy_start, R_Hipy_target,
                                   0, 0,
                                   0, 0,
                                   time_all, 0.005);



    R_Knee_trj = Trj_ploy5th_plus(R_Knee_start, R_Knee_target,
                                  0, 0,
                                  0, 0,
                                  time_all, 0.005);



    R_Ankle_trj = Trj_ploy5th_plus(R_Ankle_start, R_Ankle_target,
                                   0, 0,
                                   0, 0,
                                   time_all, 0.005);

    servo_trj = Trj_ploy5th_plus(servo_start, servo_target,
                                 0, 0,
                                 0, 0,
                                 time_all, 0.005);



    Cal_L_Hipx_degree = L_Hip_x_trj[0][motor_move_count_times]*100;

    Cal_L_Hipz_degree = L_Hip_z_trj[0][motor_move_count_times]*100;

    Cal_L_Hipy_degree = L_Hip_y_trj[0][motor_move_count_times]*100;

    Cal_L_Knee_degree = L_Knee_trj[0][motor_move_count_times]*100;

    Cal_L_Ankle_degree  =L_Ankle_trj[0][motor_move_count_times]*100;


    Cal_R_Hipx_degree = R_Hip_x_trj[0][motor_move_count_times]*100;

    Cal_R_Hipz_degree = R_Hip_z_trj[0][motor_move_count_times]*100;

    Cal_R_Hipy_degree = R_Hip_y_trj[0][motor_move_count_times]*100;

    Cal_R_Knee_degree = R_Knee_trj[0][motor_move_count_times]*100;

    Cal_R_Ankle_degree = R_Ankle_trj[0][motor_move_count_times]*100;

    Cal_servo_degree = servo_trj[0][motor_move_count_times]*100;



    if(time_all<(motor_move_count_times+2)*0.005)
    {
        fTimer_move->stop();
        motor_move_count_times = 0;
    }
    else
    {
        motor_move_count_times = motor_move_count_times + 1;
    }


}

void Widget::on_timer_data_show_timeout()
{
    int displaytime = 0;
    fdisplaycounter.start();


    ui->label_Cal_R_Hipx_degree->setText(QString::number(Cal_R_Hipx_degree*0.01));
    ui->label_Cal_R_Hipz_degree->setText(QString::number(Cal_R_Hipz_degree*0.01));
    ui->label_Cal_R_Hipy_degree->setText(QString::number(Cal_R_Hipy_degree*0.01));
    ui->label_Cal_R_Knee_degree->setText(QString::number(Cal_R_Knee_degree*0.01));
    ui->label_Cal_R_Ankle_degree->setText(QString::number(Cal_R_Ankle_degree*0.01));
    ui->label_Cal_L_Hipx_degree->setText(QString::number(Cal_L_Hipx_degree*0.01));
    ui->label_Cal_L_Hipz_degree->setText(QString::number(Cal_L_Hipz_degree*0.01));
    ui->label_Cal_L_Hipy_degree->setText(QString::number(Cal_L_Hipy_degree*0.01));
    ui->label_Cal_L_Knee_degree->setText(QString::number(Cal_L_Knee_degree*0.01));
    ui->label_Cal_L_Ankle_degree->setText(QString::number(Cal_L_Ankle_degree*0.01));
    ui->label_Cal_servo_degree->setText(QString::number(Cal_servo_degree*0.01));

    ui->label__Fan_F_Cal->setText(QString::number(Cal_Fan_F_percent));
    ui->label__Fan_B_Cal->setText(QString::number(Cal_Fan_B_percent));
    ui->label__Fan_L_Cal->setText(QString::number(Cal_Fan_L_percent));
    ui->label__Fan_R_Cal->setText(QString::number(Cal_Fan_R_percent));

    ui->label_Rectify_Roll->setText(QString::number(imu_euler_roll));
    ui->label_Rectify_Pitch->setText(QString::number(imu_euler_pitch));
    ui->label_Rectify_Yaw->setText(QString::number(imu_euler_yaw));

    ui->label_Rollv->setText(QString::number(imu_euler_roll_v));
    ui->label_Pitchv->setText(QString::number(imu_euler_pitch_v));
    ui->label_Yawv->setText(QString::number(imu_euler_yaw_v));

    ui->label_R_Hipx_connect_flag->setText(QString::number(R_Hipx_connect_flag));
    ui->label_R_Hipz_connect_flag->setText(QString::number(R_Hipz_connect_flag));
    ui->label_R_Hipy_connect_flag->setText(QString::number(R_Hipy_connect_flag));
    ui->label_R_Knee_connect_flag->setText(QString::number(R_Knee_connect_flag));
    ui->label_R_Ankle_connect_flag->setText(QString::number(R_Ankle_connect_flag));
    ui->label_L_Hipx_connect_flag->setText(QString::number(L_Hipx_connect_flag));
    ui->label_L_Hipz_connect_flag->setText(QString::number(L_Hipz_connect_flag));
    ui->label_L_Hipy_connect_flag->setText(QString::number(L_Hipy_connect_flag));
    ui->label_L_Knee_connect_flag->setText(QString::number(L_Knee_connect_flag));
    ui->label_L_Ankle_connect_flag->setText(QString::number(L_Ankle_connect_flag));

    ui->label_R_Hipx_degree->setText(QString::number(R_Hipx_degree));
    ui->label_R_Hipz_degree->setText(QString::number(R_Hipz_degree));
    ui->label_R_Hipy_degree->setText(QString::number(R_Hipy_degree));
    ui->label_R_Knee_degree->setText(QString::number(R_Knee_degree));
    ui->label_R_Ankle_degree->setText(QString::number(R_Ankle_degree));
    ui->label_L_Hipx_degree->setText(QString::number(L_Hipx_degree));
    ui->label_L_Hipz_degree->setText(QString::number(L_Hipz_degree));
    ui->label_L_Hipy_degree->setText(QString::number(L_Hipy_degree));
    ui->label_L_Knee_degree->setText(QString::number(L_Knee_degree));
    ui->label_L_Ankle_degree->setText(QString::number(L_Ankle_degree));

    ui->label_R_Hipx_vel->setText(QString::number(R_Hipx_velocity));
    ui->label_R_Hipz_vel->setText(QString::number(R_Hipz_velocity));
    ui->label_R_Hipy_vel->setText(QString::number(R_Hipy_velocity));
    ui->label_R_Knee_vel->setText(QString::number(R_Knee_velocity));
    ui->label_R_Ankle_vel->setText(QString::number(R_Ankle_velocity));
    ui->label_L_Hipx_vel->setText(QString::number(L_Hipx_velocity));
    ui->label_L_Hipz_vel->setText(QString::number(L_Hipz_velocity));
    ui->label_L_Hipy_vel->setText(QString::number(L_Hipy_velocity));
    ui->label_L_Knee_vel->setText(QString::number(L_Knee_velocity));
    ui->label_L_Ankle_vel->setText(QString::number(L_Ankle_velocity));

    ui->label_R_Hipx_tor->setText(QString::number(R_Hipx_current));
    ui->label_R_Hipz_tor->setText(QString::number(R_Hipz_current));
    ui->label_R_Hipy_tor->setText(QString::number(R_Hipy_current));
    ui->label_R_Knee_tor->setText(QString::number(R_Knee_current));
    ui->label_R_Ankle_tor->setText(QString::number(R_Ankle_current));
    ui->label_L_Hipx_tor->setText(QString::number(L_Hipx_current));
    ui->label_L_Hipz_tor->setText(QString::number(L_Hipz_current));
    ui->label_L_Hipy_tor->setText(QString::number(L_Hipy_current));
    ui->label_L_Knee_tor->setText(QString::number(L_Knee_current));
    ui->label_L_Ankle_tor->setText(QString::number(L_Ankle_current));

    ui->label_x_pos->setText(QString::number(position_x));
    ui->label_y_pos->setText(QString::number(position_y));
    ui->label_z_pos->setText(QString::number(position_z));
    ui->label_x_vel->setText(QString::number(velocity_x));
    ui->label_y_vel->setText(QString::number(velocity_y));
    ui->label_z_vel->setText(QString::number(velocity_z));

    ui->label_L_Contact_flag->setText(QString::number(robot_state_L));
    ui->label_R_Contact_flag->setText(QString::number(robot_state_R));
    ui->label_All_Contact_flag->setText(QString::number(robot_state));

    ui->label_L_Contact->setText(QString::number(Collision_flag_L));
    ui->label_R_Contact->setText(QString::number(Collision_flag_R));

    // MPC 参数显示
    ui->label_F_f1->setText(QString::number(F_f1));
    ui->label_F_f2->setText(QString::number(F_f2));
    ui->label_F_b1->setText(QString::number(F_b1));
    ui->label_F_b2->setText(QString::number(F_b2));
    ui->label_F_f->setText(QString::number(F_f));
    ui->label_F_b->setText(QString::number(F_b));
    ui->label_F_x->setText(QString::number(F_x));
    ui->label_F_thta->setText(QString::number(F_thta));

    ui->label_upping_data_time->setText(str_upping_time);
    ui->label_downing_data_time->setText(str_downing_time);

    displaytime = fdisplaycounter.nsecsElapsed();
    int us = displaytime/1000;
    QString str = QString::asprintf("%d us",us);
    qDebug() << "displaytime: " << str;
}

void Widget::on_timer_timeout()
{


    //biaoji // 仿真时注释
    position_x = motive.getData1()-position_ref_x;
    position_y = -motive.getData0()-position_ref_y;
    position_z = motive.getData2()-position_ref_z;


    position_z = low_past_filter_z(position_z_,position_z);

    Cal_d_dd(position_x,position_y,position_z);


    if( rectify_motive_flag == 0)
    {
        //save 100 sequence data for rectifying
        motive_position_x_array[rectify_motive_count] = position_x;
        motive_position_y_array[rectify_motive_count] = position_y;
        motive_position_z_array[rectify_motive_count] = position_z;

        rectify_motive_count++;

        if(rectify_motive_count==100)
        {
            position_ref_x = (std::accumulate(motive_position_x_array.begin(),motive_position_x_array.end(),0.0))/100;
            position_ref_y = (std::accumulate(motive_position_y_array.begin(),motive_position_y_array.end(),0.0))/100;
            position_ref_z = (std::accumulate(motive_position_z_array.begin(),motive_position_z_array.end(),0.0))/100;
            rectify_motive_flag = 1;
        }
    }

    //计算下发时间
    int tmMsec = fTimeCounter.elapsed();
    int ms = tmMsec%1000;
    QString str = QString::asprintf("%d milesecond",ms);
    // ui->label_motive_data_time->setText(str);
    fTimeCounter.start();


}



Widget::~Widget()
{
    delete ui;
}


void Widget::on_pushButton_clicked()
{
    if (Movement_mode == 0)
    {
        ui->lineEdit_IK_Base_P1->setText("0");
        ui->lineEdit_IK_Base_P2->setText("0");
        ui->lineEdit_IK_Base_P3->setText("568");

        ////////////// right_foot pos and rot
        ui->lineEdit_IK_R_P1->setText("-150");
        ui->lineEdit_IK_R_P2->setText("-100");
        ui->lineEdit_IK_R_P3->setText("70");


        ui->lineEdit_IK_R_yaw->setText("0");
        ui->lineEdit_IK_R_pitch->setText("25"); //踮脚尖实验

        ////////////// left_foot pos and rot
        ui->lineEdit_IK_L_P1->setText("-150");
        ui->lineEdit_IK_L_P2->setText("100");
        ui->lineEdit_IK_L_P3->setText("70");

        ui->lineEdit_IK_L_yaw->setText("0");
        ui->lineEdit_IK_L_pitch->setText("25"); //踮脚尖实验
        ui->lineEdit_input_servo_degree->setText("90");

        ///////连续弹跳实验///////
        // ui->lineEdit_IK_Base_P1->setText("0");
        // ui->lineEdit_IK_Base_P2->setText("0");
        // ui->lineEdit_IK_Base_P3->setText("520"); // 490是仿真的数据，520为样机数据

        // ////////////// right_foot pos and rot
        // ui->lineEdit_IK_R_P1->setText("-10");
        // ui->lineEdit_IK_R_P2->setText("-116.8");
        // ui->lineEdit_IK_R_P3->setText("0");


        // ui->lineEdit_IK_R_yaw->setText("0");
        // ui->lineEdit_IK_R_pitch->setText("8");

        // ////////////// left_foot pos and rot
        // ui->lineEdit_IK_L_P1->setText("-10");
        // ui->lineEdit_IK_L_P2->setText("116.8");
        // ui->lineEdit_IK_L_P3->setText("0");

        // ui->lineEdit_IK_L_yaw->setText("0");
        // ui->lineEdit_IK_L_pitch->setText("8");
        // ui->lineEdit_input_servo_degree->setText("90");
        zero_flag =0;
        time_all = 2;
        on_pushButton_Cal_clicked();

    }
    else if (Movement_mode == 1)
    {
        ui->lineEdit_L_Hipx->setText("0");
        ui->lineEdit_L_Hipz->setText("0");
        ui->lineEdit_L_Hipy->setText("-43.78");
        ui->lineEdit_L_Knee->setText("58.42");
        ui->lineEdit_L_Ankle->setText("-15");// 起跳姿势

        ui->lineEdit_R_Hipx->setText("0");
        ui->lineEdit_R_Hipz->setText("0");
        ui->lineEdit_R_Hipy->setText("-43.78");
        ui->lineEdit_R_Knee->setText("58.42");
        ui->lineEdit_R_Ankle->setText("-15");// 起跳姿势

        ui->lineEdit_input_servo_degree->setText("90"); //舵机角度保持90度

        Cal_L_Ankle_kp =120*100;
        Cal_L_Ankle_kd =1*100;

        zero_flag =0;
        time_all = 2;

        on_pushButton_Cal_clicked();
    }
}


void Widget::on_pushButton_Cal_clicked()
{
    if (Movement_mode == 0)
    {
        body_p(0,0) = ui->lineEdit_IK_Base_P1->text().toFloat();
        body_p(1,0) = ui->lineEdit_IK_Base_P2->text().toFloat();
        body_p(2,0) = ui->lineEdit_IK_Base_P3->text().toFloat();
        ////////////////////////////////////////////////////////////////////////////////////////
        right_p(0,0) = ui->lineEdit_IK_R_P1->text().toFloat();
        right_p(1,0) = ui->lineEdit_IK_R_P2->text().toFloat();
        right_p(2,0) = ui->lineEdit_IK_R_P3->text().toFloat();

        Rfoot_yaw = ui->lineEdit_IK_R_yaw->text().toFloat();
        Rfoot_pitch = ui->lineEdit_IK_R_pitch->text().toFloat();
        ////////////////////////////////////////////////////////////////////////////////////////
        left_p(0,0) = ui->lineEdit_IK_L_P1->text().toFloat();
        left_p(1,0) = ui->lineEdit_IK_L_P2->text().toFloat();
        left_p(2,0) = ui->lineEdit_IK_L_P3->text().toFloat();

        Lfoot_yaw = ui->lineEdit_IK_L_yaw->text().toFloat();
        Lfoot_pitch = ui->lineEdit_IK_L_pitch->text().toFloat();
    }

    //读取文本



    R_Hipy_start = ui->label_Cal_R_Hipy_degree->text().toFloat();
    R_Hipz_start = ui->label_Cal_R_Hipz_degree->text().toFloat();
    R_Hipx_start = ui->label_Cal_R_Hipx_degree->text().toFloat();
    R_Knee_start = ui->label_Cal_R_Knee_degree->text().toFloat();
    R_Ankle_start = ui->label_Cal_R_Ankle_degree->text().toFloat();

    L_Hipy_start = ui->label_Cal_L_Hipy_degree->text().toFloat();
    L_Hipz_start = ui->label_Cal_L_Hipz_degree->text().toFloat();
    L_Hipx_start = ui->label_Cal_L_Hipx_degree->text().toFloat();
    L_Knee_start = ui->label_Cal_L_Knee_degree->text().toFloat();
    L_Ankle_start = ui->label_Cal_L_Ankle_degree->text().toFloat();

    servo_start = ui->label_Cal_servo_degree->text().toFloat();

    servo_target = ui->lineEdit_input_servo_degree->text().toFloat();


    motor_move_count_times=0.0;
    motor_move_time = 0.0;
    fTimer_move->start();
}


void Widget::on_pushButton_stand_clicked()
{
    if (Movement_mode == 0)
    {
        ui->lineEdit_IK_Base_P1->setText("0");
        ui->lineEdit_IK_Base_P2->setText("0");
        ui->lineEdit_IK_Base_P3->setText("559"); // 490是仿真的数据


        ////////////// right_foot pos and rot
        ui->lineEdit_IK_R_P1->setText("0");
        ui->lineEdit_IK_R_P2->setText("-116.8");
        ui->lineEdit_IK_R_P3->setText("0");

        ui->lineEdit_IK_R_yaw->setText("0");
        ui->lineEdit_IK_R_pitch->setText("0");

        ////////////// left_foot pos and rot
        ui->lineEdit_IK_L_P1->setText("0");
        ui->lineEdit_IK_L_P2->setText("116.8");
        ui->lineEdit_IK_L_P3->setText("0");

        ui->lineEdit_IK_L_yaw->setText("0");
        ui->lineEdit_IK_L_pitch->setText("0");

        time_all = 1;

        //UI设置文本
        on_pushButton_Cal_clicked();
    }
    else if (Movement_mode == 1)
    {
        ui->lineEdit_L_Hipx->setText("0");
        ui->lineEdit_L_Hipz->setText("0");
        ui->lineEdit_L_Hipy->setText("0");
        ui->lineEdit_L_Knee->setText("0");
        ui->lineEdit_L_Ankle->setText("0");

        ui->lineEdit_R_Hipx->setText("0");
        ui->lineEdit_R_Hipz->setText("0");
        ui->lineEdit_R_Hipy->setText("0");
        ui->lineEdit_R_Knee->setText("0");
        ui->lineEdit_R_Ankle->setText("0");

        time_all = 1;
        on_pushButton_Cal_clicked();
    }
}


void Widget::on_pushButton_2_clicked()
{
    send_mode = 1;
    deal.start();


}


void Widget::on_pushButton_stop_clicked()
{
    deal.stopmsg();
    fan_text_count = 0;

    fTimer_control->stop();
    Cal_Fan_F_percent = 0;
    Cal_Fan_B_percent = 0;
    Cal_Fan_L_percent = 0;
    Cal_Fan_R_percent = 0;

    JET_HR2.m_stop = true;
    // JET_HR2.terminate();

    // motive.stopThread();
}


void Widget::on_pushButton_data_start_clicked()
{
    fTimer_data_show->start();
    fTimer_control->start();

    //打开motive线程
    QThreadDAQ::controll_flag = true;
    QThreadDAQ::controll_comman ='1';
    motive.start();
    fTimer->start();

}



void Widget::on_pushButton_text_servo_clicked()
{
    ui->lineEdit_input_servo_degree->setText("70");
    on_pushButton_Cal_clicked();
}


void Widget::on_pushButton_text_servo_2_clicked()
{
    ui->lineEdit_input_servo_degree->setText("90");
    on_pushButton_Cal_clicked();
}


void Widget::on_pushButton_fan_text_clicked()
{
    fan_text_flag = 1;
    fan_text_count = 0;
}

void Widget::on_timer_control_timeout()
{
    float servo_control = 0;
    float fan_control = 0;
    float tor_servo = 0;
    float step=0;
    float time_num=0;

    if(fly_flag == 1)
    {

        imu_euler_pitch_v_new = low_past_filter(imu_euler_pitch_v_nn,imu_euler_pitch_v);
        qDebug() << "imu_euler_pitch_v_new: " << imu_euler_pitch_v_new;

        servo_control = kp_servo*(imu_euler_pitch+5) + kd_servo*imu_euler_pitch_v_new;

        imu_euler_pitch_v_nn = imu_euler_pitch_v_new;

        servo_control = limit(servo_control,-40,40);
        Cal_servo_degree = (90-servo_control)*100;

        fan_control = kp_fan*imu_euler_pitch + kd_fan*imu_euler_pitch_v;
        fan_control = limit(fan_control,-30,30);

        Cal_Fan_F_percent = base_fan - fan_control;
        Cal_Fan_B_percent = base_fan + fan_control;

    // if (jump_flag == 1)
    // {

    //     //计算轨迹
    //     ankle_vel = -angle_jump/time_jump;
    //     step = ankle_vel*0.002;
    //     time_num = int(time_jump/0.002);
    //     for (int i=0;i<time_num;i++)
    //     {
    //         ankle_angle_tra[i] = L_Ankle_target - step*i;
    //     }

    //     Cal_R_Ankle_degree = ankle_angle_tra[jump_num]*100;
    //     Cal_L_Ankle_degree = ankle_angle_tra[jump_num]*100;
    //     Cal_R_Ankle_velocity = 0*100;
    //     Cal_L_Ankle_velocity = 0*100;
    //     Cal_R_Ankle_extra_torque = tor_jump*100;
    //     Cal_L_Ankle_extra_torque = tor_jump*100;

    //     jump_num = jump_num+1;
    //     if (jump_num >= time_num)
    //     {
    //         jump_flag = 0;
    //         jump_num = 0;
    //         Cal_R_Ankle_degree = R_Ankle_target*100;
    //         Cal_L_Ankle_degree = L_Ankle_target*100;
    //         Cal_R_Ankle_velocity = 0;
    //         Cal_L_Ankle_velocity = 0;
    //         Cal_R_Ankle_extra_torque = 0;
    //         Cal_L_Ankle_extra_torque = 0;
    //     }

    // }

        // 接触测试
        if (Contact_test_flag == 1)
        {

            // 右腿接触检测
            if (R_Ankle_current > 1.2)
                Collision_flag_R = 1;
            else
                Collision_flag_R = 0;

            robot_state_R = 0;
            if (robot_state_R == 0)
            {
                if (Collision_flag_R == 1)
                    robot_state_R = 1;
            }
            if (robot_state_R == 1)
            {
                if (R_Ankle_velocity > 2)
                    robot_state_R = 2;
            }
            if (robot_state_R == 2)
            {
                if (Collision_flag_R == 0)
                    robot_state_R = 0;
            }

            // 左腿接触检测
            if (L_Ankle_current > 1.2)
                Collision_flag_L = 1;
            else
                Collision_flag_L = 0;

            robot_state_L = 0;
            if (robot_state_L == 0)//腾空时检测落地
            {
                if (Collision_flag_L == 1)
                    robot_state_L = 1;
            }
            if (robot_state_L == 1)//检测伸长相
            {
                if (L_Ankle_velocity > 2)
                    robot_state_L = 2;
            }
            if (robot_state_L == 2)//检测离地
            {
                if (Collision_flag_L == 0)
                    robot_state_L = 0;
            }
            // if (robot_state_L == 0)//检测是否达到最高点
            // {
            //     if (velocity_z < 0.02)
            //         robot_state_L = 3;
            // }
            // if (robot_state_L == 0)//检测离开最高点
            // {
            //     if (velocity_z < -0.02)
            //         robot_state_L = 0;
            // }
            if (robot_state_L == 3)
            {
                h_max = position_z;
            }

            // robot_state = 0;//腾空相
            // 当左右腿均触地时视为机器人触地
            if ((robot_state_R == 1) || (robot_state_L == 1))
                robot_state = 1;
            if ((robot_state_R == 2) || (robot_state_L == 2))
                robot_state = 2;


            if ((robot_state_R == 0) && (robot_state_L == 0))
                robot_state = 0;

            qDebug() << "robot_state_R：" << robot_state_R;
            qDebug() << "robot_state_L：" << robot_state_L;
            qDebug() << "robot_state："   << robot_state;



            // 脚踝测试策略
            // if (Collision_flag_R == 2 && Collision_flag_L == 2)
            //     jump_start_flag = 1;
            // if (jump_start_flag == 1)
            // {
            //     ankle_vel = -angle_jump/time_jump;
            //     step = ankle_vel*0.002;
            //     time_num = int(time_jump/0.002);
            //     for (int i=0;i<time_num;i++)
            //     {
            //         ankle_angle_tra[i] = L_Ankle_target - step*i;
            //     }

            //     Cal_R_Ankle_degree = ankle_angle_tra[jump_num]*100;
            //     Cal_L_Ankle_degree = ankle_angle_tra[jump_num]*100;
            //     Cal_R_Ankle_velocity = 0*100;
            //     Cal_L_Ankle_velocity = 0*100;
            //     Cal_R_Ankle_extra_torque = 14*100;
            //     Cal_L_Ankle_extra_torque = 14*100;

            //     jump_num = jump_num+1;
            //     if (jump_num >= time_num)
            //     {
            //         jump_start_flag = 0;
            //         jump_num = 0;
            //         Cal_R_Ankle_degree = R_Ankle_target*100;
            //         Cal_L_Ankle_degree = L_Ankle_target*100;
            //         Cal_R_Ankle_velocity = 0;
            //         Cal_L_Ankle_velocity = 0;
            //         Cal_R_Ankle_extra_torque = 0;
            //         Cal_L_Ankle_extra_torque = 0;

            //     }

            // }


            // 脚踝策略
            float F_Roll_Support = 0.0;
            float kp_h = 0;
            float h_expect = 0.8;

            Kp_Roll_Support = 0;
            kd_Roll_Support = 0;
            F_Roll_Support = Kp_Roll_Support*(imu_euler_roll - 0) + kd_Roll_Support*(imu_euler_roll_v);
            if (F_Roll_Support <= -1.5)
            {
                F_Roll_Support = -1.5;
            }
            else if (F_Roll_Support >= 1.5)
            {
                F_Roll_Support = 1.5;
            }
            // 左脚脚踝策略
            int L_Ankle_kp_temp = 10;
            int L_Ankle_kd_temp = 0;
            float exp_L_Ankle_degree = 10;
            int R_Ankle_kp_temp = 10;
            int R_Ankle_kd_temp = 0;
            float exp_R_Ankle_degree = 10;
            if (take_off_flag == 1)
            {
                ankle_vel = -angle_jump/time_jump;
                step = ankle_vel*0.002;
                time_num = int(time_jump/0.002);
                for (int i=0;i<time_num;i++)
                {
                    ankle_angle_tra[i] = L_Ankle_target - step*i;
                }
                Cal_L_Ankle_kp = 100*100;
                Cal_L_Ankle_kd = 1*100;
                Cal_R_Ankle_degree = ankle_angle_tra[jump_num]*100;
                Cal_L_Ankle_degree = ankle_angle_tra[jump_num]*100;
                Cal_R_Ankle_velocity = 0*100;
                Cal_L_Ankle_velocity = 0*100;
                Cal_R_Ankle_extra_torque = 18*100;
                Cal_L_Ankle_extra_torque = 18*100;

                jump_num = jump_num+1;
                if (jump_num >= time_num)
                {
                    // take_off_flag = 0;
                    jump_start_flag = 0;
                    jump_num = time_num;
                    Cal_R_Ankle_degree = ankle_angle_tra[jump_num-1]*100;
                    Cal_L_Ankle_degree = ankle_angle_tra[jump_num-1]*100;
                    Cal_R_Ankle_velocity = 0;
                    Cal_L_Ankle_velocity = 0;
                    Cal_R_Ankle_extra_torque = 0;
                    Cal_L_Ankle_extra_torque = 0;
                    if (robot_state == 0)
                    {
                        take_off_flag = 0;
                    }

                }

                // Cal_L_Ankle_kp = 100*100;
                // Cal_L_Ankle_kd = 1*100;

                // Cal_L_Ankle_degree = 25*100;
                // Cal_R_Ankle_degree = 25*100;
                // Cal_L_Ankle_extra_torque = 10*100;
                // Cal_R_Ankle_extra_torque = 18*100;
            }
            else if (robot_state == 0)
            {
                take_off_flag = 0;
                Cal_L_Ankle_kp = 10*100;
                Cal_L_Ankle_kd = 0.2*100;

                Cal_L_Ankle_degree = 10*100;
                Cal_R_Ankle_degree = 10*100;
                Cal_L_Ankle_extra_torque = 0*100;
                Cal_R_Ankle_extra_torque = 0*100;
            }
            else if (robot_state == 1) // 左脚压缩相策略
            {
                Cal_L_Ankle_kp = 5*100;
                Cal_L_Ankle_kd = 0.2*100;

                Cal_L_Ankle_degree = 10*100;
                Cal_R_Ankle_degree = 10*100;
                Cal_L_Ankle_extra_torque = 0*100;
                Cal_R_Ankle_extra_torque = 0*100;
            }
            else if (robot_state == 2)
            {
                Cal_L_Ankle_kp = 100*100;
                Cal_L_Ankle_kd = 1*100;
                Cal_L_Ankle_degree = 25*100;
                Cal_R_Ankle_degree = 25*100;
                // Cal_L_Ankle_extra_torque = L_Ankle_kp_temp*(exp_L_Ankle_degree - L_Ankle_degree) + 10
                //                            + kp_h*(h_expect - h_max) + F_Roll_Support;
                Cal_L_Ankle_extra_torque =18*100;
                Cal_R_Ankle_extra_torque =18*100;
                // if (L_Ankle_degree > 50 || R_Ankle_degree > 50)
                // {
                //     Cal_L_Ankle_kp = 10*100;
                //     Cal_L_Ankle_kd = 0.5*100;
                //     Cal_L_Ankle_degree = 50*100;
                //     Cal_R_Ankle_degree = 50*100;
                //     Cal_L_Ankle_extra_torque =0*100;
                //     Cal_R_Ankle_extra_torque =0*100;
                // }
            }

            // Cal_Fan_B_percent = 100;// 转化为百分比风力

            // 膝关节策略
            // if (robot_state == 1 || robot_state == 2 || robot_state == 0)
            // {

            //     float force_hip = 0;
            //     hip_Supporting_ecpect = -0;
            //     kp_hip_Support = 1.5;
            //     kd_hip_Support = -0;
            //     force_hip = kp_hip_Support*(imu_euler_pitch - hip_Supporting_ecpect) + kd_hip_Support*(imu_euler_pitch_v);
            //     Cal_L_Knee_kp = 0*100;
            //     Cal_L_Knee_kd = 0*100;
            //     force_hip = limit(force_hip, -18,18);
            //     Cal_L_Knee_extra_torque = force_hip*100;

            //     qDebug() << "膝关节控制: " << force_hip;
            // }
            // else
            // {
            //     Cal_L_Knee_kp = 100*100;
            //     Cal_L_Knee_kd = 0.5*100;
            //     Cal_L_Knee_torque = 0*100;
            // }
            // // 脚掌风机策略
            // if (robot_state == 0)
            // {
            //     float BaseLink_Roll_expect = 0;
            //     float F_Roll;
            //     K_Roll_air = 180;
            //     kd_Roll_air = 20;
            //     F_Roll = K_Roll_air*(imu_euler_roll - BaseLink_Roll_expect) + kd_Roll_air*(imu_euler_roll_v);
            //     if (F_Roll >= 0)
            //     {
            //         Cal_Fan_R_percent = F_Roll;
            //         Cal_Fan_L_percent = 0;
            //     }
            //     else
            //     {
            //         Cal_Fan_R_percent = 0;
            //         Cal_Fan_L_percent = F_Roll;
            //     }
            // }
            // 风机MPC策略
            if (robot_state == 4)
            {
                Eigen::Matrix<double,3,1> u;
                // double F_thta_temp = 0;
                // // double F_f1, F_b1, F_x, F_f2, F_b2, F_f, F_b, F_thta;
                // // u = fan_mpc(0, 0.5, 2*toRad, 0, 0);
                // u = fan_mpc(position_x, velocity_x, imu_euler_pitch*toRad, imu_euler_pitch_v*toRad, 0);
                // F_f1 = u(0, 0);
                // F_b1 = u(1, 0);
                // F_x = u(2, 0);
                // if ((F_f1 + F_b1) == 0)
                // {
                //     F_f2 = 0;
                //     F_b2 = 0;
                // }
                // else
                // {
                //     F_f2 = F_f1/(F_f1 + F_b1)*F_x;
                //     F_b2 = F_b1/(F_f1 + F_b1)*F_x;
                //     F_f = sqrt(F_f1*F_f1 + F_f2*F_f2);
                //     F_b = sqrt(F_b1*F_b1 + F_b2*F_b2);
                //     F_thta_temp =  atan2(F_f2,F_f1)*toDeg + imu_euler_pitch;
                // }
                // F_thta = 90 - limit(F_thta_temp, -40, 40);

                // F_f = limit(F_f, 0, 50);
                // F_b = limit(F_b, 0, 50);
                // Cal_Fan_F_percent = F_f*2;// 转化为百分比风力
                // Cal_Fan_B_percent = F_b*2;// 转化为百分比风力
                // Cal_servo_degree = F_thta*100;
                // qDebug() << "F_f: " << F_f;
                // qDebug() << "F_b: " << F_b;
                // qDebug() << "F_thta: " << F_thta;
                // qDebug() << "F_f1: " << F_f1;
                // qDebug() << "F_b1: " << F_b1;
                // qDebug() << "F_x: " << F_x;

            }
        }






    }





    // if (jump_flag == 1)
    // {
    //     jump_time_count = jump_time_count + 0.002;
    //     tor_servo = ui->lineEdit_tor_jump->text().toFloat();
    //     Cal_R_Ankle_extra_torque = tor_servo*100;
    //     Cal_L_Ankle_extra_torque = tor_servo*100;
    //     if (jump_time_count > 0.2)
    //     {
    //         Cal_R_Ankle_extra_torque = 0;
    //         Cal_L_Ankle_extra_torque = 0;
    //         jump_flag = 0;
    //         jump_time_count = 0;
    //     }

    // }




    fan_text_count = fan_text_count + 0.002;
    if (fan_text_flag == 1)
    {
        if(fan_text_count>0)
        {
            Cal_Fan_F_percent = 10;
            Cal_Fan_B_percent = 0;
        }
        if(fan_text_count>1)
        {
            Cal_Fan_F_percent = 0;
            Cal_Fan_B_percent = 10;
        }

        if(fan_text_count>2.1)
        {
            Cal_Fan_B_percent = 0;
            Cal_Fan_L_percent = 10;
        }
        if(fan_text_count>3.1)
        {
            Cal_Fan_L_percent = 0;
            Cal_Fan_R_percent = 10;
        }
        if(fan_text_count>4.1)
        {
            Cal_Fan_L_percent = 0;
            Cal_Fan_R_percent = 0;
            Cal_Fan_F_percent = 0;
            Cal_Fan_B_percent = 0;
            fan_text_flag = 0;
        }
    }

}

void Widget::on_pushButton_fan_text_2_clicked()
{
    fly_flag = 1;
    excel_time_count = 0.0;
    RecordFlag = 1;
    fTimer_excel->start();
    // JET_HR2.start();

    QDateTime current_date_time = QDateTime::currentDateTime();
    excel_time = current_date_time.toString("yyyy_MM_dd_hh_mm_ss");
}


void Widget::on_pushButton_fan_text_3_clicked()
{
    rectify_imu_flag = 1;
    rectify_imu_count = 0;

    rectify_motive_flag = 0;
    rectify_motive_count = 0;

    position_ref_x = 0;
    position_ref_y = 0;
    position_ref_z = 0;

    qx_ref = 0;
    qy_ref = 0;
    qz_ref = 0;
    qw_ref = 0;

    //buff reset
    on_Buffer_data_reset_clicked();
}

void Widget::on_Buffer_data_reset_clicked()
{
    //motive data
    buf_motive_time.clear();

    buf_x.clear();
    buf_y.clear();
    buf_z.clear();
    buf_dx.clear();
    buf_dy.clear();
    buf_dz.clear();
    buf_ddx.clear();
    buf_ddy.clear();
    buf_ddz.clear();
    buf_motive.clear();
    buf_target_x_pos.clear();
    buf_target_y_pos.clear();
    buf_target_z_pos.clear();
    buf_target_x_vel.clear();
    buf_target_y_vel.clear();
    buf_target_z_vel.clear();

    buf_motive_time.reserve(1000); //设置容器大小，预留至少能够存储1000个元素的空间
    buf_x.reserve(1000);
    buf_y.reserve(1000);
    buf_z.reserve(1000);
    buf_dx.reserve(1000);
    buf_dy.reserve(1000);
    buf_dz.reserve(1000);
    buf_ddx.reserve(1000);
    buf_ddy.reserve(1000);
    buf_ddz.reserve(1000);
    buf_target_x_pos.reserve(1000);
    buf_target_y_pos.reserve(1000);
    buf_target_z_pos.reserve(1000);
    buf_target_x_vel.reserve(1000);
    buf_target_y_vel.reserve(1000);
    buf_target_z_vel.reserve(1000);
    buf_motive.reserve(1000);

    excel_time_count = 0.0;
}


void Widget::on_pushButton_param_set_clicked()
{
    kp_servo = ui->lineEdit_Kp_servo->text().toFloat();
    kd_servo = ui->lineEdit_Kd_servo->text().toFloat();
    ki_servo = ui->lineEdit_Ki_servo->text().toFloat();
    base_fan = ui->lineEdit_base_fan->text().toFloat();

    kp_fan = ui->lineEdit_Kp_fan->text().toFloat();
    kd_fan = ui->lineEdit_Kd_fan->text().toFloat();
    ki_fan = ui->lineEdit_Ki_fan->text().toFloat();
}


void Widget::on_pushButton_jump_clicked()
{
    jump_flag = 1;
}

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////        other_function        //////////////
////////////////////////////////////////////////////////////////////////////////////

void save_motive_data()
{
    buf_motive_time.append(excel_time_count);
    buf_x.append(position_x);
    buf_y.append(position_y);
    buf_z.append(position_z);

    buf_dx.append(velocity_x);
    buf_dy.append(velocity_y);
    buf_dz.append(velocity_z);

    buf_ddx.append(acceleration_x);
    buf_ddy.append(acceleration_y);
    buf_ddz.append(acceleration_z);

    buf_target_x_pos.append(position_x_desire);
    buf_target_y_pos.append(position_y_desire);
    buf_target_z_pos.append(position_z_desire);
    buf_target_x_vel.append(velocity_x_desire);
    buf_target_y_vel.append(velocity_y_desire);
    buf_target_z_vel.append(velocity_z_desire);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void save_upping_data()
{
    buf_upping_time.append(excel_time_count);
    buf_imu_roll.append(imu_euler_roll);
    buf_imu_pitch.append(imu_euler_pitch);
    buf_imu_yaw.append(imu_euler_yaw);
    buf_imu_rollV.append(imu_euler_roll_v);
    buf_imu_pitchV.append(imu_euler_pitch_v);
    buf_imu_euler_pitch_v_new.append(imu_euler_pitch_v_new);
    buf_imu_yawV.append(imu_euler_yaw_v);

    // buf_imu_rollV_raw.append(imu_euler_roll_v_raw);
    // buf_imu_pitchV_raw.append(imu_euler_pitch_v_raw);
    // buf_imu_yawV_raw.append(imu_euler_yaw_v_raw);

    // buf_imu_phi_desire.append(attitude_phi_desire);
    // buf_imu_xita_desire.append(attitude_xita_desire);
    // buf_imu_psid_desire.append(attitude_psid_desire);
    // buf_imu_phi_vel_desire.append(attitude_phi_v_desire);
    // buf_imu_xita_vel_desire.append(attitude_xita_v_desire);
    // buf_imu_psid_vel_desire.append(attitude_psid_v_desire);

    buf_L_Hipx_degree.append(L_Hipx_degree);
    buf_L_Hipz_degree.append(L_Hipz_degree);
    buf_L_Hipy_degree.append(L_Hipy_degree);
    buf_L_Knee_degree.append(L_Knee_degree);
    buf_L_Ankle_degree.append(L_Ankle_degree);

    buf_R_Hipx_degree.append(R_Hipx_degree);
    buf_R_Hipz_degree.append(R_Hipz_degree);
    buf_R_Hipy_degree.append(R_Hipy_degree);
    buf_R_Knee_degree.append(R_Knee_degree);
    buf_R_Ankle_degree.append(R_Ankle_degree);

    buf_L_Hipx_velocity.append(L_Hipx_velocity);
    buf_L_Hipz_velocity.append(L_Hipz_velocity);
    buf_L_Hipy_velocity.append(L_Hipy_velocity);
    buf_L_Knee_velocity.append(L_Knee_velocity);
    buf_L_Ankle_velocity.append(L_Ankle_velocity);

    buf_R_Hipx_velocity.append(R_Hipx_velocity);
    buf_R_Hipz_velocity.append(R_Hipz_velocity);
    buf_R_Hipy_velocity.append(R_Hipy_velocity);
    buf_R_Knee_velocity.append(R_Knee_velocity);
    buf_R_Ankle_velocity.append(R_Ankle_velocity);

    buf_L_Hipx_current.append(L_Hipx_current);
    buf_L_Hipz_current.append(L_Hipz_current);
    buf_L_Hipy_current.append(L_Hipy_current);
    buf_L_Knee_current.append(L_Knee_current);
    buf_L_Ankle_current.append(L_Ankle_current);

    buf_R_Hipx_current.append(R_Hipx_current);
    buf_R_Hipz_current.append(R_Hipz_current);
    buf_R_Hipy_current.append(R_Hipy_current);
    buf_R_Knee_current.append(R_Knee_current);
    buf_R_Ankle_current.append(R_Ankle_current);
    // filter_used_time.append(str_filter_upping_time);


}

////////////////////////////////////////////////////////////////////////////////////////////////////

void save_downing_data()
{
    buf_downing_time.append(excel_time_count);
    buf_fan_flag.append(fan_flag);
    buf_Fan_F_percent.append(Cal_Fan_F_percent);
    buf_Fan_B_percent.append(Cal_Fan_B_percent);
    buf_Fan_L_percent.append(Cal_Fan_L_percent);
    buf_Fan_R_percent.append(Cal_Fan_R_percent);
    buf_Cal_R_Hipx_degree.append(Cal_R_Hipx_degree*0.01);
    buf_Cal_R_Hipz_degree.append(Cal_R_Hipz_degree*0.01);
    buf_Cal_R_Hipy_degree.append(Cal_R_Hipy_degree*0.01);
    buf_Cal_R_Knee_degree.append(Cal_R_Knee_degree*0.01);
    buf_Cal_R_Ankle_degree.append(Cal_R_Ankle_degree*0.01);
    buf_Cal_L_Hipx_degree.append(Cal_L_Hipx_degree*0.01);
    buf_Cal_L_Hipz_degree.append(Cal_L_Hipz_degree*0.01);
    buf_Cal_L_Hipy_degree.append(Cal_L_Hipy_degree*0.01);
    buf_Cal_L_Knee_degree.append(Cal_L_Knee_degree*0.01);
    buf_Cal_L_Ankle_degree.append(Cal_L_Ankle_degree*0.01);

    buf_robot_state_R.append(robot_state_R);
    buf_robot_state_L.append(robot_state_L);
    buf_robot_state.append(robot_state);

    buf_servo_degree.append(Cal_servo_degree*0.01);
}


void Widget::on_timer_excel_timeout()
{
    if(RecordFlag == 1)
    {
        save_motive_data();
        save_upping_data();
        save_downing_data();
    }
    excel_time_count = excel_time_count + 0.01;
}

void Widget::on_pushButton_ToExcle_clicked()
{
    MotiveDataToExcel();
    UppingDataToExcel();
    DowningDataToExcel();
}

void Widget::MotiveDataToExcel( )
{
    QXlsx::Document xlsx;

    int length_row = buf_x.size();

    buf_motive.push_back(buf_motive_time);
    buf_motive.push_back(buf_x);
    buf_motive.push_back(buf_y);
    buf_motive.push_back(buf_z);
    buf_motive.push_back(buf_dx);
    buf_motive.push_back(buf_dy);
    buf_motive.push_back(buf_dz);
    buf_motive.push_back(buf_ddx);
    buf_motive.push_back(buf_ddy);
    buf_motive.push_back(buf_ddz);
    buf_motive.push_back(buf_target_x_pos);
    buf_motive.push_back(buf_target_y_pos);
    buf_motive.push_back(buf_target_z_pos);
    buf_motive.push_back(buf_target_x_vel);
    buf_motive.push_back(buf_target_y_vel);
    buf_motive.push_back(buf_target_z_vel);
    int length_motive = buf_motive.size();

    for (int j=0;j<length_motive;j++)
    {
        for (int i=0;i<length_row;i++)
        {
            xlsx.write(i+1,j+1,buf_motive[j][i]);
        }
    }
    xlsx.write(1,1,"motive_time_count");
    xlsx.write(1,2,"position_x");
    xlsx.write(1,3,"position_y");
    xlsx.write(1,4,"position_z");
    xlsx.write(1,5,"velocity_x");
    xlsx.write(1,6,"velocity_y");
    xlsx.write(1,7,"velocity_z");
    xlsx.write(1,8,"motive_roll");
    xlsx.write(1,9,"motive_pitch");
    xlsx.write(1,10,"motive_yaw");
    xlsx.write(1,11,"target_x_pos");
    xlsx.write(1,12,"target_y_pos");
    xlsx.write(1,13,"target_z_pos");
    xlsx.write(1,14,"target_x_vel");
    xlsx.write(1,15,"target_y_vel");
    xlsx.write(1,16,"target_z_vel");
    QString file_path_head = "C:/Users/19463/Desktop/datatrj/motive_data_";
    QString file_path_tail = ".xlsx";
    //        QString file_path = file_path_head + current_date + file_path_tail;
    QString file_path = file_path_head + excel_time + file_path_tail;
    xlsx.saveAs(file_path);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Widget::UppingDataToExcel( )
{
    QXlsx::Document xlsx;

    int length_row = buf_imu_roll.size();

    buf_upping.push_back(buf_upping_time);
    buf_upping.push_back(buf_imu_roll);
    buf_upping.push_back(buf_imu_pitch);
    buf_upping.push_back(buf_imu_yaw);
    buf_upping.push_back(buf_imu_rollV);
    buf_upping.push_back(buf_imu_pitchV);
    buf_upping.push_back(buf_imu_euler_pitch_v_new);

    buf_upping.push_back(buf_imu_yawV);

    // buf_upping.push_back(buf_imu_phi_desire);
    // buf_upping.push_back(buf_imu_xita_desire);
    // buf_upping.push_back(buf_imu_psid_desire);
    // buf_upping.push_back(buf_imu_phi_vel_desire);
    // buf_upping.push_back(buf_imu_xita_vel_desire);
    // buf_upping.push_back(buf_imu_psid_vel_desire);

    buf_upping.push_back(buf_L_Hipx_degree);
    buf_upping.push_back(buf_L_Hipz_degree);
    buf_upping.push_back(buf_L_Hipy_degree);
    buf_upping.push_back(buf_L_Knee_degree);
    buf_upping.push_back(buf_L_Ankle_degree);
    buf_upping.push_back(buf_R_Hipx_degree);
    buf_upping.push_back(buf_R_Hipz_degree);
    buf_upping.push_back(buf_R_Hipy_degree);
    buf_upping.push_back(buf_R_Knee_degree);
    buf_upping.push_back(buf_R_Ankle_degree);

    buf_upping.push_back(buf_L_Hipx_velocity);
    buf_upping.push_back(buf_L_Hipz_velocity);
    buf_upping.push_back(buf_L_Hipy_velocity);
    buf_upping.push_back(buf_L_Knee_velocity);
    buf_upping.push_back(buf_L_Ankle_velocity);
    buf_upping.push_back(buf_R_Hipx_velocity);
    buf_upping.push_back(buf_R_Hipz_velocity);
    buf_upping.push_back(buf_R_Hipy_velocity);
    buf_upping.push_back(buf_R_Knee_velocity);
    buf_upping.push_back(buf_R_Ankle_velocity);

    buf_upping.push_back(buf_L_Hipx_current);
    buf_upping.push_back(buf_L_Hipz_current);
    buf_upping.push_back(buf_L_Hipy_current);
    buf_upping.push_back(buf_L_Knee_current);
    buf_upping.push_back(buf_L_Ankle_current);
    buf_upping.push_back(buf_R_Hipx_current);
    buf_upping.push_back(buf_R_Hipz_current);
    buf_upping.push_back(buf_R_Hipy_current);
    buf_upping.push_back(buf_R_Knee_current);
    buf_upping.push_back(buf_R_Ankle_current);


    // buf_upping.push_back(buf_imu_rollV_raw);
    // buf_upping.push_back(buf_imu_pitchV_raw);
    // buf_upping.push_back(buf_imu_yawV_raw);

    // buf_upping.push_back(filter_used_time);

    int length_motive = buf_upping.size();

    for (int j=0;j<length_motive;j++)
    {
        for (int i=0;i<length_row;i++)
        {
            xlsx.write(i+1,j+1,buf_upping[j][i]);
        }
    }
    xlsx.write(1,1,"upping_time");
    xlsx.write(1,2,"imu_roll");
    xlsx.write(1,3,"imu_pitch");
    xlsx.write(1,4,"imu_yaw");
    xlsx.write(1,5,"imu_rollV");
    xlsx.write(1,6,"imu_pitchV");
    xlsx.write(1,7,"imu_pitchV_new");
    xlsx.write(1,8,"imu_yawV");

    // xlsx.write(1,8,"imu_roll_desire");
    // xlsx.write(1,9,"imu_pitch_desire");
    // xlsx.write(1,10,"imu_yaw_desire");
    // xlsx.write(1,11,"imu_roll_vel_desire");
    // xlsx.write(1,12,"imu_pitch_vel_desire");
    // xlsx.write(1,13,"imu_yaw_vel_desire");

    xlsx.write(1,9,"L_Hipx_degree");
    xlsx.write(1,10,"L_Hipz_degree");
    xlsx.write(1,11,"L_Hipy_degree");
    xlsx.write(1,12,"L_Knee_degree");
    xlsx.write(1,13,"L_Ankle_degree");
    xlsx.write(1,14,"R_Hipx_degree");
    xlsx.write(1,15,"R_Hipz_degree");
    xlsx.write(1,16,"R_Hipy_degree");
    xlsx.write(1,17,"R_Knee_degree");
    xlsx.write(1,18,"R_Ankle_degree");

    xlsx.write(1,19,"L_Hipx_velocity");
    xlsx.write(1,20,"L_Hipz_velocity");
    xlsx.write(1,21,"L_Hipy_velocity");
    xlsx.write(1,22,"L_Knee_velocity");
    xlsx.write(1,23,"L_Ankle_velocity");
    xlsx.write(1,24,"R_Hipx_velocity");
    xlsx.write(1,25,"R_Hipz_velocity");
    xlsx.write(1,26,"R_Hipy_velocity");
    xlsx.write(1,27,"R_Knee_velocity");
    xlsx.write(1,28,"R_Ankle_velocity");

    xlsx.write(1,29,"L_Hipx_current");
    xlsx.write(1,30,"L_Hipz_current");
    xlsx.write(1,31,"L_Hipy_current");
    xlsx.write(1,32,"L_Knee_current");
    xlsx.write(1,33,"L_Ankle_current");
    xlsx.write(1,34,"R_Hipx_current");
    xlsx.write(1,35,"R_Hipz_current");
    xlsx.write(1,36,"R_Hipy_current");
    xlsx.write(1,37,"R_Knee_current");
    xlsx.write(1,38,"R_Ankle_current");

    // xlsx.write(1,44,"imu_rollV_raw");
    // xlsx.write(1,45,"imu_pitchV_raw");
    // xlsx.write(1,46,"imu_yawV_raw");

    // xlsx.write(1,47,"filter_used_time");


    QString file_path_head = "C:/Users/19463/Desktop/datatrj/upping_data_";
    QString file_path_tail = ".xlsx";
    //        QString file_path = file_path_head + current_date + file_path_tail;
    QString file_path = file_path_head + excel_time + file_path_tail;
    xlsx.saveAs(file_path);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Widget::DowningDataToExcel( )
{
    QXlsx::Document xlsx;

    int length_row = buf_Fan_F_percent.size();

    buf_downing.push_back(buf_downing_time);
    buf_downing.push_back(buf_fan_flag);
    buf_downing.push_back(buf_Fan_F_percent);
    buf_downing.push_back(buf_Fan_B_percent);
    buf_downing.push_back(buf_Fan_L_percent);
    buf_downing.push_back(buf_Fan_R_percent);
    buf_downing.push_back(buf_Cal_R_Hipx_degree);
    buf_downing.push_back(buf_Cal_R_Hipz_degree);
    buf_downing.push_back(buf_Cal_R_Hipy_degree);
    buf_downing.push_back(buf_Cal_R_Knee_degree);
    buf_downing.push_back(buf_Cal_R_Ankle_degree);
    buf_downing.push_back(buf_Cal_L_Hipx_degree);
    buf_downing.push_back(buf_Cal_L_Hipz_degree);
    buf_downing.push_back(buf_Cal_L_Hipy_degree);
    buf_downing.push_back(buf_Cal_L_Knee_degree);
    buf_downing.push_back(buf_Cal_L_Ankle_degree);

    buf_downing.push_back(buf_servo_degree);

    buf_downing.push_back(buf_robot_state_R);
    buf_downing.push_back(buf_robot_state_L);
    buf_downing.push_back(buf_robot_state);



    int length_column = buf_downing.size();

    for (int j=0;j<length_column;j++)
    {
        for (int i=0;i<length_row;i++)
        {
            xlsx.write(i+1,j+1,buf_downing[j][i]);
        }
    }
    xlsx.write(1,1,"downing_time_count");
    xlsx.write(1,2,"fan_flag");
    xlsx.write(1,3,"Cal_Fan_F_percent");
    xlsx.write(1,4,"Cal_Fan_B_percent");
    xlsx.write(1,5,"Cal_Fan_L_percent");
    xlsx.write(1,6,"Cal_Fan_R_percent");
    xlsx.write(1,7,"Cal_R_Hipx_degree" );
    xlsx.write(1,8,"Cal_R_Hipz_degree" );
    xlsx.write(1,9,"Cal_R_Hipy_degree" );
    xlsx.write(1,10,"Cal_R_Knee_degree" );
    xlsx.write(1,11,"Cal_R_Ankle_degree" );
    xlsx.write(1,12,"Cal_L_Hipx_degree" );
    xlsx.write(1,13,"Cal_L_Hipz_degree" );
    xlsx.write(1,14,"Cal_L_Hipy_degree" );
    xlsx.write(1,15,"Cal_L_Knee_degree" );
    xlsx.write(1,16,"Cal_L_Ankle_degree" );
    xlsx.write(1,17,"servo_degree" );

    xlsx.write(1,18,"robot_state_R" );
    xlsx.write(1,19,"robot_state_L" );
    xlsx.write(1,20,"robot_state" );

    QString file_path_head = "C:/Users/19463/Desktop/datatrj/downing_data_";
    QString file_path_tail = ".xlsx";
    //        QString file_path = file_path_head + current_date + file_path_tail;
    QString file_path = file_path_head + excel_time + file_path_tail;
    xlsx.saveAs(file_path);
}

///////////////////////////////////////////////////////////////////////////////////////////////////


void Widget::on_pushButton_zero_clicked()
{
    zero_flag = 1;
    // Sleep(10);



}



void Widget::on_pushButton_Default_clicked()
{
    ui->lineEdit_Kp_servo->setText("2.5");
    ui->lineEdit_Kd_servo->setText("0.5");
    ui->lineEdit_Ki_servo->setText("0");
    ui->lineEdit_base_fan->setText("100");

    ui->lineEdit_Kp_fan->setText("0");
    ui->lineEdit_Kd_fan->setText("0");
    ui->lineEdit_Ki_fan->setText("0");
}


void Widget::on_pushButton_Contact_clicked()
{
    Contact_test_flag = 1;

}


void Widget::on_pushButton_Default_MPC_clicked()
{
    ui->lineEdit_Qi_x->setText("0");
    ui->lineEdit_Qi_dx->setText("1");
    ui->lineEdit_Qi_thta->setText("50");
    ui->lineEdit_Qi_dthta->setText("1");

    ui->lineEdit_Ri_f->setText("0.001");

}


void Widget::on_pushButton_param_set_MPC_clicked()
{
    Qi_x = ui->lineEdit_Qi_x->text().toFloat();
    Qi_dx = ui->lineEdit_Qi_dx->text().toFloat();
    Qi_thta = ui->lineEdit_Qi_thta->text().toFloat();
    Qi_dthta = ui->lineEdit_Qi_dthta->text().toFloat();

    Ri_f = ui->lineEdit_Ri_f->text().toFloat();
}

