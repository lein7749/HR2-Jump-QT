#include <qpOASES.hpp>
#include <fan_MPC.h>
#include <Eigen/Dense>
#include <math.h>
#include <QDebug>
#include <param_init.h>
#include "filter.h"

using namespace Eigen;
using namespace std;
using namespace qpOASES;

#define STATE_NUM   4//状态量
#define INPUT_NUM   3//控制量
#define PREDICT_WINDOW  5//预测步数
#define T_MPC 0.05//离散时间

#define toDeg (180/Pi)
#define toRad (Pi/180)
#define E 2.71828182845904523536029
#define Pi 3.14159265358979323846264
#define Degree 0.01745329251994329576924

#define J (0.5)
#define l (0.15)
#define M 17

fan_mpc::fan_mpc() {}

MatrixXd matrixPower(const MatrixXd &A, int power);


Eigen::Matrix<double,3,1> fan_mpc::attitude_vx_mpc(float x_pos,float x_vel,
                     float phi,float phi_vel,float exp_spd)
{
    Eigen::Matrix<double, STATE_NUM, STATE_NUM> A;//状态方程，离散化之后
    Eigen::Matrix<double, STATE_NUM, INPUT_NUM> B;
    Eigen::Matrix<double, STATE_NUM, STATE_NUM> A_;//状态方程，离散化之后
    Eigen::Matrix<double, STATE_NUM, INPUT_NUM> B_;

    // qDebug() << "B: " << B(0,0);
    Eigen::DiagonalMatrix<double, STATE_NUM> Qi;
    Eigen::DiagonalMatrix<double, INPUT_NUM> Ri;

    Eigen::Matrix<double, STATE_NUM, 1> x0;

    x0 << x_pos,x_vel,phi,phi_vel;//MPC状态量


    // B <<0,T_MPC,0;
    B_ << 0,                      0,    0,
          0,                      0,  1/M,
          0,                      0,    0,
          l*cos(phi)/J, -l*cos(phi)/J,  0;

    B << 0,                                    0,        0,
         0,                                    0,  T_MPC/M,
         0,                                    0,        0,
         T_MPC*l*cos(phi)/J, -l*T_MPC*cos(phi)/J,        0;


    A_ << 0, 1, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 1,
          0, 0, 0, 0;

    A << 1, T_MPC, 0,     0,
         0,     1, 0,     0,
         0,     0, 1, T_MPC,
         0,     0, 0,     1;
    // A << 1,T_MPC,0,
    //     0,0,-9.8*T_MPC,
    //     0,0,1;
    // Qi_x = 0;
    // Qi_dx = 10;
    // Qi_thta = 20000;
    // Qi_dthta = 0;

    Qi.diagonal() << Qi_x, Qi_dx, Qi_thta, Qi_dthta;//状态量权重

    // Ri_f = 0.001;

    Ri.diagonal() << Ri_f, Ri_f, Ri_f;//控制量权重

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(STATE_NUM*PREDICT_WINDOW, STATE_NUM*PREDICT_WINDOW);

    for(int i = 0; i < PREDICT_WINDOW ; i++)
    {
        for(int j = 0; j < STATE_NUM; j++)
        {
            double value = Qi.diagonal()[j];
            if(value != 0)
            {
                Q(i*STATE_NUM + j, i*STATE_NUM + j) = value;
            }
        }
    }

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(INPUT_NUM*PREDICT_WINDOW, INPUT_NUM*PREDICT_WINDOW);

    for(int i = 0; i < PREDICT_WINDOW ; i++)
    {
        for(int j = 0; j < INPUT_NUM; j++)
        {
            double value = Ri.diagonal()[j];
            if(value != 0)
            {
                R(i*INPUT_NUM + j, i*INPUT_NUM + j) = value;
            }
        }
    }


    Eigen::MatrixXd xRef = Eigen::MatrixXd::Zero(STATE_NUM*PREDICT_WINDOW, 1);//参考值


    for(int i = 0; i<STATE_NUM*PREDICT_WINDOW ; i= i +STATE_NUM)
    {
        xRef(i+0,0) = 0;
        xRef(i+1,0) = exp_spd;
        xRef(i+2,0) = 0;
        xRef(i+3,0) = -10*toRad;
    }

    Eigen::MatrixXd psi = Eigen::MatrixXd::Zero(STATE_NUM*PREDICT_WINDOW, STATE_NUM);

    for(int i = 0 ; i < PREDICT_WINDOW ; i++)
    {
        psi.block(STATE_NUM*i,0,STATE_NUM,STATE_NUM) = matrixPower(A,i+1);

    }


    Eigen::MatrixXd tha = Eigen::MatrixXd::Zero(STATE_NUM*PREDICT_WINDOW, INPUT_NUM*PREDICT_WINDOW);

    for(int i = 1; i < PREDICT_WINDOW+1 ; i++)
    {
        for(int j = 0 ; j < i ; j++)
        {
            tha.block((i-1)*STATE_NUM,j*INPUT_NUM,STATE_NUM,INPUT_NUM) = matrixPower(A,i-1-j)*B;
        }
    }

    Eigen::MatrixXd E2 = Eigen::MatrixXd::Zero(STATE_NUM*PREDICT_WINDOW, 1);

    E2 = psi*x0 - xRef ;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(INPUT_NUM*PREDICT_WINDOW, INPUT_NUM*PREDICT_WINDOW);

    H = 2*(tha.transpose()*Q*tha+R);

    Eigen::MatrixXd f = Eigen::MatrixXd::Zero(INPUT_NUM*PREDICT_WINDOW, 1);

    f = (2.0*E2.transpose()*Q*tha).transpose();

    Eigen::MatrixXd g = Eigen::MatrixXd::Zero(1,INPUT_NUM*PREDICT_WINDOW);

    g = f.transpose();

    ////////////////////////////////////////////////////////////////////////////

    /* Setup data of first QP. */
    real_t H_matrix[INPUT_NUM*PREDICT_WINDOW*INPUT_NUM*PREDICT_WINDOW];
    real_t g_matrix[INPUT_NUM*PREDICT_WINDOW];
    real_t lb_matrix[INPUT_NUM*PREDICT_WINDOW];
    real_t ub_matrix[INPUT_NUM*PREDICT_WINDOW];

    // /* Setup data of second QP. */
    // real_t g_new[2] = { 1.0, 1.5 };
    // real_t lb_new[2] = { 0.0, -1.0 };
    // real_t ub_new[2] = { 5.0, -0.5 };
    //转为qpOASES格式
    for(int i =0;i<H.rows();i++)
    {
        for(int j=0;j<H.cols();j++)
        {
            H_matrix[(i)*H.cols()+j] = H(i,j);
        }
        g_matrix[i] = g(0,i);
    }

    Eigen::Matrix<double, INPUT_NUM, 1> u_max;//控制量约束
    Eigen::Matrix<double, INPUT_NUM, 1> u_min;



    u_max <<  50, 50, 20;

    u_min <<  10, 10, -20;



    for(int i=0;i<INPUT_NUM*PREDICT_WINDOW;i=i+INPUT_NUM)
    {
        for(int j = 0; j < INPUT_NUM; j++)
        {
            lb_matrix[i+j] = u_min(j,0);
            ub_matrix[i+j] = u_max(j,0);
            // cout << "lb_matrix" << i+j << endl << ub_matrix[i+j] << endl;
        }

    }


    /* Setting up QProblemB object. */
    QProblemB example( INPUT_NUM*PREDICT_WINDOW );

    Options options;
    //options.enableFlippingBounds = BT_FALSE;
    options.printLevel = PL_NONE;
    options.initialStatusBounds = ST_INACTIVE;
    options.numRefinementSteps = 1;
    options.enableCholeskyRefactorisation = 1;
    example.setOptions( options );


    /* Solve first QP. */
    int_t nWSR = 50;
    example.init( H_matrix,g_matrix,lb_matrix,ub_matrix, nWSR,0 );

    /* Get and print solution of first QP. */
    real_t xOpt[INPUT_NUM*PREDICT_WINDOW];
    example.getPrimalSolution( xOpt );

    // cout << "ctr" << ctr <<endl;
    /* Solve second QP. */
    nWSR = 10;
    example.hotstart( g_matrix,lb_matrix,ub_matrix, nWSR,0 );
    // 	printf( "\nnWSR = %d\n\n", nWSR );

    /* Get and print solution of second QP. */
    example.getPrimalSolution( xOpt );
    // printf( "\nxOpt = [ %e, %e ];  objVal = %e\n\n", xOpt[0],xOpt[1],example.getObjVal() );

    Eigen::Matrix<double,INPUT_NUM,1> ctr;
    for(int i = 0;i<INPUT_NUM;i++)
    {
        ctr(i,0) = xOpt[i];//当前时刻控制量

    }
    return ctr;
    // std::cout << "controll_force_degree" << ctr <<  std::endl;


}



MatrixXd matrixPower(const MatrixXd &A, int power) {
    int rows = A.rows();
    int cols = A.cols();

    if (power == 0) {
        return MatrixXd::Identity(rows, cols);
    }
    MatrixXd result = MatrixXd::Identity(rows, cols);
    MatrixXd base = A;

    while (power > 0) {
        if (power % 2 == 1) {
            result *= base;
        }
        base *= base;
        power /= 2;
    }

    return result;
}


void fan_mpc::run()
{
    while(!m_stop)//循环主体
    {

        int tmMsec_up = fTimeCounter_mpc.elapsed();
        int ms = tmMsec_up%1000;

        str_mpc_time = QString::asprintf(" %d milesecond",ms);
        fTimeCounter_mpc.start();

        Eigen::Matrix<double,3,1> u;
        double F_thta_temp = 0;
        // double F_f1, F_b1, F_x, F_f2, F_b2, F_f, F_b, F_thta;
        // u = fan_mpc(0, 0.5, 2*toRad, 0, 0);
        u = attitude_vx_mpc(position_x, velocity_x, imu_euler_pitch*toRad, imu_euler_pitch_v*toRad, 0);
        F_f1 = u(0, 0);
        F_b1 = u(1, 0);
        F_x = u(2, 0);
        if ((F_f1 + F_b1) == 0)
        {
            F_f2 = 0;
            F_b2 = 0;
        }
        else
        {
            F_f2 = F_f1/(F_f1 + F_b1)*F_x;
            F_b2 = F_b1/(F_f1 + F_b1)*F_x;
            F_f = sqrt(F_f1*F_f1 + F_f2*F_f2);
            F_b = sqrt(F_b1*F_b1 + F_b2*F_b2);
            F_thta_temp =  atan2(F_f2,F_f1)*toDeg + imu_euler_pitch;
        }
        F_thta = 90 - limit(F_thta_temp, -40, 40);

        F_f = limit(F_f, 0, 50);
        F_b = limit(F_b, 0, 50);
        Cal_Fan_F_percent = F_f*2;// 转化为百分比风力
        Cal_Fan_B_percent = F_b*2;// 转化为百分比风力
        Cal_servo_degree = F_thta*100;
        // qDebug() << "F_f: " << F_f;
        // qDebug() << "F_b: " << F_b;
        // qDebug() << "F_thta: " << F_thta;
        // qDebug() << "F_f1: " << F_f1;
        // qDebug() << "F_b1: " << F_b1;
        // qDebug() << "F_x: " << F_x;
        // qDebug() << "str_mpc_time: " << str_mpc_time;

    }

    quit();

}














