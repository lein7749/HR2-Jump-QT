#ifndef FAN_MPC_H
#define FAN_MPC_H

#include "QObject"
#include "QThread"
#include <Eigen/Dense>
#include <QTimer>
#include <QDateTime>
#include <Eigen/Dense>
#include <qpOASES.hpp>

class fan_mpc:  public QThread
{

    Q_OBJECT
public:

    fan_mpc();
    bool m_stop = false;
    // MatrixXd matrixPower(const MatrixXd &A, int power);

    Eigen::Matrix<double,3,1> attitude_vx_mpc(float x_pos,float x_vel,float phi,float phi_vel,float exp_spd);


    // Eigen::Matrix<double, STATE_NUM, PREDICT_WINDOW + 1> mpc_ref;






protected:
    void    run() Q_DECL_OVERRIDE;
    QElapsedTimer fTimeCounter_mpc;


};








#endif // FAN_MPC_H
