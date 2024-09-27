#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QUdpSocket>
#include <QDebug>
#include <QFile>
#include <QTimer>
#include <QDateTime>
#include <QElapsedTimer>
#include <QMainWindow>
#include <QDebug>
#include <QKeyEvent>
#include "utilities.h"
#include "type.h"
#include "communicationprotocol.h"
#include "data_init.h"
#include "qmythread.h"
#include "filter.h"
#include "fan_MPC.h"
//多线程
#include <inttypes.h>
#include "conio.h"
#include <stdio.h>
#include <iostream>
#include <QDialog>
#include "deal_msg.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

int sign(int x);

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

private slots:
    void on_timer_data_show_timeout();//下行定时器处理槽函数
    void on_timer_move_timeout();//下行定时器处理槽函数
    void on_timer_excel_timeout();//excel定时器处理槽函数
    void on_timer_timeout();//motive

    void on_pushButton_clicked();

    void on_timer_control_timeout();

    void on_pushButton_Cal_clicked();

    void on_pushButton_stand_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_stop_clicked();

    void on_pushButton_data_start_clicked();


    void on_pushButton_text_servo_clicked();

    void on_pushButton_text_servo_2_clicked();

    void on_pushButton_fan_text_clicked();

    void on_pushButton_fan_text_2_clicked();

    void on_pushButton_fan_text_3_clicked();

    void on_Buffer_data_reset_clicked();

    void on_pushButton_param_set_clicked();

    void on_pushButton_jump_clicked();

    void on_pushButton_ToExcle_clicked();

    void MotiveDataToExcel();

    void UppingDataToExcel();

    void DowningDataToExcel();

    void on_pushButton_zero_clicked();


    void on_pushButton_Default_clicked();


    void on_pushButton_Contact_clicked();

    void on_pushButton_Default_MPC_clicked();

    void on_pushButton_param_set_MPC_clicked();

private:
    Ui::Widget *ui;
    deal_msg deal;
    QTimer *fTimer_excel;// Excel数据定时器
    QTimer *fTimer_data_show;//ui更新
    QTimer *fTimer_move;//电机五次多项式规划
    QTimer *fTimer_control;
    QTimer *fTimer;//motive 定时器
    QElapsedTimer fTimeCounter;//motive计时器
    QElapsedTimer fdisplaycounter;//显示数据定时器
    QThreadDAQ motive;//motive线程 biaoji
    int RecordFlag = 0;
    fan_mpc JET_HR2;
};
#endif // WIDGET_H
