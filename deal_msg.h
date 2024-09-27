
#ifndef DEAL_MSG_H
#define DEAL_MSG_H

#include "QObject"
#include "QThread"
#include "QUdpSocket"
#include "utilities.h"
#include "type.h"
#include "param_init.h"
#include "data_init.h"
#include <QTimer>
#include <QDateTime>
#include <QElapsedTimer>

class deal_msg: public QThread
{

    Q_OBJECT
public:
    deal_msg();
    QUdpSocket *socket;

    QTimer *fTimer_down;//下行定时器

    void getmsg();
    void nomsg();

    void stopmsg();

    QString up_time();
protected:
    void    run() Q_DECL_OVERRIDE;

private:
    bool    m_stop=false; //停止线程
    QElapsedTimer fTimeCounter_up;
    QElapsedTimer fTimeCounter_down;

private slots:
    void on_timer_down_timeout();//下行定时器处理槽函数
};

int data_deal1(QString str);


#endif // DEAL_MSG_H
