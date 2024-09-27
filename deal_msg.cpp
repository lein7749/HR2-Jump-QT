#include "deal_msg.h"
#include <iostream>
#include "param_init.h"
#include "filter.h"


deal_msg::deal_msg()
{
    socket = new QUdpSocket(this);
    socket->bind(8089);
    connect(socket,&QUdpSocket::readyRead,this,&deal_msg::getmsg);
    connect(socket,&QUdpSocket::disconnected,this,&deal_msg::nomsg);

    fTimer_down=new QTimer(this);
    //    fTimer_down->stop();
    fTimer_down->setInterval(2);
    connect(fTimer_down,SIGNAL(timeout()),this,SLOT(on_timer_down_timeout()));
    fTimer_down->start();          //Downing data timer
}

void deal_msg::stopmsg()
{
    m_stop=true;
}

QString deal_msg::up_time()
{
    return str_upping_time;
}

void deal_msg::on_timer_down_timeout()
{

    int tmMsec_up = fTimeCounter_down.elapsed();
    int ms = tmMsec_up%1000;


    str_downing_time = QString::asprintf(" %d milesecond",ms);
    fTimeCounter_down.start();

    Data_send.zero_flag = zero_flag;//HR2 rectify flag
    Data_send.fan_flag = fan_flag;//fan open flag

    Data_send.Cal_Fan_F_percent  = Cal_Fan_F_percent;    //Fan percent

    Data_send.Cal_Fan_B_percent = Cal_Fan_B_percent;
    Data_send.Cal_Fan_L_percent = Cal_Fan_L_percent;
    Data_send.Cal_Fan_R_percent  = Cal_Fan_R_percent;

    //Cal_R_Hipx_degree
    if(Cal_R_Hipx_degree>0)
    {
        Data_send.Cal_R_Hipx_degree_flag = 0;
    }
    else
    {
        Data_send.Cal_R_Hipx_degree_flag = 1;
    }
    Data_send.Cal_R_Hipx_degree_high = qAbs(Cal_R_Hipx_degree) / 256;
    Data_send.Cal_R_Hipx_degree_low  = qAbs(Cal_R_Hipx_degree) % 256;

    //Cal_R_Hipz_degree
    if(Cal_R_Hipz_degree>0)
    {
        Data_send.Cal_R_Hipz_degree_flag = 0;
    }
    else
    {
        Data_send.Cal_R_Hipz_degree_flag = 1;
    }
    Data_send.Cal_R_Hipz_degree_high = qAbs(Cal_R_Hipz_degree) / 256;
    Data_send.Cal_R_Hipz_degree_low  = qAbs(Cal_R_Hipz_degree) % 256;

    //Cal_R_Hipy_degree
    if(Cal_R_Hipy_degree>0)
    {
        Data_send.Cal_R_Hipy_degree_flag = 0;
    }
    else
    {
        Data_send.Cal_R_Hipy_degree_flag = 1;
    }
    Data_send.Cal_R_Hipy_degree_high = qAbs(Cal_R_Hipy_degree) / 256;
    Data_send.Cal_R_Hipy_degree_low  = qAbs(Cal_R_Hipy_degree) % 256;

    //Cal_R_Knee_degree
    if(Cal_R_Knee_degree>0)
    {
        Data_send.Cal_R_Knee_degree_flag = 0;
    }
    else
    {
        Data_send.Cal_R_Knee_degree_flag = 1;
    }
    Data_send.Cal_R_Knee_degree_high = qAbs(Cal_R_Knee_degree) / 256;
    Data_send.Cal_R_Knee_degree_low  = qAbs(Cal_R_Knee_degree) % 256;

    //Cal_R_Ankle_degree
    if(Cal_R_Ankle_degree>0)
    {
        Data_send.Cal_R_Ankle_degree_flag = 0;
    }
    else
    {
        Data_send.Cal_R_Ankle_degree_flag = 1;
    }
    Data_send.Cal_R_Ankle_degree_high = qAbs(Cal_R_Ankle_degree) / 256;
    Data_send.Cal_R_Ankle_degree_low  = qAbs(Cal_R_Ankle_degree) % 256;

    //Cal_L_Hipx_degree
    if(Cal_L_Hipx_degree>0)
    {
        Data_send.Cal_L_Hipx_degree_flag = 0;
    }
    else
    {
        Data_send.Cal_L_Hipx_degree_flag = 1;
    }
    Data_send.Cal_L_Hipx_degree_high = qAbs(Cal_L_Hipx_degree) / 256;
    Data_send.Cal_L_Hipx_degree_low  = qAbs(Cal_L_Hipx_degree) % 256;

    //Cal_L_Hipz_degree
    if(Cal_L_Hipz_degree>0)
    {
        Data_send.Cal_L_Hipz_degree_flag = 0;
    }
    else
    {
        Data_send.Cal_L_Hipz_degree_flag = 1;
    }
    Data_send.Cal_L_Hipz_degree_high = qAbs(Cal_L_Hipz_degree) / 256;
    Data_send.Cal_L_Hipz_degree_low  = qAbs(Cal_L_Hipz_degree) % 256;

    //Cal_L_Hipy_degree
    if(Cal_L_Hipy_degree>0)
    {
        Data_send.Cal_L_Hipy_degree_flag = 0;
    }
    else
    {
        Data_send.Cal_L_Hipy_degree_flag = 1;
    }
    Data_send.Cal_L_Hipy_degree_high = qAbs(Cal_L_Hipy_degree) / 256;
    Data_send.Cal_L_Hipy_degree_low  = qAbs(Cal_L_Hipy_degree) % 256;

    //Cal_L_Knee_degree
    if(Cal_L_Knee_degree>0)
    {
        Data_send.Cal_L_Knee_degree_flag = 0;
    }
    else
    {
        Data_send.Cal_L_Knee_degree_flag = 1;
    }
    Data_send.Cal_L_Knee_degree_high = qAbs(Cal_L_Knee_degree) / 256;
    Data_send.Cal_L_Knee_degree_low  = qAbs(Cal_L_Knee_degree) % 256;

    //Cal_L_Ankle_degree
    if(Cal_L_Ankle_degree>0)
    {
        Data_send.Cal_L_Ankle_degree_flag = 0;
    }
    else
    {
        Data_send.Cal_L_Ankle_degree_flag = 1;
    }
    Data_send.Cal_L_Ankle_degree_high = qAbs(Cal_L_Ankle_degree) / 256;
    Data_send.Cal_L_Ankle_degree_low  = qAbs(Cal_L_Ankle_degree) % 256;

    //Cal_R_Hipx_velocity
    if(Cal_R_Hipx_velocity>0)
    {
        Data_send.Cal_R_Hipx_velocity_flag = 0;
    }
    else
    {
        Data_send.Cal_R_Hipx_velocity_flag = 1;
    }
    Data_send.Cal_R_Hipx_velocity_high = qAbs(Cal_R_Hipx_velocity) / 256;
    Data_send.Cal_R_Hipx_velocity_low  = qAbs(Cal_R_Hipx_velocity) % 256;

    //Cal_R_Hipz_velocity
    if(Cal_R_Hipz_velocity>0)
    {
        Data_send.Cal_R_Hipz_velocity_flag = 0;
    }
    else
    {
        Data_send.Cal_R_Hipz_velocity_flag = 1;
    }
    Data_send.Cal_R_Hipz_velocity_high = qAbs(Cal_R_Hipz_velocity) / 256;
    Data_send.Cal_R_Hipz_velocity_low  = qAbs(Cal_R_Hipz_velocity) % 256;

    //Cal_R_Hipy_velocity
    if(Cal_R_Hipy_velocity>0)
    {
        Data_send.Cal_R_Hipy_velocity_flag = 0;
    }
    else
    {
        Data_send.Cal_R_Hipy_velocity_flag = 1;
    }
    Data_send.Cal_R_Hipy_velocity_high = qAbs(Cal_R_Hipy_velocity) / 256;
    Data_send.Cal_R_Hipy_velocity_low  = qAbs(Cal_R_Hipy_velocity) % 256;

    //Cal_R_Knee_velocity
    if(Cal_R_Knee_velocity>0)
    {
        Data_send.Cal_R_Knee_velocity_flag = 0;
    }
    else
    {
        Data_send.Cal_R_Knee_velocity_flag = 1;
    }
    Data_send.Cal_R_Knee_velocity_high = qAbs(Cal_R_Knee_velocity) / 256;
    Data_send.Cal_R_Knee_velocity_low  = qAbs(Cal_R_Knee_velocity) % 256;

    //Cal_R_Ankle_velocity
    if(Cal_R_Ankle_velocity>0)
    {
        Data_send.Cal_R_Ankle_velocity_flag = 0;
    }
    else
    {
        Data_send.Cal_R_Ankle_velocity_flag = 1;
    }
    Data_send.Cal_R_Ankle_velocity_high = qAbs(Cal_R_Ankle_velocity) / 256;
    Data_send.Cal_R_Ankle_velocity_low  = qAbs(Cal_R_Ankle_velocity) % 256;

    //Cal_L_Hipx_velocity
    if(Cal_L_Hipx_velocity>0)
    {
        Data_send.Cal_L_Hipx_velocity_flag = 0;
    }
    else
    {
        Data_send.Cal_L_Hipx_velocity_flag = 1;
    }
    Data_send.Cal_L_Hipx_velocity_high = qAbs(Cal_L_Hipx_velocity) / 256;
    Data_send.Cal_L_Hipx_velocity_low  = qAbs(Cal_L_Hipx_velocity) % 256;

    //Cal_L_Hipz_velocity
    if(Cal_L_Hipz_velocity>0)
    {
        Data_send.Cal_L_Hipz_velocity_flag = 0;
    }
    else
    {
        Data_send.Cal_L_Hipz_velocity_flag = 1;
    }
    Data_send.Cal_L_Hipz_velocity_high = qAbs(Cal_L_Hipz_velocity) / 256;
    Data_send.Cal_L_Hipz_velocity_low  = qAbs(Cal_L_Hipz_velocity) % 256;

    //Cal_L_Hipy_velocity
    if(Cal_L_Hipy_velocity>0)
    {
        Data_send.Cal_L_Hipy_velocity_flag = 0;
    }
    else
    {
        Data_send.Cal_L_Hipy_velocity_flag = 1;
    }
    Data_send.Cal_L_Hipy_velocity_high = qAbs(Cal_L_Hipy_velocity) / 256;
    Data_send.Cal_L_Hipy_velocity_low  = qAbs(Cal_L_Hipy_velocity) % 256;

    //Cal_L_Knee_velocity
    if(Cal_L_Knee_velocity>0)
    {
        Data_send.Cal_L_Knee_velocity_flag = 0;
    }
    else
    {
        Data_send.Cal_L_Knee_velocity_flag = 1;
    }
    Data_send.Cal_L_Knee_velocity_high = qAbs(Cal_L_Knee_velocity) / 256;
    Data_send.Cal_L_Knee_velocity_low  = qAbs(Cal_L_Knee_velocity) % 256;

    //Cal_L_Ankle_velocity
    if(Cal_L_Ankle_velocity>0)
    {
        Data_send.Cal_L_Ankle_velocity_flag = 0;
    }
    else
    {
        Data_send.Cal_L_Ankle_velocity_flag = 1;
    }
    Data_send.Cal_L_Ankle_velocity_high = qAbs(Cal_L_Ankle_velocity) / 256;
    Data_send.Cal_L_Ankle_velocity_low  = qAbs(Cal_L_Ankle_velocity) % 256;


    //L-kp//
    //Cal_L_Hipx_kp

    Data_send.Cal_L_Hipx_kp_high = qAbs(Cal_L_Hipx_kp) / 256;
    Data_send.Cal_L_Hipx_kp_low  = qAbs(Cal_L_Hipx_kp) % 256;

    //Cal_L_Hipz_kp

    Data_send.Cal_L_Hipz_kp_high = qAbs(Cal_L_Hipz_kp) / 256;
    Data_send.Cal_L_Hipz_kp_low  = qAbs(Cal_L_Hipz_kp) % 256;

    //Cal_L_Hipy_kp

    Data_send.Cal_L_Hipy_kp_high = qAbs(Cal_L_Hipy_kp) / 256;
    Data_send.Cal_L_Hipy_kp_low  = qAbs(Cal_L_Hipy_kp) % 256;

    //Cal_L_Knee_kp

    Data_send.Cal_L_Knee_kp_high = qAbs(Cal_L_Knee_kp) / 256;
    Data_send.Cal_L_Knee_kp_low  = qAbs(Cal_L_Knee_kp) % 256;

    //Cal_L_Ankle_kp

    Data_send.Cal_L_Ankle_kp_high = qAbs(Cal_L_Ankle_kp) / 256;
    Data_send.Cal_L_Ankle_kp_low  = qAbs(Cal_L_Ankle_kp) % 256;


    //L-kd//
    //Cal_L_Hipx_kd

    Data_send.Cal_L_Hipx_kd_high = qAbs(Cal_L_Hipx_kd) / 256;
    Data_send.Cal_L_Hipx_kd_low  = qAbs(Cal_L_Hipx_kd) % 256;

    //Cal_L_Hipz_kd

    Data_send.Cal_L_Hipz_kd_high = qAbs(Cal_L_Hipz_kd) / 256;
    Data_send.Cal_L_Hipz_kd_low  = qAbs(Cal_L_Hipz_kd) % 256;

    //Cal_L_Hipy_kd

    Data_send.Cal_L_Hipy_kd_high = qAbs(Cal_L_Hipy_kd) / 256;
    Data_send.Cal_L_Hipy_kd_low  = qAbs(Cal_L_Hipy_kd) % 256;

    //Cal_L_Knee_kd

    Data_send.Cal_L_Knee_kd_high = qAbs(Cal_L_Knee_kd) / 256;
    Data_send.Cal_L_Knee_kd_low  = qAbs(Cal_L_Knee_kd) % 256;

    //Cal_L_Ankle_kd

    Data_send.Cal_L_Ankle_kd_high = qAbs(Cal_L_Ankle_kd) / 256;
    Data_send.Cal_L_Ankle_kd_low  = qAbs(Cal_L_Ankle_kd) % 256;

    //L-extra_torque//
    //Cal_L_Hipx_extra_torque
    if(Cal_L_Hipx_extra_torque>0)
    {
        Data_send.Cal_L_Hipx_extra_torque_flag = 0;
    }
    else
    {
        Data_send.Cal_L_Hipx_extra_torque_flag = 1;
    }
    Data_send.Cal_L_Hipx_extra_torque_high = qAbs(Cal_L_Hipx_extra_torque) / 256;
    Data_send.Cal_L_Hipx_extra_torque_low  = qAbs(Cal_L_Hipx_extra_torque) % 256;

    //Cal_L_Hipz_extra_torque
    if(Cal_L_Hipz_extra_torque>0)
    {
        Data_send.Cal_L_Hipz_extra_torque_flag = 0;
    }
    else
    {
        Data_send.Cal_L_Hipz_extra_torque_flag = 1;
    }
    Data_send.Cal_L_Hipz_extra_torque_high = qAbs(Cal_L_Hipz_extra_torque) / 256;
    Data_send.Cal_L_Hipz_extra_torque_low  = qAbs(Cal_L_Hipz_extra_torque) % 256;

    //Cal_L_Hipy_extra_torque
    if(Cal_L_Hipy_extra_torque>0)
    {
        Data_send.Cal_L_Hipy_extra_torque_flag = 0;
    }
    else
    {
        Data_send.Cal_L_Hipy_extra_torque_flag = 1;
    }
    Data_send.Cal_L_Hipy_extra_torque_high = qAbs(Cal_L_Hipy_extra_torque) / 256;
    Data_send.Cal_L_Hipy_extra_torque_low  = qAbs(Cal_L_Hipy_extra_torque) % 256;

    //Cal_L_Knee_extra_torque
    if(Cal_L_Knee_extra_torque>0)
    {
        Data_send.Cal_L_Knee_extra_torque_flag = 0;
    }
    else
    {
        Data_send.Cal_L_Knee_extra_torque_flag = 1;
    }
    Data_send.Cal_L_Knee_extra_torque_high = qAbs(Cal_L_Knee_extra_torque) / 256;
    Data_send.Cal_L_Knee_extra_torque_low  = qAbs(Cal_L_Knee_extra_torque) % 256;

    //Cal_L_Ankle_extra_torque
    if(Cal_L_Ankle_extra_torque>0)
    {
        Data_send.Cal_L_Ankle_extra_torque_flag = 0;
    }
    else
    {
        Data_send.Cal_L_Ankle_extra_torque_flag = 1;
    }
    Data_send.Cal_L_Ankle_extra_torque_high = qAbs(Cal_L_Ankle_extra_torque) / 256;
    Data_send.Cal_L_Ankle_extra_torque_low  = qAbs(Cal_L_Ankle_extra_torque) % 256;


    //R-extra_torque//
    //Cal_R_Hipx_extra_torque
    if(Cal_R_Hipx_extra_torque>0)
    {
        Data_send.Cal_R_Hipx_extra_torque_flag = 0;
    }
    else
    {
        Data_send.Cal_R_Hipx_extra_torque_flag = 1;
    }
    Data_send.Cal_R_Hipx_extra_torque_high = qAbs(Cal_R_Hipx_extra_torque) / 256;
    Data_send.Cal_R_Hipx_extra_torque_low  = qAbs(Cal_R_Hipx_extra_torque) % 256;

    //Cal_R_Hipz_extra_torque
    if(Cal_R_Hipz_extra_torque>0)
    {
        Data_send.Cal_R_Hipz_extra_torque_flag = 0;
    }
    else
    {
        Data_send.Cal_R_Hipz_extra_torque_flag = 1;
    }
    Data_send.Cal_R_Hipz_extra_torque_high = qAbs(Cal_R_Hipz_extra_torque) / 256;
    Data_send.Cal_R_Hipz_extra_torque_low  = qAbs(Cal_R_Hipz_extra_torque) % 256;

    //Cal_R_Hipy_extra_torque
    if(Cal_R_Hipy_extra_torque>0)
    {
        Data_send.Cal_R_Hipy_extra_torque_flag = 0;
    }
    else
    {
        Data_send.Cal_R_Hipy_extra_torque_flag = 1;
    }
    Data_send.Cal_R_Hipy_extra_torque_high = qAbs(Cal_R_Hipy_extra_torque) / 256;
    Data_send.Cal_R_Hipy_extra_torque_low  = qAbs(Cal_R_Hipy_extra_torque) % 256;

    //Cal_R_Knee_extra_torque
    if(Cal_R_Knee_extra_torque>0)
    {
        Data_send.Cal_R_Knee_extra_torque_flag = 0;
    }
    else
    {
        Data_send.Cal_R_Knee_extra_torque_flag = 1;
    }
    Data_send.Cal_R_Knee_extra_torque_high = qAbs(Cal_R_Knee_extra_torque) / 256;
    Data_send.Cal_R_Knee_extra_torque_low  = qAbs(Cal_R_Knee_extra_torque) % 256;

    //Cal_R_Ankle_extra_torque
    if(Cal_R_Ankle_extra_torque>0)
    {
        Data_send.Cal_R_Ankle_extra_torque_flag = 0;
    }
    else
    {
        Data_send.Cal_R_Ankle_extra_torque_flag = 1;
    }
    Data_send.Cal_R_Ankle_extra_torque_high = qAbs(Cal_R_Ankle_extra_torque) / 256;
    Data_send.Cal_R_Ankle_extra_torque_low  = qAbs(Cal_R_Ankle_extra_torque) % 256;

    //servo_theta//
    if(servo_degree>0)
    {
        Data_send.servo_theta_flag = 0;
    }
    else
    {
        Data_send.servo_theta_flag = 1;
    }
    Data_send.servo_theta_high = qAbs(Cal_servo_degree) / 256;
    Data_send.servo_theta_low  = qAbs(Cal_servo_degree) % 256;



    //motor flag
    Data_send.motor_last_flag = 1;

    //servo_flag
    Data_send.servo_flag = 1;




    //send data to Jet-HR2
    u8 buf[124];
    u8 *msg = buf;
    memcpy(msg,&Data_send,sizeof(Data_send));
    //CRC rectify
    Utilities::getCRC16(buf,124);
    // buf[0]=0xff;
    QByteArray JCdata((char*)buf,124);  //buffer length
    if(send_mode == 1)
    {
        //        QString senddata = JCdata.toHex();
        //socket->writeDatagram(JCdata.data(),JCdata.size(),QHostAddress("169.254.146.133"),8090);
        socket->writeDatagram(JCdata.data(),JCdata.size(),QHostAddress("192.168.1.30"),8089); // 实验室样机地址
        // socket->writeDatagram(JCdata.data(),JCdata.size(),QHostAddress("192.168.0.113"),8081);
        // socket->writeDatagram(JCdata.data(),JCdata.size(),QHostAddress("10.23.15.21"),8081);

    }
}

int data_deal1(QString str)
{
    QString flag_str="";
    QString high_str="";
    QString low_str="";
    int flag = 0;
    int high = 0;
    int low = 0;
    int value = 0;
    flag_str = flag_str+str[0]+str[1];
    high_str = high_str+str[2]+str[3];
    low_str = low_str+str[4]+str[5];

    flag =flag_str.toInt(NULL,16);
    high =high_str.toInt(NULL,16);
    low =low_str.toInt(NULL,16);
    value = high*256+low;
    if(flag == 1)
        value = 0-value;
    return value;
}

void deal_msg::getmsg()
{
    //get data from Jet-HR2
    int tmMsec_up = fTimeCounter_up.elapsed();
    int ms = tmMsec_up%1000;


    str_upping_time = QString::asprintf(" %d milesecond",ms);
    fTimeCounter_up.start();



    while(socket->hasPendingDatagrams())
    {
        QString data_txt = "";
        QString data_count = "";
        qint64 dateGramSize =  socket->pendingDatagramSize();
        QByteArray buf((int)dateGramSize,0);
        QHostAddress cliaddr;
        quint16 port;
        QString str;
        qint64 len = socket->readDatagram(buf.data(),buf.size(),&cliaddr,&port);
        // qDebug() << "cliaddr is: " << cliaddr;

        char *JC2HCdata = buf.data();
        QString strhex = QString(buf.toHex());

        if (port != 8089)
        {
            qDebug() << "Data error";
            break;
        }
        // if(Utilities::checkCRC16((u8 *)JC2HCdata,buf.size()))
        // {
        //     // qDebug() << "Data error";
        //     break;
        // }

        int i=0;
        int data2value = 0;

        ////////   imu_roll   ////////
        for(i=0;i<6;i++)
        {
            data_count = data_count+strhex[i];
        }
        data2value = data_deal1(data_count);


        imu_euler_roll=data2value*0.01 - imu_euler_roll_ref;
        data_txt = data_txt + QString::number(imu_euler_roll) + " ";

        //imu_pitch
        data_count = "";

        ////////   imu_pitch   ////////
        for(;i<12;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        imu_euler_pitch=data2value*0.01 - imu_euler_pitch_ref;
        data_txt = data_txt + QString::number(imu_euler_pitch) + " ";

        //imu_yaw
        data_count = "";
        for(;i<18;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        imu_euler_yaw=data2value*0.01   - imu_euler_yaw_ref ;
        data_txt = data_txt + QString::number(imu_euler_yaw) + " ";



        //imu_Roll_v
        data_count = "";
        for(;i<24;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        imu_euler_roll_v= data2value*0.01;
        data_txt = data_txt + QString::number(imu_euler_roll_v) + " ";

        ////////   imu_pitchV  ////////
        data_count = "";
        for(;i<30;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        imu_euler_pitch_v = data2value*0.01;
        data_txt = data_txt + QString::number(imu_euler_pitch_v) + " ";


        ////////   imu_yawV  ////////
        data_count = "";
        for(;i<36;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        imu_euler_yaw_v = data2value*0.01;
        data_txt = data_txt + QString::number(imu_euler_yaw_v) + " ";


        ////////   R_Hipx  ////////
        data_count = "";
        for(;i<42;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Hipx_degree = data2value*0.01;
        data_txt = data_txt + QString::number(R_Hipx_degree) + " ";

        ////////   R_Hipz  ////////
        data_count = "";
        for(;i<48;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Hipz_degree=data2value*0.01;
        data_txt = data_txt + QString::number(R_Hipz_degree) + " ";

        ////////   R_Hipy  ////////
        data_count = "";
        for(;i<54;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Hipy_degree=data2value*0.01;
        data_txt = data_txt + QString::number(R_Hipy_degree) + " ";

        ////////   R_Knee  ////////
        data_count = "";
        for(;i<60;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Knee_degree=data2value*0.01;
        data_txt = data_txt + QString::number(R_Knee_degree) + " ";

        ////////   R_Ankle  ////////
        data_count = "";
        for(;i<66;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Ankle_degree=data2value*0.01;
        data_txt = data_txt + QString::number(R_Ankle_degree) + " ";

        ////////   L_Hipx  ////////
        data_count = "";
        for(;i<72;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Hipx_degree = data2value*0.01;
        data_txt = data_txt + QString::number(L_Hipx_degree) + " ";

        ////////   L_Hipz  ////////
        data_count = "";
        for(;i<78;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Hipz_degree = data2value*0.01;
        data_txt = data_txt + QString::number(L_Hipz_degree) + " ";

        ////////   L_Hipy  ////////
        data_count = "";
        for(;i<84;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Hipy_degree = data2value*0.01;
        data_txt = data_txt + QString::number(L_Hipy_degree) + " ";

        ////////   L_Knee  ////////
        data_count = "";
        for(;i<90;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Knee_degree = data2value*0.01;
        data_txt = data_txt + QString::number(L_Knee_degree) + " ";

        ////////   L_Ankle  ////////
        data_count = "";
        for(;i<96;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Ankle_degree = data2value*0.01;
        data_txt = data_txt + QString::number(L_Ankle_degree) + " ";

        ////////   R_Hipx_velocity  ////////
        data_count = "";
        for(;i<102;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Hipx_velocity = data2value*0.01;
        data_txt = data_txt + QString::number(R_Hipx_velocity) + " ";
        //        ui->lineEdit_v_motor5->setText(QString::number(R_Hipx_velocity));

        ////////   R_Hipz_velocity  ////////
        data_count = "";
        for(;i<108;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Hipz_velocity = data2value*0.01;
        data_txt = data_txt + QString::number(R_Hipz_velocity) + " ";
        //        ui->lineEdit_i_motor5->setText(QString::number(data2value*0.01));

        ////////   R_Hipy_velocity  ////////
        data_count = "";
        for(;i<114;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Hipy_velocity = data2value*0.01;
        data_txt = data_txt + QString::number(R_Hipy_velocity) + " ";
        //        ui->lineEdit_p_motor6->setText(QString::number(data2value*0.01));

        ////////   R_Knee_velocity  ////////
        data_count = "";
        for(;i<120;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Knee_velocity = data2value*0.01;
        data_txt = data_txt + QString::number(R_Knee_velocity) + " ";
        //        ui->lineEdit_v_motor6->setText(QString::number(data2value*0.1));

        ////////   R_Ankle_velocity  ////////
        data_count = "";
        for(;i<126;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Ankle_velocity = data2value*0.01;
        data_txt = data_txt + QString::number(R_Ankle_velocity) + " ";
        //        ui->lineEdit_i_motor6->setText(QString::number(data2value*0.01));

        ////////   L_Hipx_velocity  ////////
        data_count = "";
        for(;i<132;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Hipx_velocity = data2value*0.01;
        data_txt = data_txt + QString::number(L_Hipx_velocity) + " ";

        ////////   L_Hipz_velocity  ////////
        data_count = "";
        for(;i<138;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Hipz_velocity = data2value*0.01;
        data_txt = data_txt + QString::number(L_Hipz_velocity) + " ";

        ////////   L_Hipy_velocity  ////////
        data_count = "";
        for(;i<144;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Hipy_velocity = data2value*0.01;
        data_txt = data_txt + QString::number(L_Hipy_velocity) + " ";

        ////////   L_Knee_velocity  ////////
        data_count = "";
        for(;i<150;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Knee_velocity = data2value*0.01;
        data_txt = data_txt + QString::number(L_Knee_velocity) + " ";
        //        ui->lineEdit_v_motor6->setText(QString::number(data2value*0.1));

        ////////   L_Ankle_velocity  ////////
        data_count = "";
        for(;i<156;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Ankle_velocity = data2value*0.01;
        data_txt = data_txt + QString::number(L_Ankle_velocity) + " ";

        ////////   R_Hipx_current  ////////
        data_count = "";
        for(;i<162;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Hipx_current = data2value*0.01;
        data_txt = data_txt + QString::number(R_Hipx_current) + " ";

        ////////   R_Hipz_current  ////////
        data_count = "";
        for(;i<168;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Hipz_current = data2value*0.01;
        data_txt = data_txt + QString::number(R_Hipz_current) + " ";

        ////////   R_Hipy_current  ////////
        data_count = "";
        for(;i<174;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Hipy_current = data2value*0.01;
        data_txt = data_txt + QString::number(R_Hipy_current) + " ";

        ////////   R_Knee_current  ////////
        data_count = "";
        for(;i<180;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Knee_current = data2value*0.01;
        data_txt = data_txt + QString::number(R_Knee_current) + " ";
        //        ui->lineEdit_v_motor6->setText(QString::number(data2value*0.1));

        ////////   R_Ankle_current  ////////
        data_count = "";
        for(;i<186;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Ankle_current = data2value*0.01;
        data_txt = data_txt + QString::number(R_Ankle_current) + " ";
        //        ui->lineEdit_i_motor6->setText(QString::number(data2value*0.01));

        ////////   L_Hipx_current  ////////
        data_count = "";
        for(;i<192;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Hipx_current = data2value*0.01;
        data_txt = data_txt + QString::number(L_Hipx_current) + " ";

        ////////   L_Hipz_current  ////////
        data_count = "";
        for(;i<198;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Hipz_current = data2value*0.01;
        data_txt = data_txt + QString::number(L_Hipz_current) + " ";

        ////////   L_Hipy_current  ////////
        data_count = "";
        for(;i<204;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Hipy_current = data2value*0.01;
        data_txt = data_txt + QString::number(L_Hipy_current) + " ";

        ////////   L_Knee_current  ////////
        data_count = "";
        for(;i<210;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Knee_current = data2value*0.01;
        data_txt = data_txt + QString::number(L_Knee_current) + " ";
        //        ui->lineEdit_v_motor6->setText(QString::number(data2value*0.1));

        ////////   L_Ankle_current  ////////
        data_count = "";
        for(;i<216;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Ankle_current = data2value*0.01;
        data_txt = data_txt + QString::number(L_Ankle_current) + " ";

        ////////   R_Hipx_connect_flag  ////////
        data_count = "";
        for(;i<222;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Hipx_connect_flag = data2value*0.01;
        data_txt = data_txt + QString::number(R_Hipx_connect_flag) + " ";

        ////////   R_Hipz_connect_flag  ////////
        data_count = "";
        for(;i<228;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Hipz_connect_flag = data2value*0.01;
        data_txt = data_txt + QString::number(R_Hipz_connect_flag) + " ";

        ////////   R_Hipy_connect_flag  ////////
        data_count = "";
        for(;i<234;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Hipy_connect_flag = data2value*0.01;
        data_txt = data_txt + QString::number(R_Hipy_connect_flag) + " ";

        ////////   R_Knee_connect_flag  ////////
        data_count = "";
        for(;i<240;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Knee_connect_flag = data2value*0.01;
        data_txt = data_txt + QString::number(R_Knee_connect_flag) + " ";
        //        ui->lineEdit_v_motor6->setText(QString::number(data2value*0.1));

        ////////   R_Ankle_connect_flag  ////////
        data_count = "";
        for(;i<246;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        R_Ankle_connect_flag = data2value*0.01;
        data_txt = data_txt + QString::number(R_Ankle_connect_flag) + " ";
        //        ui->lineEdit_i_motor6->setText(QString::number(data2value*0.01));

        ////////   L_Hipx_connect_flag  ////////
        data_count = "";
        for(;i<252;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Hipx_connect_flag = data2value*0.01;
        data_txt = data_txt + QString::number(L_Hipx_connect_flag) + " ";

        ////////   L_Hipz_connect_flag  ////////
        data_count = "";
        for(;i<258;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Hipz_connect_flag = data2value*0.01;
        data_txt = data_txt + QString::number(L_Hipz_connect_flag) + " ";

        ////////   L_Hipy_connect_flag  ////////
        data_count = "";
        for(;i<264;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Hipy_connect_flag = data2value*0.01;
        data_txt = data_txt + QString::number(L_Hipy_connect_flag) + " ";

        ////////   L_Knee_connect_flag  ////////
        data_count = "";
        for(;i<270;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Knee_connect_flag = data2value*0.01;
        data_txt = data_txt + QString::number(L_Knee_connect_flag) + " ";

        ////////   L_Ankle_connect_flag  ////////
        data_count = "";
        for(;i<276;i++)
            data_count = data_count+strhex[i];
        data2value = data_deal1(data_count);
        L_Ankle_connect_flag = data2value*0.01;
        data_txt = data_txt + QString::number(L_Ankle_connect_flag) + " ";


        //////////////// Pybullet中的位置和速度信息（样机实验时记得注释）//////////////////

        // ////////   Pybullet中的x  ////////
        // data_count = "";
        // for(;i<282;i++)
        //     data_count = data_count+strhex[i];
        // data2value = data_deal1(data_count);
        // position_x = data2value*0.01;
        // data_txt = data_txt + QString::number(position_x) + " ";

        // ////////   Pybullet中的y  ////////
        // data_count = "";
        // for(;i<288;i++)
        //     data_count = data_count+strhex[i];
        // data2value = data_deal1(data_count);
        // position_y = data2value*0.01;
        // data_txt = data_txt + QString::number(position_y) + " ";

        // ////////   Pybullet中的z  ////////
        // data_count = "";
        // for(;i<294;i++)
        //     data_count = data_count+strhex[i];
        // data2value = data_deal1(data_count);
        // position_z = data2value*0.01;
        // data_txt = data_txt + QString::number(position_z) + " ";
        // qDebug() << "pos_z: " << data_count;

        // ////////   Pybullet中的dx  ////////
        // data_count = "";
        // for(;i<300;i++)
        //     data_count = data_count+strhex[i];
        // data2value = data_deal1(data_count);
        // velocity_x = data2value*0.01;
        // data_txt = data_txt + QString::number(velocity_x) + " ";

        // ////////   Pybullet中的dy  ////////
        // data_count = "";
        // for(;i<306;i++)
        //     data_count = data_count+strhex[i];
        // data2value = data_deal1(data_count);
        // velocity_y = data2value*0.01;
        // data_txt = data_txt + QString::number(velocity_y) + " ";

        // ////////   Pybullet中的dz  ////////
        // data_count = "";
        // for(;i<312;i++)
        //     data_count = data_count+strhex[i];
        // data2value = data_deal1(data_count);
        // velocity_z = data2value*0.01;
        // data_txt = data_txt + QString::number(velocity_z) + " ";


        /////////////////////////////////////////////////////////////////////////////

        if( rectify_imu_flag == 1)
        {

            imu_euler_roll_array[rectify_imu_count] = imu_euler_roll;
            imu_euler_pitch_array[rectify_imu_count] = imu_euler_pitch;
            imu_euler_yaw_array[rectify_imu_count] = imu_euler_yaw;
            rectify_imu_count++;

            std::cout << "send done" <<rectify_imu_count<< std::endl;
            if(rectify_imu_count==100)
            {
                imu_euler_roll_ref  = (std::accumulate(imu_euler_roll_array.begin(),imu_euler_roll_array.end(),0.0))/100;
                imu_euler_pitch_ref = (std::accumulate(imu_euler_pitch_array.begin(),imu_euler_pitch_array.end(),0.0))/100;
                imu_euler_yaw_ref   = (std::accumulate(imu_euler_yaw_array.begin(),imu_euler_yaw_array.end(),0.0))/100;
                rectify_imu_flag = 0;
            }
        }
    }
}

void deal_msg::nomsg()
{
    zero_flag = 5;
}

void deal_msg::run()
{

    m_stop=false;//启动线程时令m_stop=false


    while(!m_stop)//循环主体
    {

    }

    quit();
}
