#ifndef QMYTHREAD_H
#define QMYTHREAD_H


#include <QThread>
#include <QDebug>
#include <inttypes.h>
#include "NatNet/include/NatNetCAPI.h"
#include "NatNet/include/NatNetClient.h"
#include "NatNet/include/NatNetTypes.h"
// #include "NatNetCAPI.h"
// #include "NatNetClient.h"
// #include "NatNetTypes.h"
#include "conio.h"
#include <stdio.h>
#include <iostream>
#include <QTime>

#ifndef _WIN32
char getch();
#endif
void _WriteHeader(FILE* fp, sDataDescriptions* pBodyDefs);
void _WriteFrame(FILE* fp, sFrameOfMocapData* data);
void _WriteFooter(FILE* fp);

class QThreadDAQ : public QThread
{//数据采集线程
    Q_OBJECT

private:

protected:
    void    run() Q_DECL_OVERRIDE;
    //public:
    //    static bool controll_flag;
    //public:
    //    static char controll_comman;
public:
    //motive
    static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;

    static bool controll_flag;
    static char controll_comman;

    bool    m_stop=false; //停止线程

    double  getData0();
    double  getData1();
    double  getData2();
    double  getData3();
    double  getData4();
    double  getData5();
    double  getData6();
    double  getData7();

    NatNetClient* g_pClient = NULL;
    FILE* g_outputFile;

    std::vector< sNatNetDiscoveredServer > g_discoveredServers;
    sNatNetClientConnectParams g_connectParams;
    char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS;
    int g_analogSamplesPerMocapFrame = 0;
    sServerDescription g_serverDescription;

    static QThreadDAQ *nat;
    static QThreadDAQ *s_this;

    static void NATNET_CALLCONV ServerDiscoveredCallback( const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext );
    static void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);    // receives data from the server
    static void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg);      // receives NatNet error messages
    void resetClient();
    int ConnectClient();
    int Motive_Init(char controll_comman);
    //多线程
    QThreadDAQ();
    void    stopThread();
};

//class QThreadShow : public QThread
//{//数据读取线程
//    Q_OBJECT
//private:
//    bool    m_stop=false; //停止线程
//    void    run() Q_DECL_OVERRIDE;
//private slots:

//public:
//    QThreadShow();
//    void    stopThread();
//    double  getData0();
//    double  getData1();
//    double  getData2();
//    double  getData3();
//    double  getData4();
//    double  getData5();
//    double  getData6();
//    double  getData7();

//signals:
//    void    newValue(float *data,int count, int seq);
//protected:
//    double mythread_data;
//};
#endif // QMYTHREAD_H
