#include    "qmythread.h"
#include    <QSemaphore>

QThreadDAQ * QThreadDAQ::nat = nullptr;

const int BufferSize = 8;
float buffer1[BufferSize];//储存 ID 时间
float buffer2[BufferSize];//储存 位置角度
int curBuf=1; //当前正在写入的Buffer
int bufNo=0; //采集的缓冲区序号
//global char c'
static double bufferData[8];

quint8   counter=0;//数据生成器

QSemaphore emptyBufs(2);//信号量：空的缓冲区个数，初始资源个数为2
QSemaphore fullBufs; //满的缓冲区个数，初始资源为0

QThreadDAQ::QThreadDAQ()
{
    nat = this;
    //         qDebug() << "wkr7";
}

void QThreadDAQ::stopThread()
{
    m_stop=true;
}

void QThreadDAQ::run()
{
    m_stop=false;//启动线程时令m_stop=false
    bufNo=0;//缓冲区序号
    curBuf=1; //当前写入使用的缓冲区
    counter=0;//数据生成器

    int n=emptyBufs.available();

    if (n<2)  //保证 线程启动时emptyBufs.available==2
        emptyBufs.release(2-n);

    while(!m_stop)//循环主体
    {
        //        emptyBufs.acquire();//获取一个空的缓冲区
        Motive_Init(controll_comman);

        //        fullBufs.release(); //有了一个满的缓冲区,available==1
    }

    quit();
}


double QThreadDAQ::getData0()
{
    return buffer2[0];
}
double QThreadDAQ::getData1()
{
    return buffer2[1];
}
double QThreadDAQ::getData2()
{
    return buffer2[2];
}
double QThreadDAQ::getData3()
{
    return buffer2[3];
}
double QThreadDAQ::getData4()
{
    return buffer2[4];
}
double QThreadDAQ::getData5()
{
    return buffer2[5];
}
double QThreadDAQ::getData6()
{
    return buffer2[6];
}




int QThreadDAQ::Motive_Init(char controll_comman)
{
    //     print version info
    unsigned char ver[4];
    NatNet_GetVersion( ver );
    printf( "NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3] );

    // Install logging callback
    NatNet_SetLogCallback( MessageHandler );

    // create NatNet client
    g_pClient = new NatNetClient();

    // set the frame callback handler
    g_pClient->SetFrameReceivedCallback( DataHandler, g_pClient );	// this function will receive data from the server

    // If no arguments were specified on the command line,
    // attempt to discover servers on the local network.
    // ------------------------------------------------------------------
    //        if ( argc == 1 )
    // ------------------------------------------------------------------
    if(__argc == 1)
    {
        // An example of synchronous server discovery.
#if 0
        const unsigned int kDiscoveryWaitTimeMillisec = 5 * 1000; // Wait 5 seconds for responses.
        const int kMaxDescriptions = 10; // Get info for, at most, the first 10 servers to respond.
        sNatNetDiscoveredServer servers[kMaxDescriptions];
        int actualNumDescriptions = kMaxDescriptions;
        NatNet_BroadcastServerDiscovery( servers, &actualNumDescriptions );

        if ( actualNumDescriptions < kMaxDescriptions )
        {
            // If this happens, more servers responded than the array was able to store.
        }
#endif

        // Do asynchronous server discovery.
        printf( "Looking for servers on the local network.\n" );
        printf( "Press the number key that corresponds to any discovered server to connect to that server.\n" );
        printf( "Press Q at any time to quit.\n\n" );

        NatNetDiscoveryHandle discovery;

        NatNet_CreateAsyncServerDiscovery( &discovery, ServerDiscoveredCallback);

        while ( const int c = controll_comman )
        {

            if ( c >= '1' && c <= '9' )
            {
                const size_t serverIndex = c - '1';
                if ( serverIndex < g_discoveredServers.size() )
                {
                    const sNatNetDiscoveredServer& discoveredServer = g_discoveredServers[serverIndex];

                    if ( discoveredServer.serverDescription.bConnectionInfoValid )
                    {
                        // Build the connection parameters.
#ifdef _WIN32
                        _snprintf_s(
#else
                        snprintf(
#endif
                            g_discoveredMulticastGroupAddr, sizeof g_discoveredMulticastGroupAddr,
                            "%" PRIu8 ".%" PRIu8".%" PRIu8".%" PRIu8"",
                            discoveredServer.serverDescription.ConnectionMulticastAddress[0],
                            discoveredServer.serverDescription.ConnectionMulticastAddress[1],
                            discoveredServer.serverDescription.ConnectionMulticastAddress[2],
                            discoveredServer.serverDescription.ConnectionMulticastAddress[3]
                            );
                        //                        qDebug() << "I am here";
                        g_connectParams.connectionType = discoveredServer.serverDescription.ConnectionMulticast ? ConnectionType_Multicast : ConnectionType_Unicast;
                        g_connectParams.serverCommandPort = discoveredServer.serverCommandPort;
                        g_connectParams.serverDataPort = discoveredServer.serverDescription.ConnectionDataPort;
                        g_connectParams.serverAddress = discoveredServer.serverAddress;
                        g_connectParams.localAddress = discoveredServer.localAddress;
                        g_connectParams.multicastAddress = g_discoveredMulticastGroupAddr;
                    }
                    else
                    {
                        // We're missing some info because it's a legacy server.
                        // Guess the defaults and make a best effort attempt to connect.
                        g_connectParams.connectionType = kDefaultConnectionType;
                        g_connectParams.serverCommandPort = discoveredServer.serverCommandPort;
                        g_connectParams.serverDataPort = 0;
                        g_connectParams.serverAddress = discoveredServer.serverAddress;
                        g_connectParams.localAddress = discoveredServer.localAddress;
                        g_connectParams.multicastAddress = NULL;
                    }

                    break;
                }
            }
            else if ( c == 'q' )
            {
                //                qDebug() << "wkr8";
                return 0;
            }
        }
        //        qDebug() << "wkr9";
        NatNet_FreeAsyncServerDiscovery( discovery );
    }
    // ------------------------------------------------------------------
    //        else
    //        {
    //            g_connectParams.connectionType = kDefaultConnectionType;

    //            if ( argc >= 2 )
    //            {
    //                g_connectParams.serverAddress = argv[1];
    //            }

    //            if ( argc >= 3 )
    //            {
    //                g_connectParams.localAddress = argv[2];
    //            }
    //        }
    // ------------------------------------------------------------------

    int iResult;

    // Connect to Motive
    iResult = ConnectClient();
    if (iResult != ErrorCode_OK)
    {
        printf("Error initializing client.  See log for details.  Exiting");
        return 1;
    }
    else
    {
        printf("Client initialized and ready.\n");
    }


    // Send/receive test request
    void* response;
    int nBytes;
    printf("[SampleClient] Sending Test Request\n");
    iResult = g_pClient->SendMessageAndWait("TestRequest", &response, &nBytes);
    if (iResult == ErrorCode_OK)
    {
        printf("[SampleClient] Received: %s", (char*)response);
    }

    // Retrieve Data Descriptions from Motive
    printf("\n\n[SampleClient] Requesting Data Descriptions...");
    sDataDescriptions* pDataDefs = NULL;
    iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
    if (iResult != ErrorCode_OK || pDataDefs == NULL)
    {
        printf("[SampleClient] Unable to retrieve Data Descriptions.");
    }
    else
    {
        printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions );
        for(int i=0; i < pDataDefs->nDataDescriptions; i++)
        {
            printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
            if(pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
            {
                // MarkerSet
                sMarkerSetDescription* pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
                printf("MarkerSet Name : %s\n", pMS->szName);
                for(int i=0; i < pMS->nMarkers; i++)
                    printf("%s\n", pMS->szMarkerNames[i]);

            }
            else if(pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
            {
                // RigidBody
                sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
                printf("RigidBody Name : %s\n", pRB->szName);
                printf("RigidBody ID : %d\n", pRB->ID);
                printf("RigidBody Parent ID : %d\n", pRB->parentID);
                printf("Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);

                if ( pRB->MarkerPositions != NULL && pRB->MarkerRequiredLabels != NULL )
                {
                    for ( int markerIdx = 0; markerIdx < pRB->nMarkers; ++markerIdx )
                    {
                        const MarkerData& markerPosition = pRB->MarkerPositions[markerIdx];
                        const int markerRequiredLabel = pRB->MarkerRequiredLabels[markerIdx];

                        printf( "\tMarker #%d:\n", markerIdx );
                        printf( "\t\tPosition: %.2f, %.2f, %.2f\n", markerPosition[0], markerPosition[1], markerPosition[2] );

                        if ( markerRequiredLabel != 0 )
                        {
                            printf( "\t\tRequired active label: %d\n", markerRequiredLabel );
                        }
                    }
                }
            }
            else if(pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton)
            {
                // Skeleton
                sSkeletonDescription* pSK = pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
                printf("Skeleton Name : %s\n", pSK->szName);
                printf("Skeleton ID : %d\n", pSK->skeletonID);
                printf("RigidBody (Bone) Count : %d\n", pSK->nRigidBodies);
                for(int j=0; j < pSK->nRigidBodies; j++)
                {
                    sRigidBodyDescription* pRB = &pSK->RigidBodies[j];
                    printf("  RigidBody Name : %s\n", pRB->szName);
                    printf("  RigidBody ID : %d\n", pRB->ID);
                    printf("  RigidBody Parent ID : %d\n", pRB->parentID);
                    printf("  Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
                }
            }
            else if(pDataDefs->arrDataDescriptions[i].type == Descriptor_ForcePlate)
            {
                // Force Plate
                sForcePlateDescription* pFP = pDataDefs->arrDataDescriptions[i].Data.ForcePlateDescription;
                printf("Force Plate ID : %d\n", pFP->ID);
                printf("Force Plate Serial : %s\n", pFP->strSerialNo);
                printf("Force Plate Width : %3.2f\n", pFP->fWidth);
                printf("Force Plate Length : %3.2f\n", pFP->fLength);
                printf("Force Plate Electrical Center Offset (%3.3f, %3.3f, %3.3f)\n", pFP->fOriginX,pFP->fOriginY, pFP->fOriginZ);
                for(int iCorner=0; iCorner<4; iCorner++)
                    printf("Force Plate Corner %d : (%3.4f, %3.4f, %3.4f)\n", iCorner, pFP->fCorners[iCorner][0],pFP->fCorners[iCorner][1],pFP->fCorners[iCorner][2]);
                printf("Force Plate Type : %d\n", pFP->iPlateType);
                printf("Force Plate Data Type : %d\n", pFP->iChannelDataType);
                printf("Force Plate Channel Count : %d\n", pFP->nChannels);
                for(int iChannel=0; iChannel<pFP->nChannels; iChannel++)
                    printf("\tChannel %d : %s\n", iChannel, pFP->szChannelNames[iChannel]);
            }
            else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Device)
            {
                // Peripheral Device
                sDeviceDescription* pDevice = pDataDefs->arrDataDescriptions[i].Data.DeviceDescription;
                printf("Device Name : %s\n", pDevice->strName);
                printf("Device Serial : %s\n", pDevice->strSerialNo);
                printf("Device ID : %d\n", pDevice->ID);
                printf("Device Channel Count : %d\n", pDevice->nChannels);
                for (int iChannel = 0; iChannel < pDevice->nChannels; iChannel++)
                    printf("\tChannel %d : %s\n", iChannel, pDevice->szChannelNames[iChannel]);
            }
            else
            {
                printf("Unknown data type.");
                // Unknown
            }
        }
    }


    // Create data file for writing received stream into
    const char* szFile = "Client-output.pts";
    // ------------------------------------------------------------------
    //        if (argc > 3)
    //            szFile = argv[3];
    // ------------------------------------------------------------------

    g_outputFile = fopen(szFile, "w");
    if(!g_outputFile)
    {
        printf("error opening output file %s.  Exiting.", szFile);
        exit(1);
    }

    if ( pDataDefs )
    {
        _WriteHeader( g_outputFile, pDataDefs );
        NatNet_FreeDescriptions( pDataDefs );
        pDataDefs = NULL;
    }

    // Ready to receive marker stream!
    printf("\nClient is connected to server and listening for data...\n");
    int c;
    bool bExit = false;
    printf("hjb1");
    while(c=controll_comman)
    {
        //        printf("hjb2\n");
        int c;
        switch(c)
        {
        case 'q':
            bExit =  true;
            break;
        case 'r':
            resetClient();
            break;
        case 'p':
            sServerDescription ServerDescription;
            memset(&ServerDescription, 0, sizeof(ServerDescription));
            g_pClient->GetServerDescription(&ServerDescription);
            if(!ServerDescription.HostPresent)
            {
                printf("Unable to connect to server. Host not present. Exiting.");
                return 1;
            }
            break;
        case 's':
        {
            printf("\n\n[SampleClient] Requesting Data Descriptions...");
            sDataDescriptions* pDataDefs = NULL;
            iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
            if (iResult != ErrorCode_OK || pDataDefs == NULL)
            {
                printf("[SampleClient] Unable to retrieve Data Descriptions.");
            }
            else
            {
                printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions);
            }
        }
        break;
        case 'm':	                        // change to multicast
            g_connectParams.connectionType = ConnectionType_Multicast;
            iResult = ConnectClient();
            if(iResult == ErrorCode_OK)
                printf("Client connection type changed to Multicast.\n\n");
            else
                printf("Error changing client connection type to Multicast.\n\n");
            break;
        case 'u':	                        // change to unicast
            g_connectParams.connectionType = ConnectionType_Unicast;
            iResult = ConnectClient();
            if(iResult == ErrorCode_OK)
                printf("Client connection type changed to Unicast.\n\n");
            else
                printf("Error changing client connection type to Unicast.\n\n");
            break;
        case 'c' :                          // connect
            iResult = ConnectClient();
            break;
        case 'd' :                          // disconnect
            // note: applies to unicast connections only - indicates to Motive to stop sending packets to that client endpoint
            iResult = g_pClient->SendMessageAndWait("Disconnect", &response, &nBytes);
            if (iResult == ErrorCode_OK)
                printf("[SampleClient] Disconnected");
            break;
        default:
            break;
        }
        if(bExit)
            break;
    }
    printf("hjb3\n");
    // Done - clean up.
    if (g_pClient)
    {
        g_pClient->Disconnect();
        delete g_pClient;
        g_pClient = NULL;
    }

    if (g_outputFile)
    {
        _WriteFooter(g_outputFile);
        fclose(g_outputFile);
        g_outputFile = NULL;
    }

    return ErrorCode_OK;
}



void _WriteHeader(FILE *fp, sDataDescriptions *pBodyDefs)
{
    int i=0;

    if(!pBodyDefs->arrDataDescriptions[0].type == Descriptor_MarkerSet)
        return;

    sMarkerSetDescription* pMS = pBodyDefs->arrDataDescriptions[0].Data.MarkerSetDescription;

    fprintf(fp, "<MarkerSet>\n\n");
    fprintf(fp, "<Name>\n%s\n</Name>\n\n", pMS->szName);

    fprintf(fp, "<Markers>\n");
    for(i=0; i < pMS->nMarkers; i++)
    {
        fprintf(fp, "%s\n", pMS->szMarkerNames[i]);
    }
    fprintf(fp, "</Markers>\n\n");

    fprintf(fp, "<Data>\n");
    fprintf(fp, "Frame#\t");
    for(i=0; i < pMS->nMarkers; i++)
    {
        fprintf(fp, "M%dX\tM%dY\tM%dZ\t", i, i, i);
    }
    fprintf(fp,"\n");
}

void _WriteFrame(FILE *fp, sFrameOfMocapData *data)
{
    //    fprintf(fp, "%d", data->iFrame);
    //    for(int i =0; i < data->MocapData->nMarkers; i++)
    //    {
    //        fprintf(fp, "\t%.5f\t%.5f\t%.5f", data->MocapData->Markers[i][0], data->MocapData->Markers[i][1], data->MocapData->Markers[i][2]);
    //    }
    //    fprintf(fp, "\n");
}

void _WriteFooter(FILE *fp)
{
    fprintf(fp, "</Data>\n\n");
    fprintf(fp, "</MarkerSet>\n");
}

void NATNET_CALLCONV QThreadDAQ::ServerDiscoveredCallback(const sNatNetDiscoveredServer *pDiscoveredServer, void *pUserContext)
{
    char serverHotkey = '.';

    if ( nat->g_discoveredServers.size() < 9 )
    {
        serverHotkey = static_cast<char>('1' + nat->g_discoveredServers.size());
    }

    const char* warning = "";

    if ( pDiscoveredServer->serverDescription.bConnectionInfoValid == false )
    {
        warning = " (WARNING: Legacy server, could not autodetect settings. Auto-connect may not work reliably.)";
    }

    printf( "[%c] %s %d.%d at %s%s\n",
           serverHotkey,
           pDiscoveredServer->serverDescription.szHostApp,
           pDiscoveredServer->serverDescription.HostAppVersion[0],
           pDiscoveredServer->serverDescription.HostAppVersion[1],
           pDiscoveredServer->serverAddress,
           warning );
    nat->g_discoveredServers.push_back( *pDiscoveredServer );
}

void NATNET_CALLCONV QThreadDAQ::MessageHandler(Verbosity msgType, const char *msg)
{
    // Optional: Filter out debug messages
    if ( msgType < Verbosity_Info )
    {
        return;
    }

    printf( "\n[NatNetLib]" );

    switch ( msgType )
    {
    case Verbosity_Debug:
        printf( " [DEBUG]" );
        break;
    case Verbosity_Info:
        printf( "  [INFO]" );
        break;
    case Verbosity_Warning:
        printf( "  [WARN]" );
        break;
    case Verbosity_Error:
        printf( " [ERROR]" );
        break;
    default:
        printf( " [?????]" );
        break;
    }

    printf( ": %s\n", msg );
}

void NATNET_CALLCONV QThreadDAQ::DataHandler(sFrameOfMocapData *data, void *pUserData)
{
    if(QThreadDAQ::controll_flag)
    {
        NatNetClient* pClient = (NatNetClient*) pUserData;

        // Software latency here is defined as the span of time between:
        //   a) The reception of a complete group of 2D frames from the camera system (CameraDataReceivedTimestamp)
        // and
        //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
        //
        // This figure may appear slightly higher than the "software latency" reported in the Motive user interface,
        // because it additionally includes the time spent preparing to stream the data via NatNet.
        const uint64_t softwareLatencyHostTicks = data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
        const double softwareLatencyMillisec = (softwareLatencyHostTicks * 1000) / static_cast<double>(nat->g_serverDescription.HighResClockFrequency);

        // Transit latency is defined as the span of time between Motive transmitting the frame of data, and its reception by the client (now).
        // The SecondsSinceHostTimestamp method relies on NatNetClient's internal clock synchronization with the server using Cristian's algorithm.
        const double transitLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->TransmitTimestamp ) * 1000.0;

        int i=0;

        //        printf("FrameID : %d\n", data->iFrame);
        //        printf("Timestamp : %3.2lf\n", data->fTimestamp);
        //        printf("Software latency : %.2lf milliseconds\n", softwareLatencyMillisec);
        buffer1[0]=data->iFrame;
        buffer1[1]=data->fTimestamp;
        buffer1[2]=softwareLatencyMillisec;
        // Only recent versions of the Motive software in combination with ethernet camera systems support system latency measurement.
        // If it's unavailable (for example, with USB camera systems, or during playback), this field will be zero.
        const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;

        if ( bSystemLatencyAvailable )
        {
            // System latency here is defined as the span of time between:
            //   a) The midpoint of the camera exposure window, and therefore the average age of the photons (CameraMidExposureTimestamp)
            // and
            //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
            const uint64_t systemLatencyHostTicks = data->TransmitTimestamp - data->CameraMidExposureTimestamp;
            const double systemLatencyMillisec = (systemLatencyHostTicks * 1000) / static_cast<double>(nat->g_serverDescription.HighResClockFrequency);

            // Client latency is defined as the sum of system latency and the transit time taken to relay the data to the NatNet client.
            // This is the all-inclusive measurement (photons to client processing).
            const double clientLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->CameraMidExposureTimestamp ) * 1000.0;

            // You could equivalently do the following (not accounting for time elapsed since we calculated transit latency above):
            //const double clientLatencyMillisec = systemLatencyMillisec + transitLatencyMillisec;

            //            printf( "System latency : %.2lf milliseconds\n", systemLatencyMillisec );
            //            printf( "Total client latency : %.2lf milliseconds (transit time +%.2lf ms)\n", clientLatencyMillisec, transitLatencyMillisec );
            buffer1[3]=systemLatencyMillisec;
            buffer1[4]=clientLatencyMillisec;
            buffer1[5]=transitLatencyMillisec;
        }
        else
        {
            printf( "Transit latency : %.2lf milliseconds\n", transitLatencyMillisec );
            buffer1[6]=transitLatencyMillisec;
        }

        // FrameOfMocapData params
        bool bIsRecording = ((data->params & 0x01)!=0);
        bool bTrackedModelsChanged = ((data->params & 0x02)!=0);
        if(bIsRecording)
            printf("RECORDING\n");
        if(bTrackedModelsChanged)
            printf("Models Changed.\n");


        // timecode - for systems with an eSync and SMPTE timecode generator - decode to values
        int hour, minute, second, frame, subframe;
        NatNet_DecodeTimecode( data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe );
        // decode to friendly string
        char szTimecode[128] = "";
        NatNet_TimecodeStringify( data->Timecode, data->TimecodeSubframe, szTimecode, 128 );
        //        printf("Timecode : %s\n", szTimecode);

        // Rigid Bodies
        //        printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
        for(i=0; i < data->nRigidBodies; i++)
        {
            // params
            // 0x01 : bool, rigid body was successfully tracked in this frame
            bool bTrackingValid = data->RigidBodies[i].params & 0x01;

            //            printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
            //            printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
            //            printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
            //                   data->RigidBodies[i].x,
            //                   data->RigidBodies[i].y,
            //                   data->RigidBodies[i].z,
            //                   data->RigidBodies[i].qx,
            //                   data->RigidBodies[i].qy,
            //                   data->RigidBodies[i].qz,
            //                   data->RigidBodies[i].qw);
            buffer2[0]=data->RigidBodies[i].z;
            buffer2[1]=data->RigidBodies[i].x;
            buffer2[2]=data->RigidBodies[i].y;
            buffer2[3]=data->RigidBodies[i].qx;
            buffer2[4]=data->RigidBodies[i].qy;
            buffer2[5]=data->RigidBodies[i].qz;
            buffer2[6]=data->RigidBodies[i].qw;
        }
        for (int iPlate = 0; iPlate < data->nForcePlates; iPlate++)
        {
            printf("Force Plate %d\n", data->ForcePlates[iPlate].ID);
            for (int iChannel = 0; iChannel < data->ForcePlates[iPlate].nChannels; iChannel++)
            {
                // printf("\tChannel %d:\t", iChannel);
                // if (data->ForcePlates[iPlate].ChannelData[iChannel].nFrames == 0)
                // {
                //     printf("\tEmpty Frame\n");
                // }
                // else if (data->ForcePlates[iPlate].ChannelData[iChannel].nFrames != g_analogSamplesPerMocapFrame)
                // {
                //     printf("\tPartial Frame [Expected:%d   Actual:%d]\n", g_analogSamplesPerMocapFrame, data->ForcePlates[iPlate].ChannelData[iChannel].nFrames);
                // }
                for (int iSample = 0; iSample < data->ForcePlates[iPlate].ChannelData[iChannel].nFrames; iSample++)
                    printf("%3.2f\t", data->ForcePlates[iPlate].ChannelData[iChannel].Values[iSample]);
                printf("\n");
            }
        }

    }
}
void QThreadDAQ::resetClient()
{
    int iSuccess;

    printf("\n\nre-setting Client\n\n.");

    iSuccess = g_pClient->Disconnect();
    if(iSuccess != 0)
        printf("error un-initting Client\n");

    iSuccess = g_pClient->Connect( g_connectParams );
    if(iSuccess != 0)
        printf("error re-initting Client\n");
}

int QThreadDAQ::ConnectClient()
{
    // Release previous server
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect( g_connectParams );
    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.  Error code: %d. Exiting", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // connection succeeded

        void* pResult;
        int nBytes = 0;
        ErrorCode ret = ErrorCode_OK;

        // print server info
        memset( &g_serverDescription, 0, sizeof( g_serverDescription ) );
        ret = g_pClient->GetServerDescription( &g_serverDescription );
        if ( ret != ErrorCode_OK || ! g_serverDescription.HostPresent )
        {
            printf("Unable to connect to server. Host not present. Exiting.");
            return 1;
        }
        printf("\n[SampleClient] Server application info:\n");
        printf("Application: %s (ver. %d.%d.%d.%d)\n", g_serverDescription.szHostApp, g_serverDescription.HostAppVersion[0],
               g_serverDescription.HostAppVersion[1], g_serverDescription.HostAppVersion[2], g_serverDescription.HostAppVersion[3]);
        printf("NatNet Version: %d.%d.%d.%d\n", g_serverDescription.NatNetVersion[0], g_serverDescription.NatNetVersion[1],
               g_serverDescription.NatNetVersion[2], g_serverDescription.NatNetVersion[3]);
        printf("Client IP:%s\n", g_connectParams.localAddress );
        printf("Server IP:%s\n", g_connectParams.serverAddress );
        printf("Server Name:%s\n", g_serverDescription.szHostComputerName);

        // get mocap frame rate
        ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            float fRate = *((float*)pResult);
            printf("Mocap Framerate : %3.2f\n", fRate);
        }
        else
            printf("Error getting frame rate.\n");

        // get # of analog samples per mocap frame of data
        ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            g_analogSamplesPerMocapFrame = *((int*)pResult);
            printf("Analog Samples Per Mocap Frame : %d\n", g_analogSamplesPerMocapFrame);
        }
        else
            printf("Error getting Analog frame rate.\n");
    }

    return ErrorCode_OK;
}

#ifndef _WIN32
char getch()
{
    char buf = 0;
    termios old = { 0 };

    fflush( stdout );

    if ( tcgetattr( 0, &old ) < 0 )
        perror( "tcsetattr()" );

    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;

    if ( tcsetattr( 0, TCSANOW, &old ) < 0 )
        perror( "tcsetattr ICANON" );

    if ( read( 0, &buf, 1 ) < 0 )
        perror( "read()" );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;

    if ( tcsetattr( 0, TCSADRAIN, &old ) < 0 )
        perror( "tcsetattr ~ICANON" );

    //printf( "%c\n", buf );

    return buf;
}
#endif
