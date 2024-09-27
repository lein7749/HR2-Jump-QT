QT       += core gui network printsupport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    all_trj.cpp \
    data_init.cpp \
    deal_msg.cpp \
    fan_MPC.cpp \
    filter.cpp \
    main.cpp \
    param_init.cpp \
    qmythread.cpp \
    utilities.cpp \
    widget.cpp

HEADERS += \
    all_trj.h \
    communicationprotocol.h \
    data_init.h \
    deal_msg.h \
    fan_MPC.h \
    filter.h \
    # include/NatNetCAPI.h \
    # include/NatNetClient.h \
    # include/NatNetRepeater.h \
    # include/NatNetRequests.h \
    # include/NatNetTypes.h \
    param_init.h \
    qmythread.h \
    type.h \
    utilities.h \
    widget.h

FORMS += \
    widget.ui

INCLUDEPATH += $$PWD xlsx
include(xlsx/qtxlsx.pri)

INCLUDEPATH += $$PWD\qpOASES\include
INCLUDEPATH += $$PWD\NatNet\include

LIBS += $$PWD\NatNet\lib\x64\NatNetLib.lib
LIBS += $$PWD\qpOASES\lib\qpOASES.lib

LIBS += -L$$PWD\NatNet\lib\x64\



# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


