#-------------------------------------------------
#
# Project created by QtCreator 2016-02-18T19:11:04
#
#-------------------------------------------------

QT       += core widgets

QT       -= gui

TARGET = ROSRobobulls
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

QMAKE_CXXFLAGS += -std=c++0x
unix|win32: LIBS += -lprotobuf

SOURCES += main.cpp \
    visioncomm.cpp \
    include/messages_robocup_ssl_detection.pb.cc \
    include/messages_robocup_ssl_geometry.pb.cc \
    include/messages_robocup_ssl_refbox_log.pb.cc \
    include/messages_robocup_ssl_wrapper.pb.cc \
    include/netraw.cpp \
    include/robocup_ssl_client.cpp \
    utilities/point.cpp

HEADERS += \
    visioncomm.h \
    include/messages_robocup_ssl_detection.pb.h \
    include/messages_robocup_ssl_geometry.pb.h \
    include/messages_robocup_ssl_refbox_log.pb.h \
    include/messages_robocup_ssl_wrapper.pb.h \
    include/netraw.h \
    include/robocup_ssl_client.h \
    include/config/communication.h \
    include/config/simulated.h \
    utilities/point.h \
    include/config/team.h

OTHER_FILES += \
    include/messages_robocup_ssl_detection.proto \
    include/messages_robocup_ssl_geometry.proto \
    include/messages_robocup_ssl_refbox_log.proto \
    include/messages_robocup_ssl_wrapper.proto
