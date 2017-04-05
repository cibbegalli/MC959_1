QT += core

PATH = ../V-REP_PRO_EDU_V3_3_2_64_Linux/programming

INCLUDEPATH += $${PATH}/remoteApi
INCLUDEPATH += $${PATH}/include
INCLUDEPATH += $${PATH}/common

DEFINES += "MAX_EXT_API_CONNECTIONS=255"
DEFINES += "NON_MATLAB_PARSING"
QMAKE_LFLAGS += -pthread

CONFIG += c++11

TARGET = VREPClient
CONFIG += console
CONFIG -= app_bundle


TEMPLATE = app

SOURCES += $${PATH}/remoteApi/extApi.c \
    Main.cpp \
    Robot.cpp \
    Simulator.cpp
SOURCES += $${PATH}/remoteApi/extApiPlatform.c
SOURCES += $${PATH}/common/v_repLib.cpp

HEADERS += $${PATH}/include/v_repLib.h \
    Robot.h \
    Simulator.h

SOURCES +=

HEADERS +=
