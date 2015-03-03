#-------------------------------------------------
#
# Project created by QtCreator 2013-04-09T18:05:58
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = simu_cncdriver
TEMPLATE = app

DEFINES += ON_QT_PLATFORM

INCLUDEPATH += ../../../Prog/Project/MainProject/ \
               ../../../Prog/Project/MainProject/func/

SOURCES += main.cpp\
        mainw.cpp \
    hw_wrapper.cpp \
    ../../../Prog/Project/MainProject/func/events_ui.c \
    ../../../Prog/Project/MainProject/mainapp.c \
    ../../../Prog/Project/MainProject/func/motion_core.c \
    ../../../Prog/Project/MainProject/func/frontend.c

HEADERS  += mainw.h \
    stm32f10x.h \
    ../../../Prog/Project/MainProject/typedefs.h \
    hw_stuff.h \
    ../../../Prog/Project/MainProject/func/events_ui.h \
    ../../../Prog/Project/MainProject/func/cnc_defs.h \
    ../../../Prog/Project/MainProject/func/motion_core.h \
    ../../../Prog/Project/MainProject/func/motion_core_internals.h \
    ../../../Prog/Project/MainProject/func/frontend.h \
    ../../../Prog/Project/MainProject/func/frontend_internals.h \
    ../../../Prog/Project/MainProject/func/comm_fe.h

FORMS    += mainw.ui
