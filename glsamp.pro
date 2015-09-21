#-------------------------------------------------
#
# Project created by QtCreator 2015-09-08T08:25:54
#
#-------------------------------------------------

QT       += core gui opengl xml network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = glsamp
TEMPLATE = app

CONFIG += c++11

DEFINES += _USE_MATH_DEFINES

SOURCES += main.cpp\
        mainwindow.cpp \
    glspace.cpp \
    quadmodel.cpp \
    controlmodel.cpp \
    gyrodata.cpp \
    simple_xml.cpp \
    datachart.cpp \
    simplekalmanfilter.cpp

HEADERS  += mainwindow.h \
    virtglobject.h \
    glspace.h \
    quadmodel.h \
    controlmodel.h \
    struct_controls.h \
    gyrodata.h \
    simple_xml.hpp \
    datachart.h \
    global.h \
    simplekalmanfilter.h

FORMS    += mainwindow.ui \
    glspace.ui
