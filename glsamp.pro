#-------------------------------------------------
#
# Project created by QtCreator 2015-09-08T08:25:54
#
#-------------------------------------------------

QT       += core gui opengl xml

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = glsamp
TEMPLATE = app

CONFIG += c++11

DEFINES += _USE_MATH_DEFINES

SOURCES += main.cpp\
        mainwindow.cpp \
    glspace.cpp \
    quadmodel.cpp \
    controlmodel.cpp

HEADERS  += mainwindow.h \
    virtglobject.h \
    glspace.h \
    quadmodel.h \
    controlmodel.h \
    struct_controls.h

FORMS    += mainwindow.ui \
    glspace.ui
