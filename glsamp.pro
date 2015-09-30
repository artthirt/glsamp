#-------------------------------------------------
#
# Project created by QtCreator 2015-09-08T08:25:54
#
#-------------------------------------------------

QT       += core gui opengl xml network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG(debug, debug|release){
	DST_DIR=$$OUT_PWD/debug
}else{
	DST_DIR=$$OUT_PWD/release
}

TARGET = glsamp
TEMPLATE = app

CONFIG += c++11

DEFINES += _USE_MATH_DEFINES

INCLUDEPATH += . \
				$$PWD/submodules/struct_controls

SOURCES += main.cpp\
        mainwindow.cpp \
    glspace.cpp \
    quadmodel.cpp \
    controlmodel.cpp \
    gyrodata.cpp \
    simple_xml.cpp \
    datachart.cpp \
    simplekalmanfilter.cpp \
    writelog.cpp

HEADERS  += mainwindow.h \
    virtglobject.h \
    glspace.h \
    quadmodel.h \
    controlmodel.h \
    gyrodata.h \
    simple_xml.hpp \
    datachart.h \
    global.h \
    simplekalmanfilter.h \
	writelog.h \
	submodules/struct_controls/struct_controls.h

FORMS    += mainwindow.ui \
    glspace.ui

UI_DIR = $$DST_DIR/tmp/ui
OBJECTS_DIR = $$DST_DIR/obj
MOC_DIR = $$DST_DIR/moc
RCC_DIR = $$DST_DIR/rcc
