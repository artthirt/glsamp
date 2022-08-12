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

CONFIG += c++11 console

DEFINES += _USE_MATH_DEFINES

VERSION_BUILD = $$system("git rev-parse HEAD")
isEmpty(VERSION_BUILD){
	VERSION_BUILD = 1
}

VERBUILDSTR = '$${VERSION_BUILD}'
DEFINES += VERBUILD=\\\"$$VERBUILDSTR\\\"

SOURCES += main.cpp\
        mainwindow.cpp \
		simple_xml.cpp

HEADERS  += mainwindow.h \
    simple_xml.hpp \
	global.h

FORMS    += mainwindow.ui \

UI_DIR = $$DST_DIR/tmp/ui
OBJECTS_DIR = $$DST_DIR/obj
MOC_DIR = $$DST_DIR/moc
RCC_DIR = $$DST_DIR/rcc

win32{
    LIBS += -lopengl32
}

include(about/about.pri)
include(controlmodel/controlmodel.pri)
include(glutils/glutils.pri)
include(graphics/graphics.pri)
include(log/log.pri)
include(quadmodel/quadmodel.pri)
include(sensors/sensors.pri)
include(utils/utils.pri)
include(wnd/wnd.pri)
include(submodules/struct_controls/struct_controls.pri)

RESOURCES += \
    resources.qrc
