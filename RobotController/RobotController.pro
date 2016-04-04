#-------------------------------------------------
#
# Project created by QtCreator 2016-04-03T16:29:55
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RobotController
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    controller.cpp

HEADERS  += mainwindow.h \
    controller.h

FORMS    += mainwindow.ui

INCLUDEPATH +=/usr/local/Aria/include \
             /usr/local/Aria/ArNetworking/include

LIBS += /usr/local/Aria/lib/*.so \
       -lpthread -ldl -lrt
#unix: CONFIG += link_pkgconfig
#unix: PKGCONFIG += -Bstatic -lstdc++ -Xlinker -Bdynamic -lpthread -ldl -lrt
unix: QMAKE_CXX += -Xlinker -Bdynamic -lpthread -ldl -lrt -Xlinker -Bstatic -lstdc++ -Xlinker -Bdynamic
