#-------------------------------------------------
#
# Project created by QtCreator 2016-04-03T15:15:31
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RobotControler
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    controller.cpp

HEADERS  += mainwindow.h \
    controller.h

FORMS    += mainwindow.ui

INCLUDEPATH +=/usr/local/Aria/include \
             /usr/local/Aria/ArNetworking/include
SOURCES += main.cpp
            #/usr/local/Aria/src/Aria.cpp
LIBS += /usr/local/Aria/lib/libAria.so \
       -lpthread -ldl -lrt
#unix: CONFIG += link_pkgconfig
#unix: PKGCONFIG += -Bstatic -lstdc++ -Xlinker -Bdynamic -lpthread -ldl -lrt
unix: QMAKE_CXX += -Xlinker -Bdynamic -lpthread -ldl -lrt -Xlinker -Bstatic -lstdc++ -Xlinker -Bdynamic
