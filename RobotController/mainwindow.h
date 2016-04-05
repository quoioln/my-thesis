#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "Aria.h"
#include "ArNetworking.h"
#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_btnConnectServer_clicked();

    void on_btnEnableMotor_clicked();

    void on_btnStop_clicked();

private:
    Ui::MainWindow *ui;
    ArClientBase* client;
    ArFunctor1C<MainWindow, ArNetPacket *> enableCB;
    ArFunctor1C<MainWindow, ArNetPacket *> getFileCB;
    ArFunctor1C<MainWindow, ArNetPacket *> checkObjectCB;
    ArFunctor1C<MainWindow, ArNetPacket *> recievePoseCB;
    ArFunctor1C<MainWindow, ArNetPacket *> disableCB;

    void enable(ArNetPacket* packet);
    void disable(ArNetPacket* packet);
    void getFile(ArNetPacket* packet);
    void checkObject(ArNetPacket* packet);
    void recievePose(ArNetPacket* packet);
    bool connectServer(char* hostName, int port);
};

#endif // MAINWINDOW_H
