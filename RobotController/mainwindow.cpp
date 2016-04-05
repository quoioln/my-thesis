#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QMessageBox"
#include "Aria.h"
#include "ArNetworking.h"
#include "iostream"
#include "sstream"
using namespace std;

double findObject = 0;
bool requestFile = false;
bool writeImageDone = false;
FILE *file = NULL;
//ArClientBase client;

void MainWindow::enable(ArNetPacket* packet) {
}
void MainWindow::disable(ArNetPacket* packet) {
}
void MainWindow::getFile(ArNetPacket* packet) {
    cout <<"get file is called";
    int ret;
    char fileName[2048];
    ret = packet->bufToUByte2();
    packet->bufToStr(fileName, sizeof(fileName));
    if (ret != 0) {
        printf("Bad return %d on file %s\n", ret, fileName);
        exit(1);
    }
    if (file == NULL) {
        printf("Getting file %s\n", fileName);
        if ((file = ArUtil::fopen("ballDetect.jpg", "w")) == NULL) {
          printf("Can't open fileClientRaw.jpg to dump file into\n");
          exit(2);
        }
    }
    ArTypes::UByte4 numBytes;
    char buf[3200];
    //file should be good here, so just write into it
    numBytes = packet->bufToUByte4();
    if (numBytes == 0) {
        printf("Got all of file %s\n", fileName);
        fclose(file);
        ui->lblImage->setPixmap(QPixmap("ballDetect.jpg"));
//        QMessageBox msg;
//        msg.setIconPixmap(QPixmap("ballDetect.jpg"));
//        //msg.setIcon(QMessageBox::Information);
//        msg.setInformativeText("Co dung la qua qua bong");
//        msg.setWindowTitle("Xac nhan");
//    //    msg.setDetailedText("The details are as follows:");
//        msg.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
//        msg.exec();
//        client->remHandler("getFile", &getFileCB);
//        writeImageDone = true;
        ArUtil::sleep(1000);
    } else {
        printf("Got %d bytes of file %s\n", numBytes, fileName);
        packet->bufToData(buf, numBytes);
        fwrite(buf, 1, numBytes, file);
    }
}
void MainWindow::checkObject(ArNetPacket* packet) {
    findObject = packet->bufToDouble();
//    cout << "reviece: "<<findObject<<endl;
    stringstream content ;
    content << "Find object: " <<findObject;
    ui->lblFindObject->setText(QString(content.str().c_str()));
    if (findObject && requestFile == true) {
        client->addHandler("getFile", &getFileCB);
        client->requestOnceWithString("getFile", "image/ball.jpg");

        requestFile = true;
    }
}
void MainWindow::recievePose(ArNetPacket* packet) {
    double x = packet->bufToDouble();
    double y = packet->bufToDouble();
    stringstream content;
    content<<"x = "<<x<<"  y = "<<y;

    ui->lblPose->setText(content.str().c_str());
//    char* buff = new char[100];
//    packet->bufToStr(buff, 100);
//    ui->lblPose->setText(buff);
}

bool MainWindow::connectServer(char* hostName, int port) {
    return client->blockingConnect(hostName, 7272);
}
/*
ArGlobalFunctor1<ArNetPacket*> enanleCB(&enable);
ArGlobalFunctor1<ArNetPacket*> getFileCB(&getFile);
ArGlobalFunctor1<ArNetPacket*> checkObjectCB(&checkObject);
*/
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow), enableCB(this, &MainWindow::enable), disableCB(this, &MainWindow::disable),
    getFileCB(this, &MainWindow::getFile),
    checkObjectCB(this, &MainWindow::checkObject), recievePoseCB(this, &MainWindow::recievePose),
    client(new ArClientBase())
{
    ui->setupUi(this);
    Aria::init();
}

MainWindow::~MainWindow()
{
    delete ui;
}
/*
void MainWindow::on_pushButton_clicked()
{
    ui->label->setVisible(true);
    ui->label->setPixmap(QPixmap("../../../../../Add.png"));
    QMessageBox msg;
    msg.setIconPixmap(QPixmap("../../../../../ballDetect.jpg"));
    //msg.setIcon(QMessageBox::Information);
    msg.setInformativeText("Co dung la qua qua bong");
    msg.setWindowTitle("MessageBox demo");
//    msg.setDetailedText("The details are as follows:");
    msg.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    msg.exec();
}
*/

void MainWindow::on_btnConnectServer_clicked()
{
    string address = ui->txtAddress->text().toStdString();
//    QMessageBox::information(this, "Thong bao", QString(address.c_str()));
    if (client->blockingConnect(address.c_str(), 7272)) {
        QMessageBox::information(this, "Thong bao", "Ket noi den server thanh cong");
        client->setRobotName(client->getHost());  // include server hostname in log messages
        client->runAsync();
        client->addHandler("handleCheckObjectData", &checkObjectCB);
        client->addHandler("handlePoseRobot", &recievePoseCB);
//        client->addHandler("getFile", &getFileCB);
//        while(client->getRunningWithLock()) {
            client->request("handlePoseRobot", 10);
            client->request("handleCheckObjectData", 10);

//        }

    } else {
        QMessageBox::information(this, "Thong bao", "Ket noi den server that bai");
    }

}

void MainWindow::on_btnEnableMotor_clicked()
{
    client->addHandler("requestEnableMotor", &enableCB);
//    ArNetPacket dataRequest;
//    dataRequest.doubleToBuf(1);
    client->requestOnce("requestEnableMotor");
//    ArNetPacket packet;
//    packet.doubleToBuf(1);
//    packet.byteToBuf(1);
//    pa
//    client->requestOnceByCommand(ArCommands::ENABLE, &packet);
//    client->requestOnce("enableMotor", &packet);
}

void MainWindow::on_btnStop_clicked()
{
    client->addHandler("requestDisableMotor", &disableCB);
    client->requestOnce("requestDisableMotor");
}
