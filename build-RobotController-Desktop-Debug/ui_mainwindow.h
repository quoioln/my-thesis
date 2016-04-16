/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.2.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGroupBox *groupButton;
    QPushButton *btnEnableMotor;
    QPushButton *btnDicrectMotion;
    QPushButton *btnStop;
    QLineEdit *txtAddress;
    QPushButton *btnConnectServer;
    QLabel *lblFindObject;
    QLabel *lblPose;
    QLabel *lblImage;
    QPushButton *btnTrue;
    QPushButton *btnFalse;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(793, 526);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        groupButton = new QGroupBox(centralWidget);
        groupButton->setObjectName(QStringLiteral("groupButton"));
        groupButton->setGeometry(QRect(40, 20, 291, 211));
        btnEnableMotor = new QPushButton(groupButton);
        btnEnableMotor->setObjectName(QStringLiteral("btnEnableMotor"));
        btnEnableMotor->setGeometry(QRect(30, 30, 111, 27));
        btnDicrectMotion = new QPushButton(groupButton);
        btnDicrectMotion->setObjectName(QStringLiteral("btnDicrectMotion"));
        btnDicrectMotion->setGeometry(QRect(30, 90, 111, 27));
        btnStop = new QPushButton(groupButton);
        btnStop->setObjectName(QStringLiteral("btnStop"));
        btnStop->setGeometry(QRect(30, 140, 99, 27));
        txtAddress = new QLineEdit(groupButton);
        txtAddress->setObjectName(QStringLiteral("txtAddress"));
        txtAddress->setGeometry(QRect(20, 180, 113, 27));
        btnConnectServer = new QPushButton(groupButton);
        btnConnectServer->setObjectName(QStringLiteral("btnConnectServer"));
        btnConnectServer->setGeometry(QRect(150, 170, 99, 27));
        lblFindObject = new QLabel(centralWidget);
        lblFindObject->setObjectName(QStringLiteral("lblFindObject"));
        lblFindObject->setGeometry(QRect(140, 320, 231, 17));
        lblPose = new QLabel(centralWidget);
        lblPose->setObjectName(QStringLiteral("lblPose"));
        lblPose->setGeometry(QRect(20, 280, 241, 20));
        lblImage = new QLabel(centralWidget);
        lblImage->setObjectName(QStringLiteral("lblImage"));
        lblImage->setGeometry(QRect(310, 100, 391, 371));
        btnTrue = new QPushButton(centralWidget);
        btnTrue->setObjectName(QStringLiteral("btnTrue"));
        btnTrue->setEnabled(false);
        btnTrue->setGeometry(QRect(288, 490, 131, 27));
        btnFalse = new QPushButton(centralWidget);
        btnFalse->setObjectName(QStringLiteral("btnFalse"));
        btnFalse->setEnabled(false);
        btnFalse->setGeometry(QRect(460, 490, 111, 27));
        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        groupButton->setTitle(QApplication::translate("MainWindow", "B\341\272\243ng \304\221i\341\273\201u khi\341\273\203n", 0));
        btnEnableMotor->setText(QApplication::translate("MainWindow", "enable motor", 0));
        btnDicrectMotion->setText(QApplication::translate("MainWindow", "dicrect motion", 0));
        btnStop->setText(QApplication::translate("MainWindow", "Stop", 0));
        btnConnectServer->setText(QApplication::translate("MainWindow", "connect server", 0));
        lblFindObject->setText(QApplication::translate("MainWindow", "find object", 0));
        lblPose->setText(QApplication::translate("MainWindow", "Vi tri:", 0));
        lblImage->setText(QApplication::translate("MainWindow", "Anh", 0));
        btnTrue->setText(QApplication::translate("MainWindow", "\304\220\303\272ng \304\221\341\273\221i t\306\260\341\273\243ng", 0));
        btnFalse->setText(QApplication::translate("MainWindow", "Sai \304\221\341\273\221i t\306\260\341\273\243ng", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
