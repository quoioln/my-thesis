#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QMessageBox"
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //QMessageBox::information(this, "Thong bao", "Hello  world");
}

MainWindow::~MainWindow()
{
    delete ui;
}
