#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "cartcontrollerwidget.h"
#include <QMouseEvent>
#include <QGridLayout>
#include <iostream>
#include <string>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QWidget *mainWidget = new QWidget();
    QGridLayout *gridLayout = new QGridLayout();
    auto controller = new CartControllerWidget();
    //controller->setLayout(ui->centralWidget->layout());
    gridLayout->addWidget(new CartControllerWidget(),2,2);

    mainWidget->setLayout(gridLayout);
    setCentralWidget(mainWidget);
    //ui->centralWidget->layout()->addWidget(new CartControllerWidget());



   // connect(ui->graphicsView,SIGNAL(mousePress(QMouseEvent*)),SLOT(plotClicked(QMouseEvent *)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionExit_triggered()
{
    this->close();
}

