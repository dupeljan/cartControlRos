#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMouseEvent>
#include <QGridLayout>
#include <QPushButton>
#include <iostream>
#include <string>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QPushButton *sendPathButton = new QPushButton();
    QWidget *mainWidget = new QWidget();
    QGridLayout *gridLayout = new QGridLayout();
    controller = new CartControllerWidget();
    setter = new CartPathSetter();

    // Connecing
    connect(sendPathButton,SIGNAL(clicked()),setter,SLOT(sendPath()));
    //controller->setLayout(ui->centralWidget->layout());
    // Add widgets to layout
    gridLayout->addWidget(setter,1,1);
    gridLayout->addWidget(controller,1,2);
    gridLayout->addWidget(sendPathButton,2,1);

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

