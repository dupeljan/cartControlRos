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

    // Setup all widgets
    ui->setupUi(this);

    pub = std::shared_ptr<RosPublisher>(new RosPublisher());
    sub = std::unique_ptr<RosSubscriber>(new RosSubscriber());

    QPushButton *sendPathButton = new QPushButton();
    sendPathButton->setText("Send path");
    QWidget *mainWidget = new QWidget();
    QGridLayout *gridLayout = new QGridLayout();
    controller = new CartControllerWidget(pub);
    setter = new CartPathSetter(pub);
    getter = new CartPathGetter();

    // Connect widgets to each other
    connect(sendPathButton,SIGNAL(clicked()),setter,SLOT(sendPath()));
    connect(setter,SIGNAL(pathChosen(std::vector<QPointF>)),getter,SLOT(drawAnaliticPath(std::vector<QPointF>)));
    connect(sub.get(),SIGNAL(getPos(QPointF)),getter,SLOT(drawSimulationPos(QPointF)));


    // Add widgets to layout
    gridLayout->addWidget(setter,1,1);
    gridLayout->addWidget(getter,1,2);
    gridLayout->addWidget(controller,1,3);

    gridLayout->addWidget(sendPathButton,2,1);

    mainWidget->setLayout(gridLayout);
    setCentralWidget(mainWidget);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionExit_triggered()
{
    this->close();
}

