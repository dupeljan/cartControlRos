#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "commonheader.h"

#include <QMainWindow>
#include "rospublisher.h"
#include "rossubscriber.h"
#include "cartcontrollerwidget.h"
#include "cartpathsetter.h"
#include "cartpathgetter.h"

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
    void on_actionExit_triggered();

private:
    Ui::MainWindow *ui;
    CartControllerWidget *controller;
    CartPathSetter *setter;
    CartPathGetter *getter;
    std::shared_ptr<RosPublisher> pub;
    std::unique_ptr<RosSubscriber> sub;

};

#endif // MAINWINDOW_H
