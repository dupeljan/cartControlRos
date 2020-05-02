#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "rospublisher.h"
#include "cartcontrollerwidget.h"

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

};

#endif // MAINWINDOW_H
