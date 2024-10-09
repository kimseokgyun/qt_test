#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QDebug>
#include <QLineEdit>
#include <rclcpp/rclcpp.hpp>  // 추가: rclcpp 헤더 파일 포함
#include <iostream>
#include <stdio.h>
#include <fcntl.h>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


inline void SetLEColor(QLineEdit *le, QString color){
    le->setStyleSheet("QLineEdit{background-color:"+color+"}");
}


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


public slots:
    void onTimer();



private slots:


    void on_pushButton_clicked();

private:
    rclcpp::Node::SharedPtr node;             // ROS2 노드
    Ui::MainWindow *ui;
    QTimer timer;
};
#endif // MAINWINDOW_H
