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
#include "qt_test.hpp"

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
    int icp_max_feature_num;
    double icp_cost_threshold;
    int icp_correspondence_threshold;
    double dock_size_x;
    double dock_size_y;
    double dock_x;
    double dock_y;

public slots:
    void onTimer();



private slots:

    void on_pushButton_clicked();

    void on_lineEdit_textChanged(const QString &arg1);

    void on_lineEdit_2_textChanged(const QString &arg1);

    void on_lineEdit_3_textChanged(const QString &arg1);

    void on_lineEdit_4_textChanged(const QString &arg1);

    void on_lineEdit_5_textChanged(const QString &arg1);

    void on_lineEdit_6_textChanged(const QString &arg1);

    void on_lineEdit_7_textChanged(const QString &arg1);
        
    void on_lineEdit_8_textChanged(const QString &arg1);
    
private:
    //rclcpp::Node::SharedPtr node;             // ROS2 노드
    std::shared_ptr<qt_test::qt_node> node; // ROS2 노드
    Ui::MainWindow *ui;
    QTimer timer;
};
#endif // MAINWINDOW_H
