#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "rclcpp/rclcpp.hpp"
#include "qt_test.hpp"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , node (std::make_shared<qt_test::qt_node>(rclcpp::NodeOptions()))
{
    ui->setupUi(this);

    connect(&timer, SIGNAL(timeout()), this, SLOT(onTimer()));
    
    timer.start(10); //10

    
}

MainWindow::~MainWindow()
{
    rclcpp::shutdown(); // 프로그램 종료 시 ROS 종료
    delete ui;
}




void MainWindow::onTimer(){
    // RCLCPP_INFO(node->get_logger(), "onTimer");

}





void MainWindow::on_pushButton_clicked()
{
    rclcpp::spin(node->get_node_base_interface());
}

