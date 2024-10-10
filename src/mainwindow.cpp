#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "rclcpp/rclcpp.hpp"



MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , node (std::make_shared<qt_test::qt_node>(rclcpp::NodeOptions()))
{
    ui->setupUi(this);

    connect(&timer, SIGNAL(timeout()), this, SLOT(onTimer()));
    
    timer.start(100); //10

    
}

MainWindow::~MainWindow()
{
    rclcpp::shutdown(); // 프로그램 종료 시 ROS 종료
    delete ui;
}




void MainWindow::onTimer(){

        c_point T1 = c_point(node->DOCK_X, 0);
        c_point T2 = c_point(node->DOCK_X -2*node->DOCK_SIZE_X[1], node->DOCK_Y + 1*node->DOCK_SIZE_Y[1]);
        c_point T3 = c_point(node->DOCK_X -2*node->DOCK_SIZE_X[1], node->DOCK_Y - 1*node->DOCK_SIZE_Y[1]);

    node->publishKFrameMarker(node->generateSampleKFrame(T1,T2,T3), 0, "current_lidar", 1.0, 0.0, 0.0);
    node->publishKFrameMarker(node->generateVKFrame(), 0, "sample_point", 0.0, 1.0, 0.0);

}





void MainWindow::on_pushButton_clicked()
{
    node->icp_go();
}
void MainWindow::on_lineEdit_textChanged(const QString &arg1)
{
    dock_size_x = arg1.toDouble();
    node->DOCK_SIZE_X[0] = -dock_size_x;
    node->DOCK_SIZE_X[1] = dock_size_x;
}


void MainWindow::on_lineEdit_2_textChanged(const QString &arg1)
{
    dock_size_y = arg1.toDouble();
    node->DOCK_SIZE_Y[0] = -dock_size_y;   
    node->DOCK_SIZE_Y[1] = dock_size_y;

}


void MainWindow::on_lineEdit_3_textChanged(const QString &arg1)
{
    dock_x = arg1.toDouble();
    node->DOCK_X = dock_x;
}


void MainWindow::on_lineEdit_4_textChanged(const QString &arg1)
{
    dock_y = arg1.toDouble();
    node->DOCK_Y = dock_y;
}


void MainWindow::on_lineEdit_5_textChanged(const QString &arg1)
{
    icp_cost_threshold = arg1.toDouble();
    node->ICP_COST_THRESHOLD = icp_cost_threshold;
}


void MainWindow::on_lineEdit_6_textChanged(const QString &arg1)
{
    icp_correspondence_threshold = arg1.toInt();
    node->ICP_CORRESPONDENCE_THRESHOLD = icp_correspondence_threshold;
}


void MainWindow::on_lineEdit_7_textChanged(const QString &arg1)
{
    node->DOCK_ANGLE = arg1.toDouble();
}

void MainWindow::on_lineEdit_8_textChanged(const QString &arg1)
{
    node->sample_noise = arg1.toDouble();
}


