// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <QApplication>
#include "mainwindow.h"

// int main(int argc, char ** argv)
// {
//   // Initialize Qt application
//   QApplication app(argc, argv);

//   // Initialize ROS
//   rclcpp::init(argc, argv);
//   // const rclcpp::NodeOptions options;
//   // auto node = std::make_shared<qt_test::qt_node>(options);

//   // Show the main window
//   MainWindow window;
//   window.show();

//   // // Spin the ROS node in a separate thread
//   // std::thread ros_thread([&]() {
//   //   rclcpp::spin(node);
//   // });

//   // Execute the Qt application
//   int result = app.exec();

//   // // Shutdown ROS
//   // rclcpp::shutdown();
//   // ros_thread.join();

//   return result;
// }

int main(int argc, char ** argv)
{
    // Qt 애플리케이션 초기화
    QApplication app(argc, argv);

    // ROS 초기화
    rclcpp::init(argc, argv);

    // MainWindow 객체 생성 및 표시
    MainWindow window;
    window.show();

    // ROS 노드를 별도의 스레드에서 실행
    std::thread ros_thread([]() {
        rclcpp::spin(std::make_shared<rclcpp::Node>("ros_thread"));
        rclcpp::shutdown();
    });

    // Qt 애플리케이션 실행
    int result = app.exec();

    // ROS 스레드 종료 대기
    if (ros_thread.joinable()) {
        ros_thread.join();
    }

    return result;
}