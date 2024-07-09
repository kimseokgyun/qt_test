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
#include "qt_test.hpp"
#include "rclcpp/rclcpp.hpp"
#include <QApplication>
#include "mainwindow.h"

int main(int argc, char ** argv)
{
  // Initialize Qt application
  QApplication app(argc, argv);

  // Initialize ROS
  rclcpp::init(argc, argv);
  const rclcpp::NodeOptions options;
  auto node = std::make_shared<qt_test::qt_node>(options);

  // Show the main window
  MainWindow window;
  window.show();

  // Spin the ROS node in a separate thread
  std::thread ros_thread([&]() {
    rclcpp::spin(node);
  });

  // Execute the Qt application
  int result = app.exec();

  // Shutdown ROS
  rclcpp::shutdown();
  ros_thread.join();

  return result;
}