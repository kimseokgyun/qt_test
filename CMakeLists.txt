cmake_minimum_required(VERSION 3.8)
project(qt_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
# Qt 패키지 찾기
find_package(Qt5 COMPONENTS Widgets Core Xml REQUIRED)
# Qt MOC
set(CMAKE_AUTOMOC ON)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Xml_INCLUDE_DIRS}
)
# Header files
# set(HEADERS
#   include/qt_test/mainwindow.h
# )

set(executable_name qt_test)

add_executable(${executable_name} 
  ${HEADERS}
  src/main.cpp
  src/mainwindow.cpp
)
set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
)

add_library(${library_name} SHARED
  src/main.cpp
  src/mainwindow.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name} Qt5::Widgets Qt5::Core Qt5::Xml )

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
