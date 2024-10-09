cmake_minimum_required(VERSION 3.8)
project(qt_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core Xml REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Enable automatic MOC generation
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# Specify UI file
set(UI_FILES
  src/mainwindow.ui
)


include_directories(
  include
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Xml_INCLUDE_DIRS}
)

# Header files
set(HEADERS
  include/mainwindow.h
  include/qt_test.hpp
  include/nanoflann.hpp
)

# Source files
set(SOURCES
  src/main.cpp
  src/mainwindow.cpp
  src/qt_test.cpp
  ${UI_FILES}

)

set(executable_name qt_test)

# Add executable
add_executable(${executable_name} ${SOURCES} ${HEADERS})

# set(library_name ${executable_name}_core)

# Set dependencies
set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  visualization_msgs 
)

# add_library(${library_name} SHARED
#   src/main.cpp
#   src/mainwindow.cpp
#   src/qt_test.cpp
# )

ament_target_dependencies(${executable_name}
  ${dependencies}
)

# Eigen3 라이브러리를 실행 파일과 라이브러리에 링크
target_link_libraries(${executable_name} Eigen3::Eigen)
target_link_libraries(${library_name} Eigen3::Eigen)

target_link_libraries(${executable_name} Qt5::Widgets Qt5::Core Qt5::Xml)

ament_target_dependencies(${executable_name}
  ${dependencies}
)


#install(
#  DIRECTORY launch
#  DESTINATION share/${PROJECT_NAME}
#)

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
