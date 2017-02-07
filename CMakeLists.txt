cmake_minimum_required(VERSION 2.8.3)
project(cpr_rviz_plugin)


find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)

include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(src/)

qt5_wrap_cpp(QT_MOC src/teleop_panel.h)
set(SOURCES src/teleop_panel.cpp ${QT_MOC})

add_library(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
