cmake_minimum_required(VERSION 2.8.3)
project(drone_control_fb_slam)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        )

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp
)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/drone_bebop_control.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})


