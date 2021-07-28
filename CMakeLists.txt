cmake_minimum_required(VERSION 3.5)
project(fog_rviz_plugins)

find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(fog_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)

set(QT_LIBRARIES Qt5::Widgets)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

link_directories(${catkin_LIBRARY_DIRS})

add_definitions(-DQT_NO_KEYWORDS)
# ignore c++17 deprecation warnings for the register storage class specifier because of OGRE
add_definitions("-Wno-register")

include_directories(
  include
  ${Qt5Widgets_INCLUDE_DIRS}
  ${rviz_ogre_vendor}
  )


## --------------------------------------------------------------
## |                           BUMPER                           |
## --------------------------------------------------------------

add_library(BumperRviz SHARED
  include/bumper/display.h
  include/bumper/visual.h
  src/bumper/display.cpp
  src/bumper/visual.cpp
  )

ament_target_dependencies(BumperRviz
  fog_msgs
  rclcpp
  rviz_common
  rviz_rendering
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
  )

target_link_libraries(BumperRviz 
  ${QT_LIBRARIES}
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  )

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )

install(
  DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
  )

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
