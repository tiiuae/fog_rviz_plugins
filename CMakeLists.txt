cmake_minimum_required(VERSION 3.5)
project(fog_rviz_plugins)

find_package(ament_cmake REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

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
  include/bumper/display.hpp
  src/bumper/display.cpp
  include/bumper/visual.hpp
  src/bumper/visual.cpp
  include/rviz_rendering/objects/mesh_shape.hpp
  src/rviz_rendering/objects/mesh_shape.cpp
  )

ament_target_dependencies(BumperRviz
  fog_msgs
  rclcpp
  rviz_common
  rviz_rendering
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
  TARGETS BumperRviz
  EXPORT BumperRviz
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )

install(
  DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
  )

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY icons
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
