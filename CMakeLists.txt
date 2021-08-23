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

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

## --------------------------------------------------------------
## |                           BUMPER                           |
## --------------------------------------------------------------

set(fog_rviz_plugins_headers_to_moc
  include/bumper/display.hpp
  )

foreach(header "${fog_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(fog_rviz_plugins_moc_files "${header}")
endforeach()

set(fog_rviz_plugins_source_files
  src/bumper/display.cpp
  )

add_library(fog_rviz_plugins SHARED
  ${fog_rviz_plugins_moc_files}
  ${fog_rviz_plugins_source_files}
  include/bumper/visual.hpp
  include/rviz_rendering/objects/mesh_shape.hpp
  src/bumper/visual.cpp
  src/rviz_rendering/objects/mesh_shape.cpp
  )

target_include_directories(fog_rviz_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(fog_rviz_plugins PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  )

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(fog_rviz_plugins PRIVATE "FOG_RVIZ_PLUGINS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(fog_rviz_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_export_include_directories(include)
ament_export_targets(fog_rviz_plugins HAS_LIBRARY_TARGET)
ament_target_dependencies(fog_rviz_plugins
  PUBLIC
  fog_msgs
  rclcpp
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  )

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  TARGETS fog_rviz_plugins
  EXPORT fog_rviz_plugins
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
