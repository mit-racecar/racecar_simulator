cmake_minimum_required(VERSION 3.8)
project(racecar_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)
file(GLOB SRC_FILES src/*.cpp)
add_library(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${LIBS})
set(LIBS ${LIBS} ${PROJECT_NAME})

# Install the library to CMAKE_INSTALL_PREFIX
# which defaults to /usr/local
install(TARGETS ${PROJECT_NAME}
        DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include)

add_executable(distance_transform_visualizer node/distance_transform_visualizer.cpp)
ament_target_dependencies(distance_transform_visualizer rclcpp nav_msgs)
target_link_libraries(distance_transform_visualizer ${LIBS})

add_executable(simulate node/simulate.cpp)
ament_target_dependencies(simulate 
  rclcpp 
  tf2
  tf2_ros 
  tf2_geometry_msgs 
  ackermann_msgs
  nav_msgs 
  sensor_msgs 
  geometry_msgs
)
target_link_libraries(simulate ${LIBS})

install(TARGETS
  distance_transform_visualizer
  simulate
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY 
  launch
  maps  
  config
  DESTINATION share/${PROJECT_NAME})

install(
  FILES 
  racecar.xacro    
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
