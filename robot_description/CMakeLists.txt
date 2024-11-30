cmake_minimum_required(VERSION 3.5)
project(robot_description)

if($ENV{ROS_VERSION} EQUAL 1)
  # In ROS1 compilation environment
  message("Currently in ROS1 compilation environment")

  find_package(catkin REQUIRED COMPONENTS roscpp)
  
  catkin_package(CATKIN_DEPENDS)

  # Install directories 'meshes' and 'urdf' to the package share directory
  install(DIRECTORY airbot DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
elseif($ENV{ROS_VERSION} EQUAL 2)
  # In ROS2 compilation environment
  message("Currently in ROS2 compilation environment")

  find_package(ament_cmake REQUIRED)

  install(DIRECTORY airbot DESTINATION share/${PROJECT_NAME})

  ament_package()
endif()
