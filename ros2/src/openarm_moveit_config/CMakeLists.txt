cmake_minimum_required(VERSION 3.8)
project(openarm_moveit_config)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)

set(OPENARM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
set(OPENARM_CAN_DIR "${OPENARM_ROOT}/third_party/openarm_can")
set(OPENARM_URDF_PATH "${OPENARM_ROOT}/assets/v1.urdf")

add_executable(real_arm
  scripts/real_arm.cpp
  ${OPENARM_ROOT}/src/controller/dynamics.cpp
  ${OPENARM_CAN_DIR}/src/openarm/can/socket/arm_component.cpp
  ${OPENARM_CAN_DIR}/src/openarm/can/socket/gripper_component.cpp
  ${OPENARM_CAN_DIR}/src/openarm/can/socket/openarm.cpp
  ${OPENARM_CAN_DIR}/src/openarm/canbus/can_device_collection.cpp
  ${OPENARM_CAN_DIR}/src/openarm/canbus/can_socket.cpp
  ${OPENARM_CAN_DIR}/src/openarm/damiao_motor/dm_motor.cpp
  ${OPENARM_CAN_DIR}/src/openarm/damiao_motor/dm_motor_control.cpp
  ${OPENARM_CAN_DIR}/src/openarm/damiao_motor/dm_motor_device.cpp
  ${OPENARM_CAN_DIR}/src/openarm/damiao_motor/dm_motor_device_collection.cpp)
target_include_directories(real_arm PRIVATE
  ${OPENARM_CAN_DIR}/include
  ${OPENARM_ROOT}/src
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS})
target_compile_definitions(real_arm PRIVATE
  OPENARM_URDF_PATH="${OPENARM_URDF_PATH}"
  OPENARM_ROOT="${OPENARM_ROOT}")
target_link_libraries(real_arm
  Eigen3::Eigen
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  ${urdfdom_LIBRARIES})
if(TARGET kdl_parser::kdl_parser)
  target_link_libraries(real_arm kdl_parser::kdl_parser)
endif()
ament_target_dependencies(real_arm rclcpp rclcpp_action control_msgs sensor_msgs std_msgs std_srvs)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES config/sensors_3d.yaml DESTINATION share/${PROJECT_NAME}/config)
install(
  DIRECTORY ${OPENARM_ROOT}/models/OpenArm_V1/meshes
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS real_arm DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/fake_arm.py DESTINATION lib/${PROJECT_NAME})

ament_package()
