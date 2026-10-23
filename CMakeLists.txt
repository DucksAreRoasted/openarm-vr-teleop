cmake_minimum_required(VERSION 3.22)
project(openarm_vr_teleop LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# Additional ROS dependency search path.
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble")
if(NOT TARGET yaml-cpp::yaml-cpp AND TARGET yaml-cpp)
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()
if(NOT TARGET kdl_parser::kdl_parser)
  add_library(kdl_parser::kdl_parser INTERFACE IMPORTED)
  target_link_libraries(kdl_parser::kdl_parser INTERFACE ${kdl_parser_LIBRARIES})
  target_include_directories(kdl_parser::kdl_parser INTERFACE ${kdl_parser_INCLUDE_DIRS})
endif()

set(OPENARM_CAN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/openarm_can")
add_library(openarm_can_vr STATIC
  ${OPENARM_CAN_DIR}/src/openarm/can/socket/arm_component.cpp
  ${OPENARM_CAN_DIR}/src/openarm/can/socket/gripper_component.cpp
  ${OPENARM_CAN_DIR}/src/openarm/can/socket/openarm.cpp
  ${OPENARM_CAN_DIR}/src/openarm/canbus/can_device_collection.cpp
  ${OPENARM_CAN_DIR}/src/openarm/canbus/can_socket.cpp
  ${OPENARM_CAN_DIR}/src/openarm/damiao_motor/dm_motor.cpp
  ${OPENARM_CAN_DIR}/src/openarm/damiao_motor/dm_motor_control.cpp
  ${OPENARM_CAN_DIR}/src/openarm/damiao_motor/dm_motor_device.cpp
  ${OPENARM_CAN_DIR}/src/openarm/damiao_motor/dm_motor_device_collection.cpp)
target_include_directories(openarm_can_vr PUBLIC ${OPENARM_CAN_DIR}/include)

execute_process(
  COMMAND ${Python3_EXECUTABLE} -c
          "import pathlib,mujoco; print(pathlib.Path(mujoco.__file__).resolve().parent)"
  OUTPUT_VARIABLE MUJOCO_PYTHON_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE MUJOCO_QUERY_RESULT)
if(NOT MUJOCO_QUERY_RESULT EQUAL 0)
  message(FATAL_ERROR "Python package 'mujoco' is required; run ./install.sh first")
endif()
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h HINTS "${MUJOCO_PYTHON_DIR}/include" REQUIRED)
find_library(MUJOCO_LIBRARY NAMES mujoco libmujoco.so.3.12.0 libmujoco.so.3.11.0
             HINTS "${MUJOCO_PYTHON_DIR}" REQUIRED)
find_package(glfw3 QUIET)

add_library(vr_control_core STATIC
  src/controller/dynamics.cpp src/controller/control.cpp
  src/controller/reference_smoother.cpp src/controller/human_mimic_kinematics.cpp
  src/controller/virtual_arm_skeleton.cpp src/controller/human_mimic_ik_solver.cpp
  src/controller/joint_reference_generator.cpp src/controller/human_mimic_controller.cpp
  src/controller/unitree_ipopt_client.cpp
  src/openarm_port/openarm_init.cpp src/openarm_port/joint_mapper.cpp
  src/safety/safety_filter.cpp src/io/teleop_io.cpp src/io/quest_pose_receiver.cpp
  src/io/teleop_trajectory_logger.cpp
  src/hardware/teleop_hardware.cpp src/hardware/mujoco_teleop_hardware.cpp)
target_include_directories(vr_control_core PUBLIC src ${MUJOCO_INCLUDE_DIR})
target_link_libraries(vr_control_core PUBLIC openarm_can_vr Eigen3::Eigen
  ${orocos_kdl_LIBRARIES} kdl_parser::kdl_parser urdfdom::urdfdom_model
  yaml-cpp::yaml-cpp ${MUJOCO_LIBRARY})
if(TARGET glfw)
  target_link_libraries(vr_control_core PUBLIC glfw)
elseif(TARGET glfw3)
  target_link_libraries(vr_control_core PUBLIC glfw3)
else()
  target_link_libraries(vr_control_core PUBLIC glfw GL)
endif()

add_executable(quest_cartesian_control control/openarm_quest_cartesian_control.cpp)
target_link_libraries(quest_cartesian_control PRIVATE vr_control_core pthread)

add_executable(openarm_zero_g control/openarm_zero_g.cpp)
target_link_libraries(openarm_zero_g PRIVATE vr_control_core pthread)

add_executable(openarm_teaching_replay control/openarm_teaching_replay.cpp)
target_link_libraries(openarm_teaching_replay PRIVATE vr_control_core pthread)

add_executable(openarm_gravity_calib control/openarm_gravity_calib.cpp)
target_link_libraries(openarm_gravity_calib PRIVATE vr_control_core pthread)

add_executable(openarm_read_motor_params control/openarm_read_motor_params.cpp)
target_link_libraries(openarm_read_motor_params PRIVATE openarm_can_vr pthread)

# Offline calibration evaluator: never opens CAN or enables motors.
add_executable(openarm_gravity_model control/openarm_gravity_model.cpp)
target_link_libraries(openarm_gravity_model PRIVATE vr_control_core pthread)

# Hardware-free human_mimic clutch contract. Compiles the real controller sources
# without CAN, MuJoCo, or the rest of vr_control_core.
add_library(human_mimic_test_core STATIC
  src/controller/dynamics.cpp
  src/controller/human_mimic_kinematics.cpp
  src/controller/virtual_arm_skeleton.cpp
  src/controller/human_mimic_ik_solver.cpp
  src/controller/joint_reference_generator.cpp
  src/controller/reference_smoother.cpp
  src/controller/human_mimic_controller.cpp
  src/controller/unitree_ipopt_client.cpp)
target_include_directories(human_mimic_test_core PUBLIC src)
target_link_libraries(human_mimic_test_core PUBLIC Eigen3::Eigen
  ${orocos_kdl_LIBRARIES} kdl_parser::kdl_parser urdfdom::urdfdom_model)

add_executable(human_mimic_clutch_test tests/human_mimic_clutch_test.cpp)
target_include_directories(human_mimic_clutch_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/tests
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/openarm_can/include)
target_link_libraries(human_mimic_clutch_test PRIVATE human_mimic_test_core pthread)

add_executable(unitree_style_test tests/unitree_style_test.cpp)
target_include_directories(unitree_style_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/openarm_can/include)
target_link_libraries(unitree_style_test PRIVATE human_mimic_test_core pthread)

add_executable(teleop_gravity_test tests/teleop_gravity_test.cpp)
target_link_libraries(teleop_gravity_test PRIVATE vr_control_core pthread)

add_executable(trajectory_spline_test tests/trajectory_spline_test.cpp)
target_include_directories(trajectory_spline_test PRIVATE src)
