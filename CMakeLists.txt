cmake_minimum_required(VERSION 3.16)
project(ros2_control_kindyn LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
  std_srvs
  roboy_middleware_msgs
  roboy_control_msgs
  ros2_common_utilities
  tf2_ros
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
  roboy_simulation_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# idyntree
find_package(iDynTree 0.11.1 REQUIRED)
include_directories(${iDynTree_INCLUDE_DIRS})
# # Eigen
# find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIRS})

## COMPILE

add_executable(send_cmd test/send_cmd.cpp)
ament_target_dependencies(
  send_cmd PUBLIC
  rclcpp
  sensor_msgs
  std_msgs
)


add_library(
  ros2_control_kindyn
  SHARED
  utilities/EigenExtension.cpp
  utilities/kinematics.cpp
  hardware/robot.cpp
  controller/cableLengthController.cpp
  controller/cardsflow_command_interface.cpp
  controller/cardsflow_state_interface.cpp
  robots/upper_body.cpp
)

target_compile_features(ros2_control_kindyn PUBLIC cxx_std_17)
target_include_directories(ros2_control_kindyn PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/utilities/include>
$<INSTALL_INTERFACE:include/ros2_control_kindyn>
)
ament_target_dependencies(
  ros2_control_kindyn PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
)
target_include_directories(ros2_control_kindyn
  PRIVATE ${ros2_common_utilities_INCLUDE_DIRS}
)
target_link_libraries(
  ros2_control_kindyn
  PRIVATE
  iDynTree::idyntree-high-level # see here under "Usage in C++":
  iDynTree::idyntree-estimation # https://pypi.org/project/idyntree/
)


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_KINDYN_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_kindyn.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface ros2_control_kindyn.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/ros2_control_kindyn
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_kindyn
)
install(
    TARGETS send_cmd
    RUNTIME DESTINATION lib/ros2_control_kindyn
)

install(TARGETS ros2_control_kindyn
  EXPORT export_ros2_control_kindyn
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_ros2_control_kindyn HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
ament_package()
