cmake_minimum_required(VERSION 3.5)
project(kindyn)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/devel/lib)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g -fext-numeric-literals")
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
execute_process(COMMAND grep -o BCM /proc/cpuinfo OUTPUT_VARIABLE IAMRASPBERRYPI3)
if (IAMRASPBERRYPI3)
    message(STATUS "detected raspberry pi, defining EIGEN_DONT_ALIGN_STATICALLY!!!")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_DONT_ALIGN_STATICALLY")
endif ()


# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#sudo apt-get install ros-foxy-ros2-control
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)

# eigen_conversions merged in geometry_2 in ros2 https://github.com/ros2/geometry2/pull/311
# apt-get install ros-foxy-geometry2
# find_package(eigen_conversions REQUIRED)
find_package(geometry2 REQUIRED)
#sudo apt-get install ros-foxy-effort-controllers
find_package(effort_controllers REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(roboy_control_msgs REQUIRED)
find_package(roboy_middleware_msgs REQUIRED)
find_package(roboy_simulation_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)



include_directories(include)


ament_export_dependencies(
  roboy_middleware_msgs
  common_utilities
)


# Eigen
find_package(PkgConfig REQUIRED )
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

# # libxml++
# find_package(LibXML++ REQUIRED)
find_package (LibXml2 REQUIRED)
include_directories(${LibXml2_INCLUDE_DIRS})

# idyntree
find_package(iDynTree 0.11.1 REQUIRED)
include_directories(${iDynTree_INCLUDE_DIRS})

# qpOASES
#https://github.com/coin-or/qpOASES
#mkdir build && cd build 
#cmake .. 
#sudo make 
#sudo make install  
find_package(qpOASES REQUIRED)
include_directories(${qpOASES_INCLUDE_DIRS})

# find openmp support
find_package(OpenMP)
if (OPENMP_FOUND)
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


#==============================================================================
# Application
#==============================================================================
# CARDSflow hardware interface
# add_library(CardsflowHardwareInterface
# 		src/controller/cardsflow_state_interface.cpp
# 		src/controller/cardsflow_command_interface.cpp)
# add_dependencies(CardsflowHardwareInterface roboy_middleware_msgs_generate_messages_cpp)
# target_link_libraries(CardsflowHardwareInterface ${catkin_LIBRARIES})


install(
  DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(
  include
)



# ROS2
# CARDSflow hardware interface
add_library(CardsflowHardwareInterface
    src/controller/cardsflow_state_interface.cpp
    src/controller/cardsflow_command_interface.cpp
    )

ament_target_dependencies(CardsflowHardwareInterface
    "rclcpp"
    "roboy_middleware_msgs"
  )
install(TARGETS CardsflowHardwareInterface
    DESTINATION lib/${PROJECT_NAME})


# # Cable Length Controller
# add_library(CableLengthController src/controller/cableLengthController.cpp )
# add_dependencies(CableLengthController roboy_middleware_msgs_generate_messages_cpp roboy_control_msgs_generate_messages_cpp)
# target_link_libraries(CableLengthController ${catkin_LIBRARIES} CardsflowHardwareInterface)

# ROS2
# Cable Length Controller
add_library(CableLengthController src/controller/cableLengthController.cpp)
ament_target_dependencies(CableLengthController
  "rclcpp"
  "controller_interface"
  "roboy_simulation_msgs"
  "roboy_control_msgs"
  "roboy_middleware_msgs"
  "hardware_interface"

  )
install(TARGETS CableLengthController 
    DESTINATION lib/${PROJECT_NAME})


# # Cable Length Velocity Controller
# add_library(CableLengthVelocityController src/controller/cableLengthVelocityController.cpp )
# add_dependencies(CableLengthVelocityController roboy_middleware_msgs_generate_messages_cpp roboy_control_msgs_generate_messages_cpp)
# target_link_libraries(CableLengthVelocityController ${catkin_LIBRARIES} CardsflowHardwareInterface)

# ROS2
# Cable Length Velocity Controller
add_library(CableLengthVelocityController src/controller/cableLengthVelocityController.cpp)
ament_target_dependencies(CableLengthVelocityController
  "rclcpp"
  "controller_interface"
  "roboy_simulation_msgs"
  "roboy_control_msgs"
  "hardware_interface"
  "roboy_middleware_msgs"
  )
install(TARGETS CableLengthVelocityController
    DESTINATION lib/${PROJECT_NAME})


# # Torque Position Controller
# add_library(TorquePositionController src/controller/torquePositionController.cpp )
# add_dependencies(TorquePositionController roboy_middleware_msgs_generate_messages_cpp roboy_control_msgs_generate_messages_cpp)
# target_link_libraries(TorquePositionController ${catkin_LIBRARIES} CardsflowHardwareInterface)

# ROS2
# Torque Position Controller
add_library(TorquePositionController src/controller/torquePositionController.cpp)
ament_target_dependencies(TorquePositionController
  "rclcpp"
  "controller_interface"
  "roboy_simulation_msgs"
  "roboy_control_msgs"
  "hardware_interface"
  "roboy_middleware_msgs"
  )
install(TARGETS TorquePositionController
    DESTINATION lib/${PROJECT_NAME})



# # Force Position Controller
# add_library(ForcePositionController src/controller/forcePositionController.cpp )
# add_dependencies(ForcePositionController roboy_middleware_msgs_generate_messages_cpp roboy_control_msgs_generate_messages_cpp)
# target_link_libraries(ForcePositionController ${catkin_LIBRARIES} CardsflowHardwareInterface)

# ROS2
# Force Position Controller
add_library(ForcePositionController src/controller/forcePositionController.cpp)
ament_target_dependencies(ForcePositionController
  "rclcpp"
  "controller_interface"
  "roboy_simulation_msgs"
  "roboy_control_msgs"
  "hardware_interface"
  "roboy_middleware_msgs"
  )
install(TARGETS ForcePositionController
    DESTINATION lib/${PROJECT_NAME})


# # kindyn library
# add_library(kindyn SHARED
#         src/robot.cpp
# 		src/kinematics.cpp
# 		src/EigenExtension.cpp
# 		include/kindyn/EigenExtension.hpp
# 		include/kindyn/robot.hpp
# 		include/kindyn/kinematics.hpp
# 		include/kindyn/cable.hpp)
# add_dependencies(kindyn roboy_simulation_msgs_generate_messages_cpp roboy_simulation_msgs_msgs_generate_messages_cpp)
# target_link_libraries(kindyn ${catkin_LIBRARIES} ${iDynTree_LIBRARIES} ${qpOASES_LIBRARIES} CardsflowHardwareInterface
# 		CableLengthController TorquePositionController ForcePositionController ${OpenMP_CXX_libraries})


# ROS2
# kindyn library
add_library(kindyn SHARED
    src/robot.cpp
		src/kinematics.cpp
		src/EigenExtension.cpp
		include/kindyn/EigenExtension.hpp
		include/kindyn/robot.hpp
		include/kindyn/kinematics.hpp
		include/kindyn/cable.hpp
    )
ament_target_dependencies(kindyn
    "rclcpp"
    "controller_interface"
    "roboy_simulation_msgs"
    "roboy_control_msgs"
    "roboy_middleware_msgs"
    "hardware_interface"
    )
install(TARGETS kindyn
    DESTINATION lib/${PROJECT_NAME})


# # vrpuppet library
# add_library(vrpuppet SHARED
# 		src/vrpuppet.cpp
# 		src/EigenExtension.cpp
# 		include/kindyn/EigenExtension.hpp
# 		include/kindyn/vrpuppet.hpp
# 		include/kindyn/cable.hpp)
# add_dependencies(vrpuppet roboy_simulation_msgs_generate_messages_cpp roboy_simulation_msgs_msgs_generate_messages_cpp)
# target_link_libraries(vrpuppet ${catkin_LIBRARIES} ${iDynTree_LIBRARIES} ${qpOASES_LIBRARIES} CardsflowHardwareInterface
# 		CableLengthController TorquePositionController ForcePositionController ${OpenMP_CXX_libraries})

# ROS2
# vrpuppet library
add_library(vrpuppet SHARED
		src/vrpuppet.cpp
		src/EigenExtension.cpp
		include/kindyn/EigenExtension.hpp
		include/kindyn/vrpuppet.hpp
		include/kindyn/cable.hpp
    )
ament_target_dependencies(vrpuppet
    "rclcpp"
    "controller_interface"
    "roboy_simulation_msgs"
    "roboy_control_msgs"
    "roboy_middleware_msgs"
    "hardware_interface"
    )
install(TARGETS vrpuppet
    DESTINATION lib/${PROJECT_NAME})


# # testing idyntree
# add_executable( test_idyntree src/utilities/test_idyntree.cpp)
# target_link_libraries(test_idyntree ${iDynTree_LIBRARIES} ${catkin_LIBRARIES})

# # upper_body
# add_executable( upper_body src/robots/upper_body.cpp)
# target_link_libraries(upper_body ${iDynTree_LIBRARIES} ${catkin_LIBRARIES} vrpuppet kindyn)

# #==============================================================================
# # Installation
# #==============================================================================
# install(TARGETS
# 		kindyn
# 		CardsflowHardwareInterface
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

# install(TARGETS
# 	    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(FILES
#         package.xml
# 		CableLengthController_plugin.xml
# 		TorquePositionController_plugin.xml
# 		ForcePositionController_plugin.xml
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(DIRECTORY
# 		include/kindyn/
# 		include/kindyn/controller
# 		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# 		FILES_MATCHING PATTERN "*.hpp"
# 		)

ament_package()
