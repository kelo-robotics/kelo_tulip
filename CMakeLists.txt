cmake_minimum_required(VERSION 3.5)
project(kelo_tulip)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS thread)

if(WIN32)
	set(INCLUDE_DIRS
		include
		include/kelo_tulip/soem
		include/kelo_tulip/soem/osal
		include/kelo_tulip/soem/osal/win32
		include/kelo_tulip/soem/oshw/win32
		${WINPCAP_INCLUDE_DIRS}
	)
else(WIN32)
	set(INCLUDE_DIRS
		include
		include/kelo_tulip/soem
		include/kelo_tulip/soem/osal
		include/kelo_tulip/soem/osal/linux
		include/kelo_tulip/soem/oshw/linux
	)
endif(WIN32)

set(msg_files
	"msg/KeloDriveInput.msg"
	"msg/KeloDrivesInput.msg"
	"msg/ShockBin.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
	${msg_files}
	DEPENDENCIES std_msgs
)

include_directories(
	${INCLUDE_DIRS}
)

# Optionally apply cap_net_raw+ep permissions to binary. Requires root previleges.
# Use sudo as default. Still, allow to be configured as environment veriable.
OPTION(USE_SETCAP "Set permissions to access ethernet interface without sudo" ON)
IF("$ENV{SUDO_COMMAND}" STREQUAL "")
	SET(SUDO_COMMAND sudo)				# default = sudo
ELSE("$ENV{SUDO_COMMAND}" STREQUAL "")
	SET(SUDO_COMMAND $ENV{SUDO_COMMAND})
ENDIF("$ENV{SUDO_COMMAND}" STREQUAL "")

add_subdirectory(src/soem)

add_library(tulip_velocity_controller
	src/Utils.cpp
	src/VelocityPlatformController.cpp
)

ament_target_dependencies(tulip_velocity_controller
	rclcpp
	geometry_msgs
	nav_msgs
	sensor_msgs
	std_msgs
	tf2
	tf2_ros
	tf2_geometry_msgs
)

add_library(tulip_library
	src/EtherCATMaster.cpp
	src/EtherCATModule.cpp
	src/EtherCATModuleROS.cpp
	src/PlatformDriver.cpp
	src/PlatformDriverROS.cpp
	src/modules/RobileMasterBattery.cpp
	src/modules/RobileMasterBatteryROS.cpp
)

ament_target_dependencies(tulip_library
	rclcpp
	geometry_msgs
	nav_msgs
	sensor_msgs
	std_msgs
	tf2
	tf2_ros
	tf2_geometry_msgs
	Boost
)

rosidl_target_interfaces(tulip_library ${PROJECT_NAME} "rosidl_typesupport_cpp")
#rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_link_libraries(tulip_library "${cpp_typesupport_target}")

add_executable(platform_driver
	src/TulipMain.cpp
)

target_link_libraries(platform_driver
	${Boost_LIBRARIES}
	tulip_library
	tulip_velocity_controller
	soem
	pthread
)

IF(USE_SETCAP)
	add_custom_command(TARGET platform_driver POST_BUILD
		COMMAND ${SUDO_COMMAND} setcap cap_sys_nice,cap_net_raw+ep $<TARGET_FILE:platform_driver>
	)
endif(USE_SETCAP)

install(TARGETS
	tulip_library
	tulip_velocity_controller
	DESTINATION lib
)

install(TARGETS
	platform_driver
	DESTINATION lib/${PROJECT_NAME}
)

#install(
#  DIRECTORY include/
#  DESTINATION include/
#)

install(DIRECTORY
	config
	launch
	DESTINATION share/${PROJECT_NAME}/
)

install(
	TARGETS tulip_library tulip_velocity_controller
	EXPORT export_${PROJECT_NAME}
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(Boost COMPONENTS thread)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(tulip_library tulip_velocity_controller soem)

ament_package()
