cmake_minimum_required(VERSION 3.5.1)
project(kelo_tulip)

if(POLICY CMP0148) 
	cmake_policy(SET CMP0148 OLD) 
endif()
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

if ("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "humble")
	rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
	target_link_libraries(tulip_library "${cpp_typesupport_target}")
else ("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "humble")
	rosidl_target_interfaces(tulip_library ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif ("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "humble")

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

# IF(USE_SETCAP)
# 	add_custom_command(
# 		TARGET platform_driver 
# 		POST_BUILD
# 		COMMAND ${CMAKE_COMMAND} -E echo "Set capabilities of platform_driver"
# 		COMMAND ${SUDO_COMMAND} setcap cap_sys_nice,cap_net_raw+ep $<TARGET_FILE:platform_driver>
# 	)
# endif(USE_SETCAP)

install(TARGETS
	tulip_library
	tulip_velocity_controller
	DESTINATION lib
)

install(TARGETS
	platform_driver
	DESTINATION lib/${PROJECT_NAME}
)

IF(USE_SETCAP)
set (t_cmd setcap)
set (t_arg cap_sys_nice,cap_net_raw+ep)
set (t_file ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/platform_driver)
set (t_ld_path /etc/ld.so.conf.d)
set (t_ld_name ld_kelo_tulip.conf)
install( CODE 
    "
message(NOTICE \"POST INSTALL\")
execute_process(COMMAND getcap ${t_file} RESULT_VARIABLE t_res OUTPUT_VARIABLE t_cap)
#message(NOTICE \"getcap1 => \${t_cap}\")
if(t_cap STREQUAL \"\")
	message(NOTICE \"sudo ${t_cmd} ${t_arg} ${t_file}\")
	execute_process(COMMAND sudo ${t_cmd} ${t_arg} ${t_file} )
	execute_process(COMMAND getcap ${t_file} RESULT_VARIABLE t_res OUTPUT_VARIABLE t_cap)
	#message(NOTICE \"getcap2 => \${t_cap}\")
else()	
	message(NOTICE \"${t_file} already has capabilities set, no action.\")
endif()
if(EXISTS ${t_ld_path}/${t_ld_name})
	message(NOTICE \"${t_ld_path}/${t_ld_name} already exists, no action.\")
else()
	file(WRITE ${t_ld_name} \"/opt/ros/\$ENV{ROS_DISTRO}/lib\n\${CMAKE_INSTALL_PREFIX}/lib\n\")
	execute_process(COMMAND sudo cp ${t_ld_name} ${t_ld_path} )
	message(NOTICE \"${t_ld_name} generated and copied to ${t_ld_path}\")
	execute_process(COMMAND sudo ldconfig )
	message(NOTICE \"sudo ldconfig excecuted.\")
endif()
    "
)
# TODO : find solution when sudo requires password (request is not shown)
# To mitigate this, cache sudo credentials before colcon build (f.e. sudo ls)
ENDIF(USE_SETCAP)

install(DIRECTORY 
	include/
	DESTINATION include
)

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
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_dependencies(Boost)
ament_export_dependencies(std_msgs)
ament_export_libraries(tulip_library tulip_velocity_controller soem)

ament_package()
