cmake_minimum_required(VERSION 2.8.3)
project(kelo_tulip)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs
	nav_msgs
	sensor_msgs
	std_msgs
	tf
)

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

catkin_package(
	INCLUDE_DIRS ${INCLUDE_DIRS} 
	LIBRARIES tulip_velocity_controller
)

include_directories(
	${INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
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

target_link_libraries(tulip_velocity_controller
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

add_executable(platform_driver
	src/EtherCATModule.cpp
	src/PlatformDriver.cpp
	src/PlatformDriverROS.cpp
	src/modules/RobileMasterBattery.cpp
)

target_link_libraries(platform_driver
	tulip_velocity_controller
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	soem
	pthread
)

IF(USE_SETCAP)
  add_custom_command(TARGET platform_driver POST_BUILD
  	COMMAND ${SUDO_COMMAND} setcap cap_net_raw+ep $<TARGET_FILE:platform_driver>
  )
endif(USE_SETCAP)

install(TARGETS platform_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

