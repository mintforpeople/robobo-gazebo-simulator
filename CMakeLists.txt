cmake_minimum_required(VERSION 2.8.3)
project(robobo_gazebo)

set (CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	gazebo_ros 
	gazebo_plugins 
	robobo_msgs 
	rospy 
	
	message_generation
	gazebo_dev
        gencpp
        nodelet
        angles
        std_srvs
        geometry_msgs
        sensor_msgs
        nav_msgs
        urdf
        tf
        tf2_ros
        dynamic_reconfigure
        rosgraph_msgs
        trajectory_msgs
        image_transport
        rosconsole
        cv_bridge
        polled_camera
        diagnostic_updater
        camera_info_manager
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(ignition-math4 REQUIRED)


## System dependencies
find_package(GAZEBO REQUIRED)

catkin_python_setup()


################################################
## Declare ROS messages, services and actions ##
################################################
# Generate services in the 'srv' folder
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
	INCLUDE_DIRS
	plugins/include

	LIBRARIES
	move_wheels
	move_pan_tilt
	encoders
	infrared_range
	robobo_imu_sensor
	light_sensor
	battery_discharge
        battery_consumer
	led_color
	emotion
	
	CATKIN_DEPENDS
	roscpp
	rospy
	gazebo_ros
	gazebo_plugins

	std_msgs

	DEPENDS
	GAZEBO
)


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS} ${IGNITION-MATH4_INCLUDE_DIRS})

# Next line is necessary because Gazebo plugins are not added to GAZEBO_LIBRARIES
link_directories(${GAZEBO_LIBRARY_DIRS})

# Plugin for MoveWheels service
add_library(move_wheels SHARED plugins/move_wheels.cpp)
add_dependencies(move_wheels ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_wheels ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

# Plugin for MovePanTilt service
add_library(move_pan_tilt SHARED plugins/move_pan_tilt.cpp)
add_dependencies(move_pan_tilt ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_pan_tilt ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Motor encoders to know joints position and ResetWheels service
add_library(encoders SHARED plugins/encoders.cpp)
add_dependencies(encoders ${catkin_EXPORTED_TARGETS})
target_link_libraries(encoders ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Plugin to read infrared sensors
add_library(infrared_range SHARED plugins/infrared_range.cpp)
add_dependencies(infrared_range ${catkin_EXPORTED_TARGETS})
target_link_libraries(infrared_range ${catkin_LIBRARIES} ${Boost_LIBRARIES} RayPlugin)

# Imu sensor plugin
add_library(robobo_imu_sensor SHARED plugins/robobo_imu_sensor.cpp)
add_dependencies(robobo_imu_sensor ${catkin_EXPORTED_TARGETS})
target_link_libraries(robobo_imu_sensor ${catkin_LIBRARIES})

#Iluminance
add_library(light_sensor SHARED plugins/light_sensor.cpp)
add_dependencies(light_sensor ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries( light_sensor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} CameraPlugin )

#EMOTION
add_library(emotion SHARED plugins/emotion.cpp)
add_dependencies(emotion ${catkin_EXPORTED_TARGETS})
target_link_libraries(emotion ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${roscpp_LIBRARIES} ${catkin_LIBRARIES})

#LED
add_library(led_color SHARED plugins/led_color.cpp)
add_dependencies(led_color ${catkin_EXPORTED_TARGETS})
target_link_libraries(led_color ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${roscpp_LIBRARIES} ${catkin_LIBRARIES})

IF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    ADD_DEFINITIONS(-fPIC)
ENDIF()

add_library(battery_discharge SHARED plugins/battery_discharge.cpp)
add_dependencies(battery_discharge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(battery_discharge ${roscpp_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${IGNITION-MATH_LIBRARIES})

add_library(battery_consumer SHARED plugins/battery_consumer.cpp)
add_dependencies(battery_consumer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(battery_consumer ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${IGNITION-MATH_LIBRARIES})


#############
## Install ##
#############
catkin_install_python(
	PROGRAMS
	nodes/robobo_irs

	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
	TARGETS
	move_wheels
	move_pan_tilt
	encoders
	infrared_range
	robobo_imu_sensor
	battery_discharge
        battery_consumer
	led_color
	emotion
	
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}

	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
	DIRECTORY plugins/include/robobo/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
	DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  	DIRECTORY models/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

install(
	DIRECTORY worlds/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)
