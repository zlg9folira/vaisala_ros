cmake_minimum_required(VERSION 2.8.3)
project(vaisala_xml)

find_package(catkin REQUIRED COMPONENTS
	rospy
	message_generation
	std_msgs
	vaisala_msgs
	sensor_msgs)


catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime vaisala_msgs)

catkin_install_python(PROGRAMS scripts/full_weather_info_publisher.py scripts/ub_weather_info_publisher.py scripts/image_publisher.py scripts/utility.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

install(DIRECTORY migration_rules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
