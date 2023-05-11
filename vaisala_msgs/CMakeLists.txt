cmake_minimum_required(VERSION 2.8.3)
project(vaisala_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Location.msg
  Weather.msg
  Full_weather_info.msg
  UB_weather_info.msg)

generate_messages(DEPENDENCIES std_msgs)


catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

install(DIRECTORY migration_rules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
