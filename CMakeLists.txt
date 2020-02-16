cmake_minimum_required( VERSION 2.8.3 )
project( cpe_project )

find_package( catkin REQUIRED COMPONENTS 
    tf
    roscpp
    nav_msgs
    std_msgs
    sensor_msgs
    zeabus_utility
    dynamic_reconfigure
)

find_package( Boost REQUIRED COMPONENTS )
find_package( zeabus_library REQUIRED COMPONENTS )

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${zeabus_library_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    "${PROJECT_SOURCE_DIR}/include"
)

add_message_files(
    FILES
    Project.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    nav_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp message_runtime std_msgs nav_msgs
    DEPENDS Boost zeabus_library
)