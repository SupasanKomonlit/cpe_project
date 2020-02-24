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

add_executable( subject_3_3 
    src/listen_overview.cpp
    src/purpose_3_3.cpp
)
target_link_libraries( subject_3_3 
    ${zeabus_library_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)
add_dependencies( subject_3_3 
    cpe_project_generate_messages_cpp 
)

add_executable( find_center
    src/learn_center.cpp
    src/learn_center_model.cpp
)
target_link_libraries( find_center
    ${zeabus_library_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)
add_dependencies( find_center 
    cpe_project_generate_messages_cpp
)

add_executable( find_inertia 
    src/learn_moment_inertia_model.cpp
    src/learn_moment_inertia.cpp
)
target_link_libraries( find_inertia
    ${zeabus_library_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)
add_dependencies( find_inertia 
    cpe_project_generate_messages_cpp
)
