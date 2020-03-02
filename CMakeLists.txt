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
    cv_bridge
    image_transport
)

find_package( Boost REQUIRED COMPONENTS )
find_package( zeabus_library REQUIRED COMPONENTS )
find_package( zeabus_vision REQUIRED COMPONENTS )
find_package( OpenCV REQUIRED  )

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${zeabus_library_INCLUDE_DIRS}
    ${zeabus_vision_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    "${PROJECT_SOURCE_DIR}/include"
)

add_message_files(
    FILES
    Project.msg
)

generate_dynamic_reconfigure_options(
    cfg/Simple3Data.cfg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    nav_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp message_runtime std_msgs nav_msgs
    DEPENDS Boost zeabus_library ${PROJECT_NAME}_gencfg zeabus_vision
)

add_executable( subject_3_3 
    src/listen/listen_overview.cpp
    src/listen/purpose_3_3.cpp
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
    src/learn_center/learn_center.cpp
    src/learn_center/learn_center_model.cpp
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
    src/learn_moment/learn_moment_inertia_model.cpp
    src/learn_moment/learn_moment_inertia.cpp
)
target_link_libraries( find_inertia
    ${zeabus_library_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)
add_dependencies( find_inertia 
    cpe_project_generate_messages_cpp
)

add_executable( observer 
    src/observer/observer.cpp
    src/observer/observer_model.cpp
    src/observer/observer_update.cpp
    src/observer/observer_gradient.cpp
)
target_link_libraries( observer
    ${zeabus_library_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)
add_dependencies( observer 
    cpe_project_generate_messages_cpp
)

add_executable( bottom 
    src/vision/vision_bottom.cpp
)
target_link_libraries( bottom 
    ${zeabus_library_LIBRARIES}
    ${zeabus_vision_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_dependencies( bottom
    cpe_project_generate_messages_cpp
    ${PROJECT_NAME}_gencfg
)

add_executable( front 
    src/vision/vision_front.cpp
)
target_link_libraries( front 
    ${zeabus_library_LIBRARIES}
    ${zeabus_vision_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_dependencies( front
    cpe_project_generate_messages_cpp
    ${PROJECT_NAME}_gencfg
)
