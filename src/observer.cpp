// FILE			: observer.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 21 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE
//  For calculate between time for 0 -> 1 
//  This code will transfer velocity and position in world coordinate

// MACRO SET

// MACRO CONDITION

#include    "observer.hpp"

int main( int argv , char** argc )
{
    zeabus_ros::Node node( argv , argc , "project_cpe" );

    ros::NodeHandle nh( "" );

    node.spin();

    // setup part listen current force
    std::mutex lock_message_current_force;
    zeabus_utility::Float64Array8 message_current_force;
    zeabus_ros::subscriber::BaseClass< zeabus_utility::Float64Array8 > listen_current_force( &nh,
            &message_current_force );
    listen_current_force.setup_mutex_data( &lock_message_current_force );
    listen_current_force.setup_subscriber( "/control/force/current" , 1 );

    // setup part listern current state
    std::mutex lock_message_current_state;
    nav_msgs::Odometry message_current_state;
    zeabus_ros::subscriber::BaseClass< nav_msgs::Odometry > listen_current_state( &nh,
            &message_current_state );
    listen_current_state.setup_mutex_data( &lock_message_current_state );
    listen_current_state.setup_subscriber( "localize/zeabus" , 1 );

    
}
