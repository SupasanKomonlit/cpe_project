// FILE			: listen_overview.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 16 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    "listen_overview.hpp"

tf::Quaternion current_quaternion( 0 ,0 , 0 , 1 );
std::vector< nav_msgs::Quaternion > buffer_current_state;
boost::qvm::vec< double , 6 > vec_current_velocity = { 0 , 0 , 0 , 0 , 0 , 0 };  
boost::qvm::mat< double , 1 , 8 > mat_force_individual_thruster = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 };

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
    listen_current_force.setup_subscriber( "/control/force/current" );

    // setup part listen current state
    std::mutex lock_message_current_state;
    nav_msgs::Odometry message_current_state;
    zeabus_ros::subscriber::BaseClass< nav_msgs::Odometry > listen_current_state( &nh,
            &message_current_state );
    listen_current_state.setup_mutex_data( &lock_message_current_state );
    listen_current_state.setup_subscriber( "localize/zeabus" );

    ros::Rate rate( 50 );
    ros::Time stamp_force = ros::Time::now();
    ros::Time stamp_state = ros::Time::now();
    bool have_new_message = false;
    while( ros::ok() )
    {
        rate.sleep(); // sleep with single period of frequency
        // Load message of current force
        lock_message_current_force.lock();
        if( stamp_force < message_current_force.header.stamp )
        {
            stamp_force = message_current_force.header.stamp;
            have_new_message = true;
            boost::qvm::A00( mat_force_thruster ) = message_current_force.data[ 0 ] *
                    zeabus::robot::gravity;
            boost::qvm::A01( mat_force_thruster ) = message_current_force.data[ 1 ] *
                    zeabus::robot::gravity;
            boost::qvm::A02( mat_force_thruster ) = message_current_force.data[ 2 ] *
                    zeabus::robot::gravity;
            boost::qvm::A03( mat_force_thruster ) = message_current_force.data[ 3 ] *
                    zeabus::robot::gravity;
            boost::qvm::A04( mat_force_thruster ) = message_current_force.data[ 4 ] *
                    zeabus::robot::gravity;
            boost::qvm::A05( mat_force_thruster ) = message_current_force.data[ 5 ] *
                    zeabus::robot::gravity;
            boost::qvm::A06( mat_force_thruster ) = message_current_force.data[ 6 ] *
                    zeabus::robot::gravity;
            boost::qvm::A07( mat_force_thruster ) = message_current_force.data[ 7 ] *
                    zeabus::robot::gravity;
        }
        lock_message_current_force.unlock();
        // Load message of current state
        lock_message_current_state.lock();
        if( stamp_state < message_current_state.header.stamp )
        {
            stamp_state = message_current_state.header.stamp;
            buffer_current_state.push( message_current_state );
        }
        lock_message_current_state.unlock();

        if( have_new_message )
        {
            have_new_message = false;
            // Now we have to calculate force
            // First we must choose data about current quaternion
            if( buffer_current_state.size() != 0 )
            {
                while( buffer_current_state.size() > 1 )
                {
                    if( buffer_current_state[2].header.stamp < stamp_state )
                    {
                        buffer_current_state.erase( buffer_current_state.begin() );
                    } // Can use index 1 (count form 0 ) instead index 0
                    else
                    {
                        break; 
                    } // Now index 1 (count from 0) have stamp_state > stamp_force 
                }
                zeabus_ros::convert::geometry_quaternion::tf( &buffer_current_state[0].header.stamp,
                        &current_quaternion );
                calculate();
                report();
            } // you don't have data in buffer you can't do anything
            
        }
    }

} // exit main
