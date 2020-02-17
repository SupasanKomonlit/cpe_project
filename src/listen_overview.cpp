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
std::vector< nav_msgs::Odometry > buffer_current_state;
boost::qvm::vec< double , 6 > vec_current_velocity = { 0 , 0 , 0 , 0 , 0 , 0 };  
boost::qvm::mat< double , 1 , 8 > mat_force_individual_thruster = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 };
cpe_project::Project message;

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

    // setup part listen current state
    std::mutex lock_message_current_state;
    nav_msgs::Odometry message_current_state;
    zeabus_ros::subscriber::BaseClass< nav_msgs::Odometry > listen_current_state( &nh,
            &message_current_state );
    listen_current_state.setup_mutex_data( &lock_message_current_state );
    listen_current_state.setup_subscriber( "localize/zeabus" , 1 );

    ros::Publisher publisher = nh.advertise< cpe_project::Project >( "/project/subject_3_3" , 10 );

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
            std::memcpy( (void*) &mat_force_individual_thruster.a[0][0] , 
                    (void*) message_current_force.data.c_array(),
                    sizeof( double ) * 8 );
            mat_force_individual_thruster *= zeabus::robot::gravity;
        }
        lock_message_current_force.unlock();
        // Load message of current state
        lock_message_current_state.lock();
        if( stamp_state < message_current_state.header.stamp )
        {
            stamp_state = message_current_state.header.stamp;
            buffer_current_state.push_back( message_current_state );
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
                    if( buffer_current_state[2].header.stamp < stamp_force )
                    {
                        buffer_current_state.erase( buffer_current_state.begin() );
                    } // Can use index 1 (count form 0 ) instead index 0
                    else
                    {
                        break; 
                    } // Now index 1 (count from 0) have stamp_state > stamp_force 
                }
                current_quaternion = tf::Quaternion( 
                        buffer_current_state[0].pose.pose.orientation.x , 
                        buffer_current_state[0].pose.pose.orientation.y , 
                        buffer_current_state[0].pose.pose.orientation.z , 
                        buffer_current_state[0].pose.pose.orientation.w );
                message.header.stamp = stamp_force;
                message.state = buffer_current_state[ 0 ];
                boost::qvm::A0( vec_current_velocity ) = message.state.twist.twist.linear.x;
                boost::qvm::A1( vec_current_velocity ) = message.state.twist.twist.linear.y;
                boost::qvm::A2( vec_current_velocity ) = message.state.twist.twist.linear.z;
                boost::qvm::A3( vec_current_velocity ) = message.state.twist.twist.angular.x;
                boost::qvm::A4( vec_current_velocity ) = message.state.twist.twist.angular.y;
                boost::qvm::A5( vec_current_velocity ) = message.state.twist.twist.angular.z;
                calculate();
                report( &publisher );
            } // you don't have data in buffer you can't do anything
            
        }
    }

} // exit main
