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

boost::qvm::mat< double , 6 , 1 > mat_constant_viscosity_k = {0,0,0,0,0,0};
boost::qvm::mat< double , 6 , 1 > mat_constant_viscosity_c = {0,0,0,0,0,0};
boost::qvm::mat< double , 6 , 1 > mat_force_observer = {0,0,0,0,0,0};
boost::qvm::mat< double , 6 , 1 > mat_force_thruster = {0,0,0,0,0,0};
boost::array< double , 6 > arr_robot_velocity;
boost::array< double , 6 > arr_world_velocity;

nav_msgs::Odometry message_localize_zeabus;
nav_msgs::Odometry message_observer_zeabus;

ros::Time force_stamp;
ros::Time state_stamp;
ros::Time vision_stamp;
ros::Time time_stamp;

ros::Publisher publisher_message;

tf::Quaternion current_quaternion;

int main( int argv , char** argc )
{
    zeabus_ros::Node node( argv , argc , "project_cpe" );

    ros::NodeHandle nh( "" );

    node.spin();

    // setup part listen current force
    std::mutex lock_message_current_force;
    zeabus_utility::Float64Array8 message_current_force;
    boost::qvm::mat< double , 1 , 8 > mat_thruster_data;
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

    // setup part listen vision data
    std::mutex lock_message_vision_data;

    // setup part publisher calculate state
    ros::Publisher pub_observer_state = nh.advertise< nav_msgs::Odometry >( "/observer/zeabus" ,10 );
    publisher_message = nh.advertise< std_msgs::String >( "/observer/message" , 10 );

    // setup part transformation 
    static tf::TransformBroadcaster broadcaster;
    tf::Transform transform_observer_zeabus;
    message_observer_zeabus.header.frame_id = "odom";
    message_observer_zeabus.child_frame_id = "base_link_observer";

    // setup time stamp
    vision_stamp = state_stamp = force_stamp = ros::Time::now();

    // prepare data parameter
    zeabus::FileCSV fh; // file handle
    fh.open( zeabus_ros::get_full_path( "cpe_project" , "parameter" , "force_observer.txt" ) );
    (void)zeabus::extract_csv_type_1( &fh , (double*) &(mat_force_observer.a) );
    fh.close();
    fh.open( zeabus_ros::get_full_path( "cpe_project" , "parameter" , "viscosity_c.txt" ) );
    (void)zeabus::extract_csv_type_1( &fh , (double*) &(mat_constant_viscosity_c.a) );
    fh.close();
    fh.open( zeabus_ros::get_full_path( "cpe_project" , "parameter" , "viscosity_k.txt" ) );
    (void)zeabus::extract_csv_type_1( &fh , (double*) &(mat_constant_viscosity_k.a) );

    std::cout   << "Parameter force observer\n"; zeabus_boost::print( mat_force_observer ); 
    std::cout   << "Parameter viscosity c\n"; zeabus_boost::print( mat_constant_viscosity_c ); 
    std::cout   << "Parameter viscosity k\n"; zeabus_boost::print( mat_constant_viscosity_k );

    ros::Rate rate( 30 );

    while( ros::ok() )
    {
        rate.sleep();
        // load data about current force
        lock_message_current_force.lock();
        if( force_stamp != message_current_force.header.stamp )
        {
            std::memcpy( (void*) mat_thruster_data.a ,
                    (void*) message_current_force.data.c_array(),
                    sizeof( double ) * 8 );
            mat_force_thruster = boost::qvm::transposed( mat_thruster_data 
                    * zeabus::robot::direction_all ) * zeabus::robot::gravity;
            force_stamp = message_current_force.header.stamp;
        }
        lock_message_current_force.unlock();
        // load data about current state
        lock_message_current_state.lock();
        if( message_localize_zeabus.header.stamp != message_current_state.header.stamp )
        {
            message_localize_zeabus = message_current_state;
        }
        lock_message_current_state.unlock();
        if( state_stamp != message_localize_zeabus.header.stamp )
        {
            // updated global yoll pitch yaw
            zeabus::math::get_euler( message_localize_zeabus.pose.pose.orientation,
                    &roll , &pitch , &yaw );
            zeabus_ros::convert::geometry_quaternion::tf( 
                    &message_localize_zeabus.pose.pose.orientation,
                    &current_quaternion ); 
            state_stamp = message_localize_zeabus.header.stamp;
        }
        // Start part activate observer
        time_stamp = ros::Time::now();

        // Fist We must to listen sensor data 
        updated_data(); 

        // Second we must active model part
        active_model();
 
    } 
}

void pub( const std::string message )
{
    std_msgs::String data;
    data.data = message;
    publisher_message.publish( data );
}
