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

boost::qvm::mat< double , 6 , 1 > mat_constant_viscosity_k = {
        9.24136,        9.29168,         5.93436, 
        1,              1,              1};
boost::qvm::mat< double , 6 , 1 > mat_constant_viscosity_c = {
        4.75,           5.42672,        4.19487,
        1,              1,              1};
boost::qvm::mat< double , 6 , 1 > mat_force_observer = {
        -0.00444832,    -0.0156679,     -340.221,
        0,              0,              0};
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
    zeabus_utility::Float64Array6 message_acceleration;
    ros::Publisher pub_acceleration = nh.advertise< zeabus_utility::Float64Array6 >( 
            "/observer/acceleration", 10 );

    // setup part transformation 
    static tf::TransformBroadcaster broadcaster;
    tf::Transform transform_observer_zeabus;
    message_observer_zeabus.header.frame_id = "odom";
    message_observer_zeabus.child_frame_id = "base_link_observer";

    // setup time stamp
    vision_stamp = state_stamp = force_stamp = ros::Time::now();
/*
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
*/
    std::cout   << "Parameter force observer\n"; zeabus_boost::print( mat_force_observer ); 
    std::cout   << "Parameter viscosity c\n"; zeabus_boost::print( mat_constant_viscosity_c ); 
    std::cout   << "Parameter viscosity k\n"; zeabus_boost::print( mat_constant_viscosity_k );

    ros::Rate rate( 30 );
    const double diff_time = 1.0/30;

    boost::array< double , 3 > array_linear_distance;
    boost::array< double , 3 > array_angular_distance;
    message_observer_zeabus.header.frame_id = "odom";
    message_observer_zeabus.child_frame_id = "base_link_observer";

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

        // Before start process calcualte please check status
        if( ! observer_status )
        {
            continue;
        }

        // Before active we must to rotation vector velocity world to robot
        arr_robot_velocity[ 0 ] = arr_world_velocity[ 0 ];          
        arr_robot_velocity[ 1 ] = arr_world_velocity[ 1 ];          
        arr_robot_velocity[ 2 ] = arr_world_velocity[ 2 ];
        arr_robot_velocity[ 3 ] = message_localize_zeabus.twist.twist.angular.x;
        arr_robot_velocity[ 4 ] = message_localize_zeabus.twist.twist.angular.y;
        arr_robot_velocity[ 5 ] = message_localize_zeabus.twist.twist.angular.z;
        zeabus_ros::convert::geometry_quaternion::tf( &current_quaternion,
                &message_localize_zeabus.pose.pose.orientation ); 
        zeabus::math::rotation( current_quaternion , arr_robot_velocity.c_array() );
    
        // Second we must active model part
        active_model();

        // Third Integral term acceleration to distance
        array_linear_distance[ 0 ] = arr_robot_velocity[ 0 ] * diff_time + 
                mat_acceleration.a[ 0 ][ 0 ] * pow( diff_time , 2 ) / 2.0;
        array_linear_distance[ 1 ] = arr_robot_velocity[ 1 ] * diff_time + 
                mat_acceleration.a[ 1 ][ 0 ] * pow( diff_time , 2 ) / 2.0;
        array_linear_distance[ 2 ] = arr_robot_velocity[ 2 ] * diff_time + 
                mat_acceleration.a[ 2 ][ 0 ] * pow( diff_time , 2 ) / 2.0;

        zeabus::math::inv_rotation( current_quaternion , array_linear_distance.c_array() );
        arr_world_velocity[ 0 ] = arr_robot_velocity[ 0 ] + mat_acceleration.a[0][0]*diff_time;
        arr_world_velocity[ 1 ] = arr_robot_velocity[ 1 ] + mat_acceleration.a[1][0]*diff_time;
        arr_world_velocity[ 2 ] = arr_robot_velocity[ 2 ] + mat_acceleration.a[2][0]*diff_time;
        zeabus::math::inv_rotation( current_quaternion , arr_world_velocity.c_array() );

        // Forth We must get result in to message
        std::cout   << "Plus position z " << array_linear_distance[ 2 ] << "\n";
        message_observer_zeabus.pose.pose.position.x += array_linear_distance[ 0 ] / 2 ;
        message_observer_zeabus.pose.pose.position.y += array_linear_distance[ 1 ] / 2;
        message_observer_zeabus.pose.pose.position.z += array_linear_distance[ 2 ] / 2;
        message_observer_zeabus.pose.pose.orientation = 
                message_localize_zeabus.pose.pose.orientation;
        message_observer_zeabus.header.stamp = time_stamp;
        message_observer_zeabus.twist.twist.linear.x = arr_robot_velocity[ 0 ];
        message_observer_zeabus.twist.twist.linear.y = arr_robot_velocity[ 1 ];
        message_observer_zeabus.twist.twist.linear.z = arr_robot_velocity[ 2 ];

        // Last We muest limit our data
        limit_data(); 

        // Before end publish data
        pub_observer_state.publish( message_observer_zeabus );
        transform_observer_zeabus.setOrigin( tf::Vector3(
                message_observer_zeabus.pose.pose.position.x,
                message_observer_zeabus.pose.pose.position.y,
                message_observer_zeabus.pose.pose.position.z ) );
        transform_observer_zeabus.setRotation( tf::Quaternion(
                message_observer_zeabus.pose.pose.orientation.x,
                message_observer_zeabus.pose.pose.orientation.y,
                message_observer_zeabus.pose.pose.orientation.z,
                message_observer_zeabus.pose.pose.orientation.z) );
        broadcaster.sendTransform( tf::StampedTransform(
                transform_observer_zeabus,
                message_observer_zeabus.header.stamp,
                message_observer_zeabus.header.frame_id,
                message_observer_zeabus.child_frame_id ) );

        std::memcpy( (void*) message_acceleration.data.c_array() , 
                (void*) mat_acceleration.a,
                sizeof( double ) * 6 );
        message_acceleration.header.stamp = time_stamp;
        pub_acceleration.publish( message_acceleration );

    } 
}

void pub( const std::string message , const bool print )
{
    if( print ) std::cout   << message << "\n";
    std_msgs::String data;
    data.data = message;
    publisher_message.publish( data );
}
