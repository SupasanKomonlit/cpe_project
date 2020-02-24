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
boost::array< double , 6 > arr_velocity;

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

    // setup part publisher calculate state
    ros::Publisher pub_observer_state = nh.advertise< nav_msgs::Odometry >( "/observer/zeabus" ,10 );

    // prepare data parameter
    zeabus::FileCSV fh; // file handle
    fh.open( zeabus_ros::get_full_path( "cpe_project" , "parameter" , "force_observer.txt" ) );
    (void)zeabus::extract_csv_type_1( &fh , &(mat_force_observer.a[0]) );
    fh.close();
    fh.open( zeabus_ros::get_full_path( "cpe_project" , "parameter" , "viscosity_c.txt" ) );
    (void)zeabus::extract_csv_type_1( &fh , &(mat_constant_viscosity_c.a[0]) );
    fh.close();
    fh.open( zeabus_ros::get_full_path( "cpe_project" , "parameter" , "viscosity_k.txt" ) );
    (void)zeabus::extract_csv_type_1( &fh , &(mat_constant_viscosity_c.a[0]) );

    std::cout   << "Parameter force observer\n"; zeabus_boost::print_data( mat_force_observer ); 
    std::cout   << "Parameter viscosity c\n"; zeabus_boost::print_data( mat_constant_viscosity_c ); 
    std::cout   << "Parameter viscosity k\n"; zeabus_boost::print_data( mat_constant_viscosity_k ); 
}
