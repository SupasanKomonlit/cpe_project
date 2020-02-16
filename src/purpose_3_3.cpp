// FILE			: purpose_3_3.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 16 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    "listen_overview.hpp"

boost::qvm::mat< double , 1 , 6 > mat_force_thruster = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 1 , 6 > mat_force_buoncy = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 1 , 6 > mat_force_gravity = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 1 , 6 > mat_force_epsilon = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 1 , 6 > mat_force_viscosity = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 1 , 6 > mat_force_summation = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::vec< double , 6 > vec_calculate_velocity = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::vec< double , 6 > vec_calculate_acceleration = { 0 , 0 , 0 , 0 , 0 , 0 };

void calculate()
{
    tf::Quaternion temp_quaternion;
    boost::qvm::vec< double , 3 > vec_temporary;
    //Prepare data for buoncy force
    temp_quaternion = tf::Quaternion( 0 , 0 , buoncy , 0 );
    temp_quaternion = current_quaternion.inverse() * temp_quaternion * current_quaternion;
    vec_temporary.a[0] = temp_quaternion.x();
    vec_temporary.a[1] = temp_quaternion.y();
    vec_temporary.a[2] = temp_quaternion.z();
    std::memcpy( (void*) &mat_force_buoncy.a[0][0],
            (void*) vec_temporary.a ,
            sizeof( double ) * 3 );
    vec_temporary = boost::qvm::cross( vec_temporary , 
            zeabus::robot::distance_center_buoncy );
    std::memcpy( (void*) &mat_force_buoncy.a[0][3],
            (void*) vec_temporary.a ,
            sizeof( double ) * 3 );
    //Prepare data for gravity
    temp_quaternion = tf::Quaternion( 0 , 0 , -1.0*weight , 0 );
    temp_quaternion = current_quaternion.inverse() * temp_quaternion * current_quaternion;
    vec_temporary.a[0] = temp_quaternion.x();
    vec_temporary.a[1] = temp_quaternion.y();
    vec_temporary.a[2] = temp_quaternion.z();
    std::memcpy( (void*) &mat_force_gravity.a[0][0],
            (void*) vec_temporary.a ,
            sizeof( double ) * 3 );
    vec_temporary = boost::qvm::cross( vec_temporary , 
            zeabus::robot::distance_center_gravity );
    std::memcpy( (void*) &mat_force_gravity.a[0][3],
            (void*) vec_temporary.a ,
            sizeof( double ) * 3 );
    mat_force_thruster = mat_force_individual_thruster * zeabus::robot::direction_all;
    
    mat_force_summation = mat_force_epsilon + 
            mat_force_thruster + 
            mat_force_gravity + 
//            mat_force_viscosity +
            mat_force_buoncy;

    // copy to message
    std::memcpy( (void*) message.thruster_force.c_array() ,
            (void*) mat_force_thruster.a[0],
            sizeof( double ) * 6 );
    std::memcpy( (void*) message.viscosity_force.c_array() ,
            (void*) mat_force_viscosity.a[0],
            sizeof( double ) * 6 );
    std::memcpy( (void*) message.gravity_force.c_array() ,
            (void*) mat_force_gravity.a[0],
            sizeof( double ) * 6 );
    std::memcpy( (void*) message.buoncy_force.c_array() ,
            (void*) mat_force_buoncy.a[0],
            sizeof( double ) * 6 );
    std::memcpy( (void*) message.epsilon_force.c_array() ,
            (void*) mat_force_epsilon.a[0],
            sizeof( double ) * 6 );
}

void report( ros::Publisher* publisher )
{
    publisher->publish( message );
    printf( "\n============= ROPORT ON SUBJECT 3.3\n");
    printf( "GRAVITY :" ); zeabus_boost::print( mat_force_gravity );
    printf( "BUONCY  :" ); zeabus_boost::print( mat_force_buoncy );
    printf( "THRUSTER:" ); zeabus_boost::print( mat_force_thruster );
    printf( "EPSILON :" ); zeabus_boost::print( mat_force_epsilon );
//    printf( "VISCOSITY :" ); zeabus_boost::print( mat_force_viscosity );
    printf( "SUM     :" ); zeabus_boost::print( mat_force_summation );
}
