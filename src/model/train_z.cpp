// FILE			: train_z.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 21 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
#define _TRAIN_OBSERVER_
// #define _RATE_LIMIT_
// #define _PRINT_VALUE_

// MACRO CONDITION

#include    "train_model.hpp"
// Global variable
double roll;
double pitch;
double yaw;
//  Individual file variable
int focus = 2;
std::string name_file = "data_train_z.csv"; 
std::string prefix;
double error_value;
double viscosity_k = 10;
double viscosity_c = 10;
double observer = 0;
double new_viscosity_k = 10;
double new_viscosity_c = 10;
double new_observer = 0;
double max_error = 200;
//  Global function 
bool set_prefix()
{
    bool result = true;
    switch(focus)
    {
        case 0 : prefix = "x";      break;
        case 1 : prefix = "y";      break;
        case 2 : prefix = "z";      break;
        case 3 : prefix = "roll";   break;
        case 4 : prefix = "pitch";  break;
        case 5 : prefix = "yaw";    break;
        default : result = false;
    }
    return result;
}

int main( int argv , char** argc )
{
    ( void )set_prefix(); 

//  Original start part
    zeabus_ros::Node node( argv , argc , "project_train_" + prefix );
    ros::NodeHandle nh("");
    node.spin();

//  For send data
    ros::Publisher pub_error = nh.advertise< std_msgs::Float64 >( "/project/error/" + prefix ,
            1000 );
    ros::Publisher pub_viscosity_k = nh.advertise< std_msgs::Float64 >( "/project/k/" + prefix , 
            1000 );
    ros::Publisher pub_viscosity_c = nh.advertise< std_msgs::Float64 >( "/project/c/" + prefix , 
            1000 );
    ros::Publisher pub_observer = nh.advertise< std_msgs::Float64 >( "/project/observer/"+prefix,
            1000 );

    std_msgs::Float64 message_float;

//  Get data
    zeabus::FileCSV fh; // file handle;
    fh.open( zeabus_ros::get_full_path( "cpe_project" , "data" , name_file ) );
    double *robot_velocity , *robot_acceleration , *robot_quaternion , *force_thruster;
    unsigned int number_data;
    (void)fh.count_line( &number_data );
    robot_velocity = (double*) malloc( sizeof( double ) * 6 * number_data );
    robot_acceleration = (double*) malloc( sizeof( double ) * 6 * number_data );
    robot_quaternion = (double*) malloc( sizeof( double ) * 4 * number_data );
    force_thruster = (double*) malloc( sizeof( double ) * 8 * number_data );
    std::cout   << "Processing collect " << number_data << " data\n";
    (void) zeabus::extract_csv_type_observer_train( &fh , 
            robot_velocity , 
            robot_acceleration , 
            robot_quaternion , 
            force_thruster );
    std::cout   << "Finish read file\n";
//  Next Set up constant parameter individual follow focus
    double focus_inertia = zeabus::robot::mat_inertia.a[ focus ][ focus ];
    std::cout   << "Inertial for this mode is " << focus_inertia << "\n";
    
//  Finish get data next parameter round to train

    unsigned int limit_round;
    unsigned int count_print;
    std::cout   << "Please input round of all data you want to train : ";
    std::cin    >> limit_round;
    std::cout   << "Please input round for print weight: ";
    std::cin    >> count_print;
    double learning_rate = 0.0001;
    std::cout   << "Input learning rate : ";
    std::cin    >> learning_rate;

    unsigned int round_train = 0;
    ros::Rate rate( 500 );
//  Finish set parameter about loop
    bool first_time_rotation = true;
    boost::qvm::mat< double , 1 , 8 > mat_thruster_data;
    boost::qvm::mat< double , 6 , 1 > mat_force_thruster;
//  Publish first data
    message_float.data = 0 ;
    pub_error.publish( message_float );
    message_float.data = viscosity_k;
    pub_viscosity_k.publish( message_float );
    message_float.data = viscosity_c;
    pub_viscosity_c.publish( message_float );
    message_float.data = observer;
    pub_observer.publish( message_float );
//  Start loop train
    while( ros::ok() )
    {
        int count_data = 0;
        while( ros::ok() && count_data < number_data )
        {
#ifdef _RATE_LIMIT_
            rate.sleep();
#endif
//  First We must to get roll pitch yaw and quaternion
            tf::Quaternion quaternion( robot_quaternion[ count_data * 4 + 0] ,
                    robot_quaternion[ count_data * 4 + 1 ],
                    robot_quaternion[ count_data * 4 + 2 ],
                    robot_quaternion[ count_data + 4 + 3 ] );
            tf::Matrix3x3( quaternion ).getRPY( roll , pitch , yaw );
            prepare_constant_force(); // Finish prepare data in constant term

//  Next we must rotation data into robot frame use this part when calculate on linear focus
            if( first_time_rotation )
            {
                zeabus::math::rotation( quaternion , &robot_velocity[ count_data * 6 ] );
                zeabus::math::rotation( quaternion , &robot_acceleration[ count_data * 6 ] );
            }
//  Now we finish rotation data linear and replace them in array

//          We must to calculate for of thurster in base link frame
            std::memcpy( (void*) mat_thruster_data.a ,    
                    (void*) &force_thruster[ count_data * 8 ] ,
                    sizeof( double ) * 8 );               
            mat_force_thruster = boost::qvm::transposed( mat_thruster_data   
                    * zeabus::robot::direction_all ) * zeabus::robot::gravity;

//  Start part calculate data
            double velocity = robot_velocity[ count_data * 6 + focus ];
            double predict_acceleration = mat_constant_force.a[ focus ][ 0 ] + // from force z
                    mat_force_thruster.a[ focus ][ 0 ] - // thruster force
                    viscosity_k * ( 1 - exp( -1 * viscosity_c * velocity ) ) + // vicosity force
                    observer; // obsever term
//  Start part gradient
            predict_acceleration /= focus_inertia;  // divide by inertial
            error_value = predict_acceleration - robot_acceleration[ count_data * 6 + focus ];
            message_float.data  = error_value * error_value / 2;
#ifdef _RATE_LIMIT_
            pub_error.publish( message_float );
#endif
            if( fabs( error_value ) > max_error ) 
            {
                std::cout   << "Abort data line " << count_data + 1 
                            << " have error " << fabs( error_value ) << "\n";
            }
            else if( message_float.data > 0.0006 )
            {
                new_viscosity_k = error_value * ( -1.0 ) * 
                        ( 1.0 - exp( -1 * viscosity_c * velocity ) ) / focus_inertia;
                new_viscosity_c = error_value * ( -1.0 ) * viscosity_k * velocity *
                        exp( -1 * viscosity_c * velocity ) / focus_inertia;
                new_observer = error_value / focus_inertia;
//  Update new value
                viscosity_k = viscosity_k - ( new_viscosity_k * learning_rate ); 
                viscosity_c = viscosity_c -  ( new_viscosity_c * learning_rate );
#ifdef _TRAIN_OBSERVER_ 
                observer = observer - ( new_observer * learning_rate ) ;
#endif
//  Publish new value
/*
                message_float.data = viscosity_k;
                pub_viscosity_k.publish( message_float );
                message_float.data = viscosity_c;
                pub_viscosity_c.publish( message_float );
                message_float.data = observer;
                pub_observer.publish( message_float );
*/
            }
//  Check print loop
            if( count_data % count_print == 0 )
            {
#ifndef _RATE_LIMIT_
                message_float.data  = error_value * error_value / 2;
                pub_error.publish( message_float );
#endif
                message_float.data = viscosity_k;
                pub_viscosity_k.publish( message_float );
                message_float.data = viscosity_c;
                pub_viscosity_c.publish( message_float );
                message_float.data = observer;
                pub_observer.publish( message_float );
/////////////////////////////==========SPLIT PART PRINT AND PUBLISH ==================
#ifdef _PRINT_VALUE_
                std::cout   << "Amont loop " << round_train << " : " << limit_round  
                            << "    data index " << count_data << " : " << number_data << "\n";
                std::cout   << "K Viscosity : " << viscosity_k << "\n";
                std::cout   << "C Viscosity : " << viscosity_c << "\n";
                std::cout   << "Observer : " << observer << "\n";
#endif
            }
            count_data++;
        }
        first_time_rotation = false;
//  This part will check about limit round or not
        round_train++;
        if( round_train == limit_round )
        {
            std::cout   << "enter 0 for stop train otherwise round to train : ";
            std::cin    >> limit_round;
            if( limit_round == 0 )
            {
                break;
            }
            else
            {
                round_train = 0;
                std::cout   << "Input learning rate : ";
                std::cin    >> learning_rate;
            }
        }
    }
//  End loop for train
    std::cout   << "End program\n";
    std::cout   << "K Viscosity : " << viscosity_k << "\n";
    std::cout   << "C Viscosity : " << viscosity_c << "\n";
    std::cout   << "Observer : " << observer << "\n";

//  Original end part
    ros::shutdown();
    node.join();
    return 0;
}
