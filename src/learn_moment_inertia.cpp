// FILE			: learn_moment_inertia.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 20 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    "learn_moment_inertia.hpp"
boost::qvm::mat< double , 3 , 3 > mat_current_inertia = {
    0.1,  0,  0,
    0,  0.1,  0,
    0,  0,  0.1
};

boost::qvm::mat< double , 3 , 2 > mat_current_value_viscosity = { 1 , 1 , 1 , 1 ,1 , 1 };
boost::qvm::mat< double , 3 , 1 > mat_current_observer_force = {0 , 0 , 0 };
boost::qvm::mat< double , 3 , 1 > mat_current_velocity = {0 , 0 , 0 };
boost::qvm::mat< double , 3 , 1 > mat_thruster_force = { 0 , 0 , 0};
boost::qvm::mat< double , 3 , 1 > mat_actual = {0 , 0 , 0 };
double roll = 0;
double pitch = 0;
double yaw = 0;

int main( int argv , char** argc )
{
    zeabus_ros::Node node( argv , argc , "project_learning" );
    ros::NodeHandle nh("");
    node.spin();
    // finish setup node

    ros::Publisher pub_error = nh.advertise< std_msgs::Float64 >( "/project/train/error" , 1000 );
    ros::Publisher pub_inertia = nh.advertise< geometry_msgs::Vector3 >( "/project/train/inertia" ,
            1000 );
    ros::Publisher pub_viscosity_k = nh.advertise< geometry_msgs::Vector3 >( "/project/train/k" , 
            1000 );
    ros::Publisher pub_viscosity_c = nh.advertise< geometry_msgs::Vector3 >( "/project/train/c" , 
            1000 );
    ros::Publisher pub_observer = nh.advertise< geometry_msgs::Vector3 >( "/porject/train/observer",
            1000 );

    std_msgs::Float64 message_float;
    geometry_msgs::Vector3 message_vector;

    // prapare data for training
    zeabus::FileCSV fh; // file handle
    fh.open( zeabus_ros::get_full_path( "cpe_project" , "data" , "project_4_1_3_sample.csv") );
    double *thruster_force, *euler_state, *angular_vel, *angular_accel;
    unsigned int number_data;
    (void)fh.count_line( &number_data );
    thruster_force = (double*) malloc( sizeof( double ) * 6 * number_data );
    euler_state = (double*) malloc( sizeof( double ) * 3 * number_data );
    angular_vel = (double*) malloc( sizeof( double ) * 3 * number_data );
    angular_accel = (double*) malloc( sizeof( double ) * 3 * number_data );
    std::cout   << "Processing collect " << number_data << " data\n";
    (void) zeabus::extract_csv_type_15( &fh , thruster_force , euler_state , 
            angular_vel , angular_accel );
    std::cout   << "Finish read file\n";
    unsigned int limit_round;
    unsigned int count_print;
    std::cout   << "Please input round of all data you want to train : ";
    std::cin    >> limit_round;
    std::cout   << "Please input round for print weight: ";
    std::cin   >> count_print;
    double learning_rate = 0.0001;
    std::cout   << "Input learning rate : ";
    std::cin    >> learning_rate;

    unsigned int round_train = 0;
    ros::Rate rate( 500 );
    
    message_float.data = 0;
    pub_error.publish( message_float );
    message_vector.x = mat_current_inertia.a[0][0];
    message_vector.y = mat_current_inertia.a[1][1];
    message_vector.z = mat_current_inertia.a[2][2];
    pub_inertia.publish( message_vector );
    message_vector.x = mat_current_observer_force.a[0][0];
    message_vector.y = mat_current_observer_force.a[1][0];
    message_vector.z = mat_current_observer_force.a[2][0];
    pub_observer.publish( message_vector );
    message_vector.x = mat_current_value_viscosity.a[0][0];
    message_vector.y = mat_current_value_viscosity.a[1][0];
    message_vector.z = mat_current_value_viscosity.a[2][0];
    pub_viscosity_k.publish( message_vector );
    message_vector.x = mat_current_value_viscosity.a[0][1];
    message_vector.y = mat_current_value_viscosity.a[1][1];
    message_vector.z = mat_current_value_viscosity.a[2][1];
    pub_viscosity_c.publish( message_vector );

    while( ros::ok() )
    {
        int count_data = 0;
        bool updated_gradient = false;
        while( ros::ok() && count_data < number_data )
        {
            rate.sleep();
            // pull data roll pitch yaw
            roll = euler_state[ ( count_data * 3 ) + 0 ];
            pitch = euler_state[ ( count_data * 3 ) + 1 ];
            yaw = euler_state[ ( count_data * 3 ) + 2 ];
            // pull data thruster 
            mat_thruster_force.a[0][0] = thruster_force[ ( count_data * 6 ) + 3 ];
            mat_thruster_force.a[1][0] = thruster_force[ ( count_data * 6 ) + 4 ];
            mat_thruster_force.a[2][0] = thruster_force[ ( count_data * 6 ) + 5 ];
            // pull angular acceleration
            mat_actual.a[0][0] = angular_accel[ ( count_data * 3 ) + 0 ];
            mat_actual.a[1][0] = angular_accel[ ( count_data * 3 ) + 1 ];
            mat_actual.a[2][0] = angular_accel[ ( count_data * 3 ) + 2 ];
            // pull angular velocity
            mat_current_velocity.a[0][0] = angular_vel[ ( count_data * 3 ) + 0 ]; 
            mat_current_velocity.a[1][0] = angular_vel[ ( count_data * 3 ) + 1 ]; 
            mat_current_velocity.a[2][0] = angular_vel[ ( count_data * 3 ) + 2 ];
            // calculate error
            prepare_z_force();
            prepare_viscosity();
            get_predict();
            message_float.data = ( pow( mat_diff.a[0][0] , 2 ) +
                    pow( mat_diff.a[1][0] , 2 ) +
                    pow( mat_diff.a[2][0] , 2 ) ) / 2;
            if( message_float.data > 0.0006 )
            {
                updated_gradient = true;
                get_gradient(); // get gradient
//                std::cout   <<  "Finsh get gradient\n";
//                mat_current_observer_force -= (learning_rate * mat_gradient_observer_force );
                mat_current_value_viscosity -= (learning_rate * mat_gradient_value_viscosity );
                mat_current_inertia -= (learning_rate * mat_gradient_inertia );
//                std::cout   <<  "Finish updated gradient\n";
                // publish new weight
                pub_error.publish( message_float );
                message_vector.x = mat_current_inertia.a[0][0];
                message_vector.y = mat_current_inertia.a[1][1];
                message_vector.z = mat_current_inertia.a[2][2];
                pub_inertia.publish( message_vector );
                message_vector.x = mat_current_observer_force.a[0][0];
                message_vector.y = mat_current_observer_force.a[1][0];
                message_vector.z = mat_current_observer_force.a[2][0];
                pub_observer.publish( message_vector );
                message_vector.x = mat_current_value_viscosity.a[0][0];
                message_vector.y = mat_current_value_viscosity.a[1][0];
                message_vector.z = mat_current_value_viscosity.a[2][0];
                pub_viscosity_k.publish( message_vector );
                message_vector.x = mat_current_value_viscosity.a[0][1];
                message_vector.y = mat_current_value_viscosity.a[1][1];
                message_vector.z = mat_current_value_viscosity.a[2][1];
                pub_viscosity_c.publish( message_vector );
            }
            if( count_data % count_print == 0 )
            {
                std::cout   << "Amont loop " << round_train << " : " << limit_round  
                            << "    data index " << count_data << " : " << number_data << "\n";
                print();
            }
            count_data++;
        }

        round_train++;
        if( round_train == limit_round )
        {
            std::cout   << "enter 0 for stop train otherwise round to train : ";
            std::cin    >> limit_round;
            break;
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
}
