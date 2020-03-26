// FILE			: learn_center.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 20 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    "learn_center.hpp"

const double volumn = 0.04;
const double mass = 35.5;
const double alpha = -0.0173577;
const double beta = -0.0002139;

double weight = -1.0*mass * gravity;
double buoncy = rho * gravity * volumn;
double estimate = -26.34255;
double velocity_yaw = +0.0224557;

bool type_viscosity = true; // true is newton characteristic

double cos_beta_cos_alpha = cos( beta ) * cos( alpha );
double cos_beta_sin_alpha = cos( beta ) * sin( alpha );
double sin_beta = sin( beta );

double thruster_roll = 0.0083073;
double thruster_pitch = 0.01423;
double thruster_yaw = 0;

boost::qvm::vec< double , 9 > vec_current_center = {
        0,0,0, // center of gravity
        0,0,0.1, // center of buoncy
        0,0,0}; // center of estimate
boost::qvm::vec< double , 9 > vec_new_center = vec_current_center;
boost::qvm::vec< double , 9 > vec_gradient_center = {0,0,0,0,0,0,0,0,0};

boost::qvm::vec< double , 2 > vec_current_viscosity = { 1 , 1 };
boost::qvm::vec< double , 2 > vec_new_viscosity = vec_current_viscosity;
boost::qvm::vec< double , 2 > vec_gradient_viscosity = { 0 , 0 };

int main( int argv , char** argc )
{
    zeabus_ros::Node node( argv , argc , "project_learning" );

    ros::NodeHandle nh( "" );

    node.spin();
    // finish setup node

    ros::Publisher pub_error = nh.advertise< std_msgs::Float64 >( "/project/train/error" , 1000 );
    ros::Publisher pub_cg = nh.advertise< geometry_msgs::Vector3 >( "/project/train/cg" , 1000 );
    ros::Publisher pub_cb = nh.advertise< geometry_msgs::Vector3 >( "/project/train/cb" , 1000 );
    ros::Publisher pub_ce = nh.advertise< geometry_msgs::Vector3 >( "/project/train/co" , 1000 );
    ros::Publisher pub_vi = nh.advertise< geometry_msgs::Vector3 >( "/project/train/vi" , 1000 );

    unsigned long int round = 0;
    unsigned long int limit_round;
    unsigned long int limit_publish;
    bool ok_error = false;
    double ok_value;
    bool print_gradient = false;
    double learning_rate = 0.000001;
    bool always_publish = true;
    std_msgs::Float64 message_error;
    geometry_msgs::Vector3 message_center;
    std::cout   << "Weight   : " << weight << " N\n";
    std::cout   << "Buoncy   : " << buoncy << " N\n";
    std::cout   << "Estimate : " << estimate << " N\n";
    std::cout   << "cos_beta_cos_alpha : " << cos_beta_cos_alpha << "\n";
    std::cout   << "cos_beta_sin_alpha : " << cos_beta_sin_alpha << "\n";
    std::cout   << "sin_beta           : " << sin_beta << "\n";
    std::cout   << "Please enter round for train : ";
    std::cin    >> limit_round;
    std::cout   << "Please enter round for publish : ";
    std::cin    >> limit_publish;
    std::cout   << "Please enter ok error for cost function : ";
    std::cin    >> ok_value;

    message_error.data = ( pow( term_alpha( vec_current_center ) , 2 ) + 
            pow( term_beta( vec_current_center ) , 2 ) +
            pow( term_gamma( vec_current_center , vec_current_viscosity ) , 2 ) ) /2.0;
    pub_error.publish( message_error );
    message_center.x = boost::qvm::A0( vec_current_center );
    message_center.y = boost::qvm::A1( vec_current_center );
    message_center.z = boost::qvm::A2( vec_current_center );
    pub_cg.publish( message_center );
    message_center.x = boost::qvm::A3( vec_current_center );
    message_center.y = boost::qvm::A4( vec_current_center );
    message_center.z = boost::qvm::A5( vec_current_center );
    pub_cb.publish( message_center );
    message_center.x = boost::qvm::A6( vec_current_center );
    message_center.y = boost::qvm::A7( vec_current_center );
    message_center.z = boost::qvm::A8( vec_current_center );
    pub_ce.publish( message_center );
    message_center.x = boost::qvm::A0( vec_current_viscosity );
    message_center.y = boost::qvm::A1( vec_current_viscosity );
    message_center.z = 0;
    pub_vi.publish( message_center );

    // main loop
    ros::Rate rate( 100 );
    while( round <  limit_round && ros::ok() )
    {
        rate.sleep();
        round++;

        // calculate gradient part of center ================================================
        boost::qvm::A0( vec_gradient_center ) = gradient_x( weight );
        boost::qvm::A1( vec_gradient_center ) = gradient_y( weight );
        boost::qvm::A2( vec_gradient_center ) = gradient_z( weight );
        boost::qvm::A3( vec_gradient_center ) = gradient_x( buoncy );
        boost::qvm::A4( vec_gradient_center ) = gradient_y( buoncy );
        boost::qvm::A5( vec_gradient_center ) = gradient_z( buoncy );
        boost::qvm::A6( vec_gradient_center ) = gradient_x( estimate );
        boost::qvm::A7( vec_gradient_center ) = gradient_y( estimate );
        boost::qvm::A8( vec_gradient_center ) = gradient_z( estimate );
        // calculate gradient part of viscosity
        if( type_viscosity )
        {
            boost::qvm::A0( vec_gradient_viscosity ) = gradient_viscosity( 0 );
        }
        else
        {
            boost::qvm::A0( vec_gradient_viscosity ) = gradient_viscosity( 0 );
            boost::qvm::A1( vec_gradient_viscosity ) = gradient_viscosity( 1 );
        }

        // Finish find gradient I will update data ===============================================
        vec_new_center = vec_current_center - vec_gradient_center * learning_rate;
        vec_new_viscosity = vec_current_viscosity - vec_gradient_viscosity * learning_rate;

        vec_current_center = vec_new_center;
        vec_current_viscosity = vec_new_viscosity;
        // Check error on cost function
        message_error.data = ( pow( term_alpha( vec_current_center ) , 2 ) + 
                pow( term_beta( vec_current_center ) , 2 ) +
                pow( term_gamma( vec_current_center , vec_current_viscosity ) , 2 ) ) /2.0;
        if( message_error.data < ok_value ) ok_error = true;

        if( round % limit_publish == 0 || round == limit_round || ok_error || always_publish )
        {
            pub_error.publish( message_error );
            message_center.x = boost::qvm::A0( vec_current_center );
            message_center.y = boost::qvm::A1( vec_current_center );
            message_center.z = boost::qvm::A2( vec_current_center );
            pub_cg.publish( message_center );
            message_center.x = boost::qvm::A3( vec_current_center );
            message_center.y = boost::qvm::A4( vec_current_center );
            message_center.z = boost::qvm::A5( vec_current_center );
            pub_cb.publish( message_center );
            message_center.x = boost::qvm::A6( vec_current_center );
            message_center.y = boost::qvm::A7( vec_current_center );
            message_center.z = boost::qvm::A8( vec_current_center );
            pub_ce.publish( message_center );
            message_center.x = boost::qvm::A0( vec_current_viscosity );
            message_center.y = boost::qvm::A1( vec_current_viscosity );
            message_center.z = 0;
            pub_vi.publish( message_center );
        }


        if( round % limit_publish == 0 )
        {
            std::cout   << "Now train round " << round << " / " << limit_round << "\n";
        }
       
        if( print_gradient )
        { 
            std::cout   << "Gradient center : "; zeabus_boost::print( vec_gradient_center );
            std::cout   << "Gradient viscos : "; zeabus_boost::print( vec_gradient_viscosity );
        }

        if( round == limit_round || ok_error )
        {
            std::cout   << "Result Center: "; zeabus_boost::print( vec_new_center );
            std::cout   << "Result Viscos: "; zeabus_boost::print( vec_new_viscosity );
            if( ok_error )
            {
                std::cout   << "Abort on error is ok\n";
                break;
            }
            std::cout   << "Finish train you want to train again? ( 0 = No , Other round train ) : ";
            std::cin    >> limit_round;
            if( limit_round == 0 )
            {
                break;
            }
            else
            {
                round = 0;
            }
        }
    }// main loop
} // main function
