// FILE			: learn_moment_inertia_model.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 20 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    "learn_moment_inertia.hpp"

boost::qvm::mat< double , 3 , 1 > mat_gravity_force = { 0 , 0 , 0 };
boost::qvm::mat< double , 3 , 1 > mat_buoncy_force = { 0 , 0 , 0 };
boost::qvm::mat< double , 3 , 1 > mat_constant_force = { 0 , 0 , 0 };
boost::qvm::mat< double , 3 , 1 > mat_viscosity_force = { 0 , 0 , 0 };
boost::qvm::mat< double , 3 , 3 > mat_gradient_inertia = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 3 , 2 > mat_gradient_value_viscosity = { 1 , 1 , 1 , 1 , 1 , 1 };
boost::qvm::mat< double , 3 , 1 > mat_gradient_observer_force = {0 , 0 , 0 };
boost::qvm::mat< double , 3 , 1 > mat_predict = {0 , 0 , 0 };
boost::qvm::mat< double , 3 , 1 > mat_diff = {0 , 0 , 0 };


void prepare_z_force()
{
    double cos_beta_cos_alpha = cos( pitch ) * cos( roll );
    double cos_beta_sin_alpha = cos( pitch ) * sin( roll );
    double sin_beta = sin( pitch );

    boost::qvm::mat< double , 3 , 3 > mat_rotation_force = 
        {   0 , 0 , -sin_beta ,
            0 , 0 , cos_beta_sin_alpha ,
            0 , 0 , cos_beta_cos_alpha };
    
    mat_gravity_force = zeabus::robot::mat_center_gravity * 
            mat_rotation_force *
            zeabus::robot::mat_force_gravity;

    mat_buoncy_force = zeabus::robot::mat_center_buoncy *
            mat_rotation_force *
            zeabus::robot::mat_force_gravity;

    mat_constant_force = zeabus::robot::mat_center_constant *
            mat_rotation_force *
            zeabus::robot::mat_force_constant;
} // prepare_z_force

void prepare_viscosity()
{
    for( unsigned int run = 0 ; run <  3 ; run++ )
    {
#ifdef _VISCOSITY_NEWTON_
        mat_viscosity_force.a[run][0] = mat_current_value_viscosity.a[run][0] * 
                mat_current_velocity.a[run][0] * -1.0;
#else
        mat_viscosity_force.a[run][0] = mat_current_value_viscosity.a[run][0] *
                ( 1 - exp( -1.0 * mat_current_value_viscosity.a[run][ 1 ] ) )* 
                mat_current_velocity.a[run][0] * -1.0;
#endif
    }
} // function prepare_viscosity

void get_predict()
{
    boost::qvm::mat< double , 3 , 3 > mat_inverse_inertia = {
        1.0/mat_current_inertia.a[0][0] , 0 , 0,
        0 , 1.0/mat_current_inertia.a[1][1] , 0,
        0 , 0, 1.0/mat_current_inertia.a[2][2]
    };

    mat_predict = mat_inverse_inertia * ( mat_gravity_force + 
            mat_buoncy_force + 
            mat_constant_force +
            mat_viscosity_force + 
            mat_thruster_force + 
            mat_current_observer_force );

    mat_diff = mat_predict - mat_actual;
} // function get_piredict

void get_gradient()
{
    // First step for gradient about moment of inertia
    boost::qvm::mat< double , 3 , 3 > mat_inverse_inertia = {
        pow( mat_current_inertia.a[0][0] , -2 ), 0 , 0 ,
        0 , pow( mat_current_inertia.a[1][1] , -2 ) , 0 ,
        0 , 0 , pow( mat_current_inertia.a[2][2] , -2 ) 
    };
    mat_gradient_inertia.a[0][0] = -1.0 * mat_inverse_inertia.a[0][0] * 
            mat_predict.a[0][0] * mat_diff.a[0][0];
    mat_gradient_inertia.a[1][1] = -1.0 * mat_inverse_inertia.a[1][1] * 
            mat_predict.a[1][0] * mat_diff.a[1][0];
    mat_gradient_inertia.a[2][2] = -1.0 * mat_inverse_inertia.a[2][2] * 
            mat_predict.a[2][0] * mat_diff.a[2][0];

    mat_inverse_inertia.a[0][0] = 1.0/mat_current_inertia.a[0][0];
    mat_inverse_inertia.a[1][1] = 1.0/mat_current_inertia.a[1][1];
    mat_inverse_inertia.a[2][2] = 1.0/mat_current_inertia.a[2][2];

    // Second step for gradient about viscosity
    for( unsigned int run = 0 ; run < 3 ; run++ )
    {
#ifdef _VISCOSITY_NEWTON_
        mat_gradient_value_viscosity.a[run][0] = -1.0 * mat_diff.a[run][0] * 
                mat_current_velocity.a[run][0] /
                mat_current_inertia.a[run][run];
#else
        mat_gradient_value_viscosity.a[run][0] =-1.0 * mat_diff.a[run][0] * 
                ( 1 - exp( -1.0 * mat_current_value_viscosity.a[run][1]   *
                mat_current_velocity.a[run][0] ) ) /
                mat_current_inertia.a[run][run];

        mat_gradient_value_viscosity.a[run][1] = -1.0 * mat_diff.a[run][0] * 
                exp( -1.0 * mat_current_velocity.a[run][0] * mat_current_value_viscosity.a[run][1] )*
                mat_current_value_viscosity.a[run][0] *
                mat_current_velocity.a[run][0] /
                mat_current_inertia.a[run][run];
#endif // _VISCOSITY_NEWTON_ 
    }

    // Third step for gradient about bias term
    mat_gradient_observer_force = mat_inverse_inertia * mat_diff;
}

void print()
{
    std::cout   << "INERTIA   :\n"; zeabus_boost::print( mat_current_inertia );
    std::cout   << "VISCOSITY :\n"; zeabus_boost::print( mat_current_value_viscosity );
    std::cout   << "OBSERVER  :\n"; zeabus_boost::print( mat_current_observer_force );
    std::cout   << "END REPORTED====================================================\n";
}
