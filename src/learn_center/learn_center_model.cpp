// FILE			: learn_center_model.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 20 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    "learn_center.hpp"

double term_alpha( const boost::qvm::vec< double , 9 > data )
{
    return  (-1.0*data.a[1]*cos_beta_cos_alpha + 1.0*data.a[2]*cos_beta_sin_alpha )*weight +
            (+1.0*data.a[4]*cos_beta_cos_alpha - 1.0*data.a[5]*cos_beta_sin_alpha )*buoncy +
            (-1.0*data.a[7]*cos_beta_cos_alpha + 1.0*data.a[8]*cos_beta_sin_alpha )*estimate -
            0 +
            thruster_roll;  
}

double term_beta( const boost::qvm::vec< double , 9 > data )
{
    return  (+1.0*data.a[2]*sin_beta + 1.0*data.a[0]*cos_beta_cos_alpha )*weight +
            (-1.0*data.a[5]*sin_beta - 1.0*data.a[3]*cos_beta_cos_alpha )*buoncy +
            (+1.0*data.a[8]*sin_beta + 1.0*data.a[6]*cos_beta_cos_alpha )*estimate -
            0 +
            thruster_pitch;
}

double term_gamma( const boost::qvm::vec< double , 9 > data 
        , const boost::qvm::vec< double , 2 > constant_velocity )
{
    return  (-1.0*data.a[0]*cos_beta_sin_alpha - 1.0*data.a[2]*sin_beta )*weight +
            (+1.0*data.a[3]*cos_beta_sin_alpha + 1.0*data.a[5]*sin_beta )*buoncy +
            (-1.0*data.a[0]*cos_beta_sin_alpha - 1.0*data.a[8]*sin_beta )*estimate -
            term_viscosity( constant_velocity.a[0] , constant_velocity.a[1] ) +
            thruster_yaw;
}

double term_viscosity( double k , double c )
{
    double answer;
    if( type_viscosity )
    {
        answer = k * velocity_yaw; 
    }
    else
    {
        answer = k - k * exp( -1.0 * c * velocity_yaw ); 
    } 
    return answer;
}

double gradient_viscosity( unsigned int mode )
{
    double answer;
    if( type_viscosity )
    {
        answer = term_gamma( vec_current_center , vec_current_viscosity ) * velocity_yaw;
    }
    else if( mode == 0 )
    {
        answer = term_gamma( vec_current_center , vec_current_viscosity );
        answer = answer + answer * exp( -1.0 * vec_current_viscosity.a[ 1 ] * velocity_yaw ); 
    }
    else
    {
        answer =    -1.0 * term_gamma( vec_current_center , vec_current_viscosity ) * 
                    exp( -1.0 * vec_current_viscosity.a[ 1 ] * velocity_yaw ) * velocity_yaw;
    }
    return answer;
}

double gradient_x( double force )
{
    return  term_beta( vec_current_center ) * -1.0 * force * cos_beta_cos_alpha +
            term_gamma( vec_current_center , vec_current_viscosity ) * force * cos_beta_sin_alpha;
}

double gradient_y( double force )
{
    return  term_alpha( vec_current_center ) * force * cos_beta_cos_alpha +
            term_gamma( vec_current_center , vec_current_viscosity ) * force * sin_beta;
}

double gradient_z( double force )
{
    return  term_alpha( vec_current_center ) * -1.0 * force * cos_beta_cos_alpha +
            term_beta( vec_current_center ) * -1.0 * force * sin_beta;
}
