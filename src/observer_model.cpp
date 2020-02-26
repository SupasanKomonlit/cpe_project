// FILE			: observer_model.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 21 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    "observer.hpp"

boost::qvm::mat< double , 6 , 1 > mat_force_gravity = {0,0,0,0,0,0};
boost::qvm::mat< double , 6 , 1 > mat_force_buoncy = {0,0,0,0,0,0};
boost::qvm::mat< double , 6 , 1 > mat_force_estimate = {0,0,0,0,0,0};
boost::qvm::mat< double , 6 , 1 > mat_force_viscosity = {0,0,0,0,0,0};
boost::qvm::mat< double , 6 , 1 > mat_acceleration = {0,0,0,0,0,0};
double roll;
double pitch;
double yaw;

void active_model()
{
    boost::qvm::mat< double , 3 , 3 > mat_rotation_force_z = {
        0,  0,  -1.0*sin( roll ),
        0,  0,  cos( pitch ) * sin( roll ),
        0,  0,  cos( pitch ) * cos( roll )
    };

    zeabus_boost::mat_concat( &mat_force_gravity , 
            mat_rotation_force_z * zeabus::robot::mat_force_gravity,
            zeabus::robot::mat_center_gravity * 
                    mat_rotation_force_z * 
                    zeabus::robot::mat_force_gravity );

    zeabus_boost::mat_concat( &mat_force_buoncy,
            mat_rotation_force_z * zeabus::robot::mat_force_buoncy,
            zeabus::robot::mat_center_buoncy *
                    mat_rotation_force_z *
                    zeabus::robot::mat_force_buoncy );

    zeabus_boost::mat_concat( &mat_force_estimate,
            mat_rotation_force_z * zeabus::robot::mat_force_buoncy,
            zeabus::robot::mat_center_constant *
                    mat_rotation_force_z *
                    zeabus::robot::mat_force_constant );

    calculate_viscosity();

    mat_acceleration = zeabus::robot::mat_inertia_inverse * (
            mat_force_gravity +
            mat_force_buoncy + 
            mat_force_estimate +
            mat_force_viscosity +
            mat_force_thruster +
            mat_force_observer );
}

inline void calculate_viscosity()
{
    boost::qvm::A00( mat_force_viscosity ) = viscosity( 0 ); 
    boost::qvm::A10( mat_force_viscosity ) = viscosity( 1 ); 
    boost::qvm::A20( mat_force_viscosity ) = viscosity( 2 ); 
    boost::qvm::A30( mat_force_viscosity ) = viscosity( 3 ); 
    boost::qvm::A40( mat_force_viscosity ) = viscosity( 4 ); 
    boost::qvm::A50( mat_force_viscosity ) = viscosity( 5 ); 
}

inline double viscosity( unsigned int index )
{
#ifdef VISCOSITY_NONNEWTON
    return -1.0 * mat_constant_viscosity_k.a[ index ][0] * 
            ( 1 - exp( -1.0 * mat_constant_viscosity_c.a[ index ][0] * arr_robot_velocity[ index ]));
#else
    return mat_constant_viscosity_k.a[ index ][0] * arr_robot_velocity[ index ][0];
#endif // viscosity
}
