// FILE			: center_function.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 21 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    "train_model.hpp"

boost::qvm::mat< double , 6 , 1 > mat_constant_force = {0,0,0,0,0,0};

void prepare_constant_force()
{
    boost::qvm::mat< double , 6 , 1 > mat_force_gravity = {0,0,0,0,0,0};
    boost::qvm::mat< double , 6 , 1 > mat_force_buoncy = {0,0,0,0,0,0};
    boost::qvm::mat< double , 6 , 1 > mat_force_estimate = {0,0,0,0,0,0};

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

    mat_constant_force = mat_force_gravity + mat_force_buoncy + mat_force_estimate;
}
