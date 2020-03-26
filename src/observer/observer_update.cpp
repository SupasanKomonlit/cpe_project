// FILE			: observer_update.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 21 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
//#define _UPDATE_DEPTH_

// MACRO CONDITION

#include    "observer.hpp"

double local_max_z = 0;
double local_min_z = 0;
bool observer_status = false;

void updated_data()
{
    // First step we must update z position to consider active or not
    if( message_localize_zeabus.pose.pose.position.z > global_active_observer )
    {
        if( observer_status ) 
        {
            pub( "UPDATED: Observer stop process" , true );
        }
        observer_status = false;
        local_max_z = local_min_z = 0;
        goto exit_updated_data;
    }
    else if( ! observer_status )
    {
        observer_status = true;
        local_max_z = message_localize_zeabus.pose.pose.position.z + global_diff_z_half;
        local_min_z = message_localize_zeabus.pose.pose.position.z - global_diff_z_half;
        message_observer_zeabus.pose.pose.position.z = message_localize_zeabus.pose.pose.position.z;
        message_observer_zeabus.pose.pose.position.x = 0;
        message_observer_zeabus.pose.pose.position.y = 0;
        message_observer_zeabus.twist = message_localize_zeabus.twist;
        arr_world_velocity[ 3 ] = message_localize_zeabus.twist.twist.angular.x;
        arr_world_velocity[ 4 ] = message_localize_zeabus.twist.twist.angular.y;
        arr_world_velocity[ 5 ] = message_localize_zeabus.twist.twist.angular.z;
        pub( "UPDATED: Observer start process" , true );
    }
    else
    {
        ;
    }
#ifdef _UPDATE_DEPTH_
    updated_depth();
#endif
   
exit_updated_data:
    return;
}

void updated_depth()
{
    // First step we must check current data of localize 
    if( message_localize_zeabus.pose.pose.position.z > global_max_z )
    {
        pub( "UPDATED: position z over global_max_z" );
    }
    else if( message_localize_zeabus.pose.pose.position.z > local_max_z )
    {
        local_max_z = message_localize_zeabus.pose.pose.position.z;
        if( local_max_z - local_min_z > global_diff_z )
        {
            local_min_z = local_max_z - global_diff_z;
        }
    }
    else if( message_localize_zeabus.pose.pose.position.z < global_min_z )
    {
        pub( "UPDATED: position z over global_min_z" );
    }
    else if( message_localize_zeabus.pose.pose.position.z < local_min_z )
    {
        local_min_z = message_localize_zeabus.pose.pose.position.z;
        if( local_max_z - local_min_z > global_diff_z )
        {
            local_max_z = local_min_z + global_diff_z;
        }
    }
    else
    {
        ;
    }
    // Next We have to dicision about state of observer
    double local_mean_z = ( local_max_z + local_min_z )/2.0;
    if( fabs( message_observer_zeabus.pose.pose.position.z - local_mean_z ) > global_diff_z )
    {
        pub( "UPDATED: observer z bound to local mean" );
        message_observer_zeabus.pose.pose.position.z = local_mean_z;    
    }
    else if( message_observer_zeabus.pose.pose.position.z > local_mean_z + global_diff_z_half )
    {
        message_observer_zeabus.pose.pose.position.z = local_max_z;
    }
    else if( message_observer_zeabus.pose.pose.position.z < local_mean_z - global_diff_z_half )
    {
        message_observer_zeabus.pose.pose.position.z = local_min_z;
    }
    else
    {
        ;
    }
    // Finish part updated z value
exit_update_depth:
    return;
}
