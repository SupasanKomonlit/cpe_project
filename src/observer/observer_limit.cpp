// FILE			: observer_limit.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 05 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    "observer.hpp"

void limit_data()
{
    if( ! observer_status )
    {
        goto exit_limit_data;
    }
    limit_depth();
exit_limit_data:
    return;
}

void limit_depth()
{
    if( message_observer_zeabus.pose.pose.position.z < global_min_z )
    {
        pub( "LIMIT : OVER limit max depth" , false );
        message_observer_zeabus.pose.pose.position.z = global_min_z;
        message_observer_zeabus.twist.twist.linear.z = 0;
    }
    else if( message_observer_zeabus.pose.pose.position.z > global_max_z )
    {
        pub( "LIMIT : OVER limit min depth" , false );
        message_observer_zeabus.pose.pose.position.z = global_max_z;
        message_observer_zeabus.twist.twist.linear.z = 0;
    }
    else
    {
        ;
    }
exit_limit_depth:
    return;
}
