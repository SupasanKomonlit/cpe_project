// FILE			: learn_moment_inertia.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 20 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE
//  This file will assume you have velocity only yaw

// MACRO SET
#define _VISCOSITY_NEWTON_

// MACRO CONDITION

#include    <cmath>
#include    <cstring>
#include    <iostream>
#include    <zeabus/ros/node.hpp>
#include    <zeabus/math/boost/print_data.hpp>
#include    <zeabus/ros/subscriber/base_class.hpp>

#include    <ros/ros.h>
#include    <geometry_msgs/Vector3.h>
#include    <std_msgs/Float64.h>

#include    <zeabus/file.hpp>
#include    <boost/qvm/vec_access.hpp>
#include    <boost/qvm/vec_operations.hpp>

extern boost::qvm::mat< double , 3 , 3 > mat_current_inertial;
extern boost::qvm::mat< double , 3 , 3 > mat_current_viscosity;
extern boost::qvm::mat< double , 3 , 1 > mat_current_velocity;
extern boost::qvm::mat< double , 3 , 1 > mat_current_

extern boost::qvm::mat< double , 3 , 1 > mat_acceleration;
#ifndef _CPE_PROJECT__
#define _CPE_PROJECT__

#endif // _CPE_PROJECT__
