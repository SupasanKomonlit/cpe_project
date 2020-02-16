// FILE			: listen_overview.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 16 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE
// ref01    : http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29

// MACRO SET

// MACRO CONDITION

#include    <mutex>
#include    <cstring>
#include    <iostream>
#include    <zeabus/ros/node.hpp>
#include    <zeabus/math/boost/print_data.hpp>
#include    <zeabus/ros/subscriber/base_class.hpp>
#include    <zeabus/ros/convert/geometry_quaternion.hpp>

#include    <tf/LinearMath/Quaternion.h>
#include    <tf/transform_listener.h>

#include    <ros/ros.h>
#include    <zeabus_utility/Float64Array8.h>

#include    <boost/qvm/vec_access.hpp>
#include    <boost/qvm/mat_access.hpp>

const double weight = zeabus_robot::gravity * zeabus_robot::mass;
const double buoncy = zeabus_robot::gravity * zeabus_robot::rho_water * zeabus_robot::volumn;

// Responsible on offside code
extern boost::qvm::mat< double , 1 , 6 > mat_force_thruster;
extern boost::qvm::mat< double , 1 , 6 > mat_force_buoncy;
extern boost::qvm::mat< double , 1 , 6 > mat_force_gravity;
extern boost::qvm::mat< double , 1 , 6 > mat_force_epsilon;
extern boost::qvm::mat< double , 1 , 6 > mat_force_viscosity;
extern boost::qvm::mat< double , 1 , 6 > mat_force_summation;
extern boost::qvm::vec< double , 6 > vec_calculate_velocity;
extern boost::qvm::vec< double , 6 > vec_calculate_acceleration;

#ifndef _CPE_PROJECT__
#define _CPE_PROJECT__
void calculate();
void report();
#endif

// Responsible on main code
extern tf::Quaternion current_quaternion;
extern std::vector< nav_msgs::Quaternion > buffer_current_state;
extern boost::qvm::vec< double , 6 > vec_current_velocity;
extern boost::qvm::mat< double , 1 , 8 > mat_force_individual_thruster;

// Don't avaliable data
extern boost::qvm::vec< double , 6 > vec_current_acceleration;
