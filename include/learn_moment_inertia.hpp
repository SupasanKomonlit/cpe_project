// FILE			: learn_moment_inertia.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 20 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE
//  This file will assume you have velocity only yaw

// MACRO SET
//#define _VISCOSITY_NEWTON_

// MACRO CONDITION

#include    <cmath>
#include    <cstring>
#include    <cstdlib>
#include    <iostream>
#include    <zeabus/ros/node.hpp>
#include    <zeabus/robot.hpp>
#include    <zeabus/ros/path_file.hpp>
#include    <zeabus/math/boost/print_data.hpp>
#include    <zeabus/ros/subscriber/base_class.hpp>

#include    <ros/ros.h>
#include    <geometry_msgs/Vector3.h>
#include    <std_msgs/Float64.h>

#include    <zeabus/file.hpp>
#include    <boost/qvm/vec_access.hpp>
#include    <boost/qvm/vec_operations.hpp>
#include    <boost/qvm/mat_access.hpp>
#include    <boost/qvm/mat_operations.hpp>

// Responsible on main file
extern boost::qvm::mat< double , 3 , 3 > mat_current_inertia;
extern boost::qvm::mat< double , 3 , 2 > mat_current_value_viscosity;
extern boost::qvm::mat< double , 3 , 1 > mat_current_observer_force;
extern boost::qvm::mat< double , 3 , 1 > mat_current_velocity;
extern boost::qvm::mat< double , 3 , 1 > mat_thruster_force;
extern boost::qvm::mat< double , 3 , 1 > mat_actual;
extern double roll;
extern double pitch;
extern double yaw;

// Responsible on model file
extern boost::qvm::mat< double , 3 , 3 > mat_gradient_inertia;
extern boost::qvm::mat< double , 3 , 2 > mat_gradient_value_viscosity;
extern boost::qvm::mat< double , 3 , 1 > mat_gradient_observer_force;
extern boost::qvm::mat< double , 3 , 1 > mat_gravity_force;
extern boost::qvm::mat< double , 3 , 1 > mat_buoncy_force;
extern boost::qvm::mat< double , 3 , 1 > mat_constant_force;
extern boost::qvm::mat< double , 3 , 1 > mat_viscosity_force;
extern boost::qvm::mat< double , 3 , 1 > mat_predict;
extern boost::qvm::mat< double , 3 , 1 > mat_diff; // predict - actual

#ifndef _CPE_PROJECT__
#define _CPE_PROJECT__
// before use this please setup roll pitch yaw before
void prepare_z_force();
// below function will prepare mat_current_viscosity from mat_current_value_viscosity
void prepare_viscosity();
// below function will find predict value;
void get_predict();
// below function will find gradient of inertial term
void get_gradient();
// below use to print current weight
void print();
#endif // _CPE_PROJECT__
