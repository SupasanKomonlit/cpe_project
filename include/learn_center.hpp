// FILE			: learn_center.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 20 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE
//  This file will assume you have velocity only yaw

// MACRO SET

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

// About function require most parameter to active function
// Order on center of gravity (CG), center of buoncy (CB) and center of error (CE)
// And suborder is x y z
extern boost::qvm::vec< double , 9 > vec_current_center;
extern boost::qvm::vec< double , 9 > vec_new_center;
extern boost::qvm::vec< double , 9 > vec_gradient_center;

extern boost::qvm::vec< double , 2 > vec_current_viscosity;
extern boost::qvm::vec< double , 2 > vec_new_viscosity;
extern boost::qvm::vec< double , 2 > vec_gradient_viscosity;

extern double cos_beta_cos_alpha;
extern double cos_beta_sin_alpha;
extern double sin_beta;

extern double weight;
extern double buoncy;
extern double estimate;
extern double velocity_yaw;
extern bool type_viscosity; // true is newton characteristic

extern double thruster_roll;
extern double thruster_pitch;
extern double thruster_yaw;

#ifndef _CPE_PROJECT__
#define _CPE_PROJECT__
const double gravity = 9.806;
const double rho = 1027;
double term_alpha( const boost::qvm::vec< double , 9 > data );
double term_beta( const boost::qvm::vec< double , 9 > data );
double term_gamma( const boost::qvm::vec< double , 9 > data 
        , const boost::qvm::vec< double , 2 > constant_velocity );
double term_viscosity( double k , double c = 1 );
double gradient_x( double force );
double gradient_y( double force );
double gradient_z( double force );
double gradient_viscosity( unsigned int mode = 0 );
#endif // _CPE_PROJECT__
