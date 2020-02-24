// FILE			: observer.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 21 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
#define VISCOSITY_NONNEWTON

// MACRO CONDITION

#include    <iostream>
#include    <cmath>
#include    <cstring>
#include    <mutex>

#include    <zeabus/robot.hpp>
#include    <zeabus/ros/node.hpp>
#include    <zeabus/math/boost/print_data.hpp>

#include    <tf/LinearMath/Quaternion.h>
#include    <tf/transform_listener.h>

#include    <ros/ros.h>
#include    <zeabus_utility/Float64Array8.h>
#include    <nav_msgs/Odometry.h>

#include    <boost/array.hpp>

#include    <boost/qvm/vec_access.hpp>
#include    <boost/qvm/mat_access.hpp>
#include    <boost/qvm/vec_operations.hpp>
#include    <boost/qvm/mat_operations.hpp>

// Part variable on observer_model.cpp
//  boost::qvm::mat< class type , int row , int column >
extern boost::qvm::mat< double , 6 , 1 > mat_force_gravity; // only k direction
extern boost::qvm::mat< double , 6 , 1 > mat_force_buoncy; // only k direction
extern boost::qvm::mat< double , 6 , 1 > mat_force_estimate; // only k direction
extern boost::qvm::mat< double , 6 , 1 > mat_force_viscosity;
extern boost::qvm::mat< double , 6 , 1 > mat_acceleration;
extern double roll;
extern double pitch;
extern double yaw;

#ifdef _CPE_PROJECT_OBSERVER_MODEL__
#define _CPE_PROJECT_OBSERVER_MODEL__
void active_model();
void calculate_viscosity();
inline double viscosity( unsigned int index );
#endif // _CPE_PROJECT_OBSERVER_MODEL__

// Part variable on observer.cpp
extern boost::qvm::mat< double , 6 , 1 > mat_constant_viscosity_k;
extern boost::qvm::mat< double , 6 , 1 > mat_constant_viscosity_c;
extern boost::qvm::mat< double , 6 , 1 > mat_force_observer;
extern boost::qvm::mat< double , 6 , 1 > mat_force_thruster;
extern boost::array< double , 6 > arr_velocity;
