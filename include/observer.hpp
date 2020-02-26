// FILE			: observer.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 21 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
#define VISCOSITY_NONNEWTON
//#define UPDATED_ALL
#define UPDATED_Z
#define UPDATED_IMU
//#define UPDATED_VISION

// MACRO CONDITION
#ifdef UPDATED_ALL
    #define UPDATED_Z
    #define UPDATED_VISION
    #define UPDATED_IMU
#endif

#include    <cmath>
#include    <mutex>
#include    <cstring>
#include    <cstdlib>
#include    <iostream>

#include    <zeabus/file.hpp>
#include    <zeabus/robot.hpp>
#include    <zeabus/ros/node.hpp>
#include    <zeabus/ros/path_file.hpp>
#include    <zeabus/math/quaternion.hpp>
#include    <zeabus/math/boost/operations.hpp>
#include    <zeabus/math/boost/print_data.hpp>
#include    <zeabus/ros/subscriber/base_class.hpp>
#include    <zeabus/ros/convert/geometry_quaternion.hpp>

#include    <tf/LinearMath/Quaternion.h>
#include    <tf/transform_listener.h>
#include    <tf/transform_broadcaster.h>

#include    <ros/ros.h>
#include    <std_msgs/String.h>
#include    <nav_msgs/Odometry.h>
#include    <zeabus_utility/Float64Array8.h>

#include    <boost/array.hpp>

#include    <boost/qvm/vec_access.hpp>
#include    <boost/qvm/mat_access.hpp>
#include    <boost/qvm/map_mat_mat.hpp>
#include    <boost/qvm/vec_operations.hpp>
#include    <boost/qvm/mat_operations.hpp>

const double global_max_z = 0.1;
const double global_min_z = -5.0;
const double global_diff_z = 0.2;
const double global_diff_z_half = global_diff_z / 2;
const double global_active_observer = -0.5;

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

#ifndef _CPE_PROJECT_OBSERVER_MODEL__
#define _CPE_PROJECT_OBSERVER_MODEL__
void active_model();
inline void calculate_viscosity();
inline double viscosity( unsigned int index );
#endif // _CPE_PROJECT_OBSERVER_MODEL__

// Part variable on observer_update.cpp
extern double local_max_z;
extern double local_min_z;
extern bool observer_status;
#ifndef _CPE_PROJECT_OBSERVER_UPDATE__
#define _CPE_PROJECT_OBSERVER_UPDATE__
void updated_data();
void updated_depth();
#endif // _CPE_PROJECT_OBSERVER_UPDATE__

#ifndef _CPE_PROJECT_OBSERVER_GRADIENT__
#define _CPE_PROJECT_OBSERVER_GRADIENT__

#endif // _CPE_PROJECT_OBSERVER_GRADIENT__

// Part variable on observer.cpp
extern boost::qvm::mat< double , 6 , 1 > mat_constant_viscosity_k;
extern boost::qvm::mat< double , 6 , 1 > mat_constant_viscosity_c;
extern boost::qvm::mat< double , 6 , 1 > mat_force_observer;
extern boost::qvm::mat< double , 6 , 1 > mat_force_thruster;
extern boost::array< double , 6 > arr_robot_velocity;
extern boost::array< double , 6 > arr_world_velocity;
extern ros::Time force_stamp;
extern ros::Time state_stamp;
extern ros::Time vision_stamp;
extern ros::Time time_stamp;
extern nav_msgs::Odometry message_localize_zeabus;
extern nav_msgs::Odometry message_observer_zeabus;
extern ros::Publisher publisher_message;
extern tf::Quaternion current_quaternion;

#ifndef _CPE_PROJECT_OBSERVER__
#define _CPE_PROJECT_OBSERVER__
void pub( const std::string message );
#endif 
