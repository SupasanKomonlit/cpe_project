// FILE			: train_model.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 05 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <cmath>
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

#include    <ros/ros.h>
#include    <std_msgs/Float64.h>
#include    <nav_msgs/Odometry.h>
#include    <zeabus_utility/Float64Array8.h>

#include    <boost/array.hpp>

#include    <boost/qvm/vec_access.hpp>
#include    <boost/qvm/vec_operations.hpp>
#include    <boost/qvm/map_mat_mat.hpp>
#include    <boost/qvm/mat_access.hpp>
#include    <boost/qvm/mat_operations.hpp>

#include    <tf/LinearMath/Quaternion.h>
#include    <tf/transform_listener.h>
#include    <tf/transform_broadcaster.h>


// Below set will prepare force frome constant force , gravity force and buoncy force
extern boost::qvm::mat< double , 6 , 1 > mat_constant_force;
extern double roll;
extern double pitch;
extern double yaw;

#ifndef _CPE_PROJECT_
#define _CPE_PROJECT_
void prepare_constant_force();
#endif

extern std::string prefix;
extern int focus;
// 0 1 2 3 4 5 is x y z roll pitch yaw in order
