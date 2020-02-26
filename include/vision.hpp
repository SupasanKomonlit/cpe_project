// FILE			: vision.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 20 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE
// ref01 : http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
// ref02 : https://en.cppreference.com/w/cpp/algorithm/stable_sort

// MACRO SET

// MACRO CONDITION

#include    <algotithm>
#include    <mutex>
#include    <iostream>
#include    <vector>

#include    <ros/ros.h>

#include    <zeabus/ros/node.hpp>
#include    <zeabus/ros/subscriber/base_class.hpp>

#include    <sensor_msgs/CompressedImage.h>
#include    <sensor_msgs/Image.h>

#include    <image_transport/image_transport.h>
#include    <cv_bridge/cv_bridge.h>
// Library image processing group
#include    <opencv2/core/mat.hpp>
#include    <opencv2/imgcodecs.hpp>
#include    <opencv2/imgproc.hpp>
#include    <opencv2/highgui/highgui.hpp>

#include    <dynamic_reconfigure/server.h>
#include    <cpe_project/Simple3DataConfig.h>
