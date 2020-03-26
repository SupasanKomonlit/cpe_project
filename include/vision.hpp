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

#include    <algorithm>
#include    <mutex>
#include    <iostream>
#include    <vector>

#include    <ros/ros.h>

#include    <zeabus/ros/node.hpp>
#include    <zeabus/ros/subscriber/base_class.hpp>
#include    <zeabus/ros/dynamic_reconfigure.hpp>
#include    <zeabus/ros/convert/geometry_quaternion.hpp>
#include    <zeabus/ros/convert/geometry_vector3.hpp>

#include    <zeabus/opencv.hpp>

#include    <sensor_msgs/CompressedImage.h>
#include    <sensor_msgs/Image.h>

#include    <image_transport/image_transport.h>
#include    <cv_bridge/cv_bridge.h>
// Library image processing group
#include    <opencv2/core/mat.hpp>
#include    <opencv2/imgcodecs.hpp>
#include    <opencv2/imgproc.hpp>
#include    <opencv2/highgui/highgui.hpp>
#include    <opencv2/calib3d.hpp>

#include    <dynamic_reconfigure/server.h>
#include    <cpe_project/Simple3DataConfig.h>

#include    <zeabus_utility/VisionResults.h>
#include    <zeabus_utility/VisionResult.h>

#ifndef _CPE_PROJECT_VISION__
#define _CPE_PROJECT_VISION__
inline zeabus_utility::VisionResult vision_result( const ros::Time& stamp,
        const std::string& frame_id,
        const std::string& child_frame_id,
        const tf::Quaternion& rotation,
        const tf::Vector3&  translation )
{
    zeabus_utility::VisionResult data;
    data.stamp = stamp;
    data.frame_id = frame_id;
    data.child_frame_id = child_frame_id;
    zeabus_ros::convert::geometry_quaternion::tf( &rotation , &data.rotation ); 
    zeabus_ros::convert::geometry_vector3::tf( &translation , &data.translation ); 
    return data;
}

inline zeabus_utility::VisionResult vision_result( const ros::Time& stamp,
        const std::string& frame_id,
        const std::string& child_frame_id,
        const cv::Mat_< double >& rotation,
        const cv::Mat_< double >& translation )
{
    zeabus_utility::VisionResult data;
    data.stamp = stamp;
    data.frame_id = frame_id;
    data.child_frame_id = child_frame_id;
    tf::Quaternion temp_quaternion;
    temp_quaternion.setRPY( *rotation[0] , *rotation[1] , *rotation[2] );
    zeabus_ros::convert::geometry_quaternion::tf( &temp_quaternion , &data.rotation );
    data.translation.x = *translation[0];
    data.translation.y = *translation[1];
    data.translation.z = *translation[2];
    return data;
}
#endif // _CPE_PROJECT_VISION__
