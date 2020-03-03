// FILE			: vision_front.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 26 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE
//  ref01 : http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
//  ref02 : https://docs.opencv.org/3.4/d3/d63/classcv_1_1Mat.html
//  ref03 : https://en.cppreference.com/w/cpp/container/vector
//  ref04 : https://docs.opencv.org/master/d8/d01/group__imgproc__color__conversions.html#gga4e0972be5de079fed4e3a10e24ef5ef0a95d70bf0c1b5aa58d1eb8abf03161f21
//  start : http://wiki.ros.org/image_transport#Published_topics
//  ref05 : http://wiki.ros.org/compressed_image_transport
//  ref05 : https://stackoverflow.com/questions/26218280/thresholding-rgb-image-in-opencv

// MACRO SET
#define _USE_SIM_TIME_

// MACRO CONDITION

#include    "vision.hpp"

std::mutex lock_image;
std::mutex lock_config;
cv::Mat message_image_mat;
std_msgs::Header message_image_header;
unsigned int upper_first = 95;
unsigned int upper_second = 214;
unsigned int upper_third = 199;
unsigned int down_first = 51;
unsigned int down_second = 107;
unsigned int down_third = 30;
const double min_area = 50;
const double min_ratio = 0.8; // max is 1
const double max_variance = 200;
bool dump_data = false;

void dynamic_reconfigure_callback( cpe_project::Simple3DataConfig &config , uint32_t level )
{
    int level_data = (int)level;
    if( level_data == -1 )
    {
        std::cout   << "Start config?\n";
        config.upper_first = upper_first;
        config.upper_second = upper_second;
        config.upper_third = upper_third;
        config.down_first = down_first;
        config.down_second = down_second;
        config.down_third = down_third;
    }
    else
    {
        lock_config.lock();
        upper_first = config.upper_first;
        upper_second = config.upper_second;
        upper_third = config.upper_third;
        down_first = config.down_first;
        down_second = config.down_second;
        down_third = config.down_third;
        std::cout   << "Upper :" << upper_first << " : " 
                    << upper_second << " : " 
                    << upper_third << "\n";
        std::cout   << "Down  :" << down_first << " : " 
                    << down_second << " : " 
                    << down_third << "\n";
        if( level_data == 1 )
        {
            dump_data = true;
        }
        lock_config.unlock();
    }
}

void imageCallback( const sensor_msgs::ImageConstPtr& msg )
{
    lock_image.lock();
    message_image_mat = cv_bridge::toCvShare( msg , "rgb8" )->image ;
    message_image_header = msg->header;
    lock_image.unlock();
}

int main( int argv , char** argc )
{

    zeabus_ros::Node node( argv , argc , "project_vision_front" );

    ros::NodeHandle nh( "" );

    node.spin();

    dynamic_reconfigure::Server< cpe_project::Simple3DataConfig > server_reconfigure;
    dynamic_reconfigure::Server< cpe_project::Simple3DataConfig >::CallbackType function_reconfigure;
    function_reconfigure = boost::bind( &dynamic_reconfigure_callback , _1 , _2 );
    server_reconfigure.setCallback( function_reconfigure );
    zeabus_ros::DynamicReconfigure drh; // dynamic reconfigur handle

    // _image_transport:=compressed
    image_transport::ImageTransport it( nh );
    image_transport::Subscriber sub = it.subscribe( "/vision/front/image_rect_color",
            1 , imageCallback );
    sensor_msgs::ImagePtr message_publish;
    image_transport::Publisher pub_hsv = it.advertise( "/project/front/hsv" , 1 );
    image_transport::Publisher pub_the = it.advertise( "/project/front/threshold" , 1 );
    image_transport::Publisher pub_cnt = it.advertise( "/project/front/contours" , 1 );
    ros::Publisher publish_results = nh.advertise< zeabus_utility::VisionResults >( 
            "/vision/results" , 1 );
    std::vector< zeabus_utility::VisionResult > data_results;
    zeabus_utility::VisionResults message_results;

    ros::Rate rate( 10 );
    ros::Time time_stamp = ros::Time::now();

    std_msgs::Header header;
    cv::Mat image_hsv;
    cv::Mat image_current;
    cv::Mat image_threshold;
    cv::Mat image_contours;
    bool new_message = false;

    std::vector< std::vector< cv::Point > > contours;
    std::vector< cv::Vec4i > hierachy;

    drh.load( "cpe_project" , "parameter" , "front.yaml" , ros::this_node::getName() );

    std::vector< cv::Point2f > vec_center;    
    zeabus_opencv::structure::LineRect rect_line;

    std::string temp_text;
    
    while( ros::ok() )
    {
        data_results.clear();
        rate.sleep();
        lock_image.lock();
        if( time_stamp != message_image_header.stamp )
        {
            image_current = message_image_mat;
#ifndef _USE_SIM_TIME_
            header.stamp = message_image_header.stamp;
#else
            header.stamp = ros::Time::now();
#endif
            header.frame_id = "front_camera_optical";
            new_message = true;
        }
        time_stamp = header.stamp;
        lock_image.unlock();

        if( ! new_message ) continue;

        cv::cvtColor( image_current , image_hsv , cv::COLOR_BGR2HSV );
        message_publish = cv_bridge::CvImage( header , "rgb8" , image_hsv ).toImageMsg();
        pub_hsv.publish( message_publish );
        lock_config.lock();
        cv::inRange( image_hsv , 
                cv::Scalar( down_first , down_second , down_third ),
                cv::Scalar( upper_first , upper_second , upper_third ), 
                image_threshold );
        if( dump_data )
        {
            drh.dump( "cpe_project" , "parameter" , "front.yaml" , ros::this_node::getName() );
            dump_data = false;
        }
        lock_config.unlock();
        message_publish = cv_bridge::CvImage( header , "mono8" , image_threshold ).toImageMsg();
        pub_the.publish( message_publish );

        cv::findContours( image_threshold , contours , hierachy ,
                cv::RETR_TREE , cv::CHAIN_APPROX_SIMPLE  );
        cv::cvtColor( image_current , image_contours , cv::COLOR_BGR2RGB );
        for( auto it = contours.begin() ; it != contours.end() ; )
        {
            double area = cv::contourArea( *it , false );
            cv::RotatedRect rect = cv::minAreaRect( *it );
            if( min_ratio > zeabus_opencv::operations::ratio( rect.size ) )
            {
                it = contours.erase( it );
            }
            else if( min_area > area )
            {
                it = contours.erase( it );
            }
            else
            {
//                std::cout   << "Area " << area << "\n";
                ++it;
            }
        }
//        std::cout   << "After cut condition remain " << contours.size() << "\n";
        cv::drawContours( image_contours , contours , -1 , cv::Scalar( 255 , 0 , 0 ) ,
                cv::FILLED );
        
        std::vector< zeabus_opencv::structure::Circle > vec_circle;
        cv::Point2f temp_point;
        float temp_radius;
        for( unsigned int run = 0 ; run < contours.size() ; run++ )
        {
            cv::minEnclosingCircle( contours.at( run ) , temp_point , temp_radius );
            vec_circle.push_back( zeabus_opencv::structure::Circle( temp_point , temp_radius ) );
        }

        zeabus_opencv::sort::cpp_sort( &vec_circle );
        int min_index = -1;
        double min_variance = max_variance; 
        cv::Mat_<double> rotation_vector, translation_vector;
        std::vector< cv::Point3_< double > > vec_object;
        for( int run = 0 ; run < ( ( int )vec_circle.size() )- 3 ; run++ )
        {
            double mean = ( vec_circle.at( run ).radius + 
                    vec_circle.at( run + 1 ).radius +
                    vec_circle.at( run + 2 ).radius +
                    vec_circle.at( run + 3 ).radius ) / 4.0 ;
            double variance = ( pow( mean - vec_circle.at( run ).radius , 2 ) +
                    pow( mean - vec_circle.at( run + 1 ).radius , 2 ) +
                    pow( mean - vec_circle.at( run + 2 ).radius , 2 ) +
                    pow( mean - vec_circle.at( run + 3 ).radius , 2 ) / 4.0 ) ;
            if( variance <= min_variance )
            {
                min_variance = variance;
                min_index = run;
            }
//            std::cout   << "Variance of index " << run << " is " << variance << "\n";            
        }

        if( min_index == -1 )
        {
            goto finish_find_box;
        }

        vec_circle.assign( vec_circle.begin() + min_index , vec_circle.begin() + min_index + 4 );
//        std::cout   << "Radius :";
        for( auto it = vec_circle.begin() ; it != vec_circle.end() ; it++ )
        {
//            std::cout   << " " << it->radius;
            cv::circle( image_contours , it->center , it->radius ,
                    cv::Scalar( 0 , 255 , 255 ) , 10 , cv::LINE_8 );
        }
//        std::cout   << "\n";
        vec_center.clear();
        zeabus_opencv::operations::pull_center( vec_circle , &vec_center );
        zeabus_opencv::sort::center( &vec_center );
        // First to consider 0 1 is have min y
        unsigned int temp01, temp02 , temp03 , temp04 , temp;
        if( vec_center.at( 0 ).y < vec_center.at( 1 ).y )
        {
            temp01 = 0; temp02 = 1; temp03 = 2; temp04 = 3;
        }
        else
        {
            temp01 = 1; temp02 = 0; temp03 = 2; temp04 = 3;
        }
        if( vec_center.at( temp03 ).y < vec_center.at( temp01 ).y )
        {
            temp = temp03;
            temp03 = temp02;
            temp02 = temp01;
            temp01 = temp;   
        }
        else if( vec_center.at( temp03 ).y < vec_center.at( temp02 ).y )
        {
            temp = temp03;
            temp03 = temp02;
            temp02 = temp;
        }
        else ; 
        if( vec_center.at( temp04 ).y < vec_center.at( temp01 ).y )
        {
            temp = temp04;
            temp04 = temp02;
            temp02 = temp01;
            temp01 = temp;   
        }
        else if( vec_center.at( temp04 ).y < vec_center.at( temp02 ).y )
        {
            temp = temp04;
            temp03 = temp02;
            temp02 = temp;
        }
        else ;
        if( vec_center.at( temp01 ).x < vec_center.at( temp02 ).x )
        {
            rect_line.bl = vec_center.at( temp01 );
            rect_line.br = vec_center.at( temp02 );
        }
        else
        {
            rect_line.bl = vec_center.at( temp02 );
            rect_line.br = vec_center.at( temp01 );
        }
        if( vec_center.at( temp03 ).x < vec_center.at( temp04 ).x )
        {
            rect_line.tl = vec_center.at( temp03 );
            rect_line.tr = vec_center.at( temp04 );
        }
        else
        {
            rect_line.tl = vec_center.at( temp04 );
            rect_line.tr = vec_center.at( temp03 );
        }
        
        cv::line( image_contours , rect_line.bl , rect_line.tl,
                cv::Scalar( 255 , 255 , 0 ) , 5 , cv::LINE_8 );
        cv::line( image_contours , rect_line.bl , rect_line.br,
                cv::Scalar( 255 , 255 , 0 ) , 5 , cv::LINE_8 );
        cv::line( image_contours , rect_line.tr , rect_line.tl,
                cv::Scalar( 255 , 255 , 0 ) , 5 , cv::LINE_8 );
        cv::line( image_contours , rect_line.tr , rect_line.br,
                cv::Scalar( 255 , 255 , 0 ) , 5 , cv::LINE_8 );

        vec_object.push_back( cv::Point3_< double >( -255.11811024 , -274.01574803, 0 ) );
        vec_object.push_back( cv::Point3_< double >( -255.11811024 , +274.01574803, 0 ) );
        vec_object.push_back( cv::Point3_< double >( +255.11811024 , +274.01574803, 0 ) );
        vec_object.push_back( cv::Point3_< double >( +255.11811024 , -274.01574803, 0 ) );
        
        cv::solvePnP( vec_object , rect_line.get_vector() , 
                zeabus_opencv::front::mat_camera,
                zeabus_opencv::front::mat_distor,
                rotation_vector,
                translation_vector );

        std::cout   << "Rotation vector row " << rotation_vector.rows
                    << " and col " << rotation_vector.cols << "\n";
        zeabus_opencv::convert::to_m( translation_vector[0] );
        zeabus_opencv::convert::to_m( translation_vector[1] );
        zeabus_opencv::convert::to_m( translation_vector[2] );
        printf( "%10.3f,%10.3f,%10.3f\n", *rotation_vector[ 0 ] , *rotation_vector[ 1 ],
                *rotation_vector[ 2 ] );
        std::cout   << "Translation vector row " << translation_vector.rows
                    << " and col " << translation_vector.cols << "\n";
        printf( "%10.3f,%10.3f,%10.3f\n", *translation_vector[ 0 ] , *translation_vector[ 1 ],
                *translation_vector[ 2 ] );

        data_results.push_back( vision_result( header.stamp , 
                header.frame_id,
                "front_sign",
                rotation_vector,
                translation_vector ) );
        message_results.data = data_results;
        publish_results.publish( message_results ); 

        cv::rectangle( image_contours , 
                cv::Point_< int >( 0 , 0 ),
                cv::Point_< int >( 600 , 200 ),
                cv::Scalar_< unsigned int >( 255 , 0 , 0 ),
                cv::FILLED , cv::LINE_8 );
        temp_text = "X = " + std::to_string( *translation_vector[ 0 ] );
        cv::putText( image_contours, temp_text, cv::Point_< double >( 0 , 60 ) , 
                cv::FONT_HERSHEY_SIMPLEX , 2 , 
                cv::Scalar_< unsigned int >( 255 , 255 , 255 ) , 3 , cv::FILLED , false );
        temp_text = "Y = " + std::to_string( *translation_vector[ 1 ] );
        cv::putText( image_contours, temp_text, cv::Point_< double >( 0 , 130 ) , 
                cv::FONT_HERSHEY_SIMPLEX , 2 , 
                cv::Scalar_< unsigned int >( 255 , 255 , 255 ) , 3 , cv::FILLED , false );
        temp_text = "Z = " + std::to_string( *translation_vector[ 2 ] );
        cv::putText( image_contours, temp_text, cv::Point_< double >( 0 , 200 ) , 
                cv::FONT_HERSHEY_SIMPLEX , 2 , 
                cv::Scalar_< unsigned int >( 255 , 255 , 255 ) , 3 , cv::FILLED , false );
finish_find_box:
        message_publish = cv_bridge::CvImage( header , "bgr8" , image_contours ).toImageMsg();
        pub_cnt.publish( message_publish );
    }

    ros::shutdown();
    node.join();
}
