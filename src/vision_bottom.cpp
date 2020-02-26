// FILE			: vision_bottom.cpp
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
const double min_area = 1000;
bool dump_data = false;


bool find_rectangle( std::vector< cv::Point2f > center , double* index )
{
    bool result;
    if( center.size() < 4 )
    {
        result = false;
    }
    

exit_find_rectangle:
    return result;
}

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
    message_image_mat = cv_bridge::toCvShare( msg , "bgr8" )->image ;
    message_image_header = msg->header;
    lock_image.unlock();
}

int main( int argv , char** argc )
{

    zeabus_ros::Node node( argv , argc , "project_vision_bottom" );

    ros::NodeHandle nh( "" );

    node.spin();

    dynamic_reconfigure::Server< cpe_project::Simple3DataConfig > server_reconfigure;
    dynamic_reconfigure::Server< cpe_project::Simple3DataConfig >::CallbackType function_reconfigure;
    function_reconfigure = boost::bind( &dynamic_reconfigure_callback , _1 , _2 );
    server_reconfigure.setCallback( function_reconfigure );
    zeabus_ros::DynamicReconfigure drh; // dynamic reconfigur handle

    // _image_transport:=compressed
    image_transport::ImageTransport it( nh );
    image_transport::Subscriber sub = it.subscribe( "/vision/bottom/image_raw",
            1 , imageCallback );
    sensor_msgs::ImagePtr message_publish;
    image_transport::Publisher pub_hsv = it.advertise( "/project/bottom/hsv" , 1 );
    image_transport::Publisher pub_the = it.advertise( "/project/bottom/threshold" , 1 );
    image_transport::Publisher pub_cnt = it.advertise( "/project/bottom/contours" , 1 );

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

    drh.load( "cpe_project" , "parameter" , "bottom.yaml" , ros::this_node::getName() );
    
    while( ros::ok() )
    {
        rate.sleep();
        lock_image.lock();
        if( time_stamp < message_image_header.stamp )
        {
            image_current = message_image_mat;
            header.stamp = message_image_header.stamp;
            header.frame_id = "bottom_camera_optical";
            new_message = true;
        }
        time_stamp = message_image_header.stamp;
        lock_image.unlock();

        if( ! new_message ) continue;

        cv::cvtColor( image_current , image_hsv , cv::COLOR_BGR2HSV );
        message_publish = cv_bridge::CvImage( header , "bgr8" , image_hsv ).toImageMsg();
        pub_hsv.publish( message_publish );
        lock_config.lock();
        cv::inRange( image_hsv , 
                cv::Scalar( down_first , down_second , down_third ),
                cv::Scalar( upper_first , upper_second , upper_third ), 
                image_threshold );
        if( dump_data )
        {
            drh.dump( "cpe_project" , "parameter" , "bottom.yaml" , ros::this_node::getName() );
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
            if( min_area > area )
            {
                it = contours.erase( it );
            }
            else
            {
//                std::cout   << "Area " << area << "\n";
                ++it;
            }
        }
        std::cout   << "Min Area Contours size is " << contours.size() << "\n";
        std::vector< cv::Point2f > center_circle;
        std::vector< float > radius_circle;
        cv::Point2f temp_point;
        float temp_radius;
        for( unsigned int run = 0 ; run < contours.size() ; run++ )
        {
            cv::minEnclosingCircle( contours.at( run ) , temp_point , temp_radius );
            center_circle.push_back( temp_point );
            radius_circle.push_back( temp_radius );
/*
            std::cout   << "Center " << center_circle.at( run ).x 
                        << " " << center_circle.at( run ).y  
                        << " have radius " << radius_circle.at( run ) << "\n";
*/
        }
        cv::drawContours( image_contours , contours , -1 , cv::Scalar( 255 , 0 , 0 ) ,
                cv::FILLED );
        
        message_publish = cv_bridge::CvImage( header , "rgb8" , image_contours ).toImageMsg();
        pub_cnt.publish( message_publish );
    }

    ros::shutdown();
    node.join();
}
