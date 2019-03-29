//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: calibration_node.cpp
//
// Purpose: This node calibrates the cameras (hopefully)
//
//////////////////////////////////////////////////////////////

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream> 
#include <string>

using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::stringstream path;                               
    path << ros::package::getPath("synchronize_pointclouds");
    path << "/calibration_images/";

    if (msg->header.frame_id.find("front") != string::npos) path << "chessboard_front.jpg";
    if (msg->header.frame_id.find("back") != string::npos)  path << "chessboard_back.jpg";
    if (msg->header.frame_id.find("left") != string::npos)  path << "chessboard_left.jpg";
    if (msg->header.frame_id.find("right") != string::npos) path << "chessboard_right.jpg";
    if (msg->header.frame_id.find("top") != string::npos)   path << "chessboard_top.jpg";
                
    ROS_ASSERT(cv::imwrite(path.str(), cv_ptr->image));     
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber front_sub = it.subscribe("camera_front/color/image_raw", 1, imageCallback);
    image_transport::Subscriber back_sub = it.subscribe("camera_back/color/image_raw", 1, imageCallback);
    image_transport::Subscriber left_sub = it.subscribe("camera_left/color/image_raw", 1, imageCallback);
    image_transport::Subscriber right_sub = it.subscribe("camera_right/color/image_raw", 1, imageCallback);
    image_transport::Subscriber top_sub = it.subscribe("camera_top/color/image_raw", 1, imageCallback);

    ros::spin();
}

//////////////////////////////////////////////////////////////

