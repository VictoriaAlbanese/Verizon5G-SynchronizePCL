//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: calibration_node.cpp
//
// Purpose: This node calibrates the cameras (hopefully)
//
//////////////////////////////////////////////////////////////

#include <bits/stdc++.h> 
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream> 
#include <string>
#include <vector>

#define WIDTH 4
#define HEIGHT 3

using namespace std;
using namespace cv;

bool FRONT = false;    
bool BACK = false;    
bool LEFT = false;    
bool RIGHT = false;    
bool TOP = false;    

string windowName = "kaiergfykauegfaueygfa";

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

    if (msg->header.frame_id.find("front") != string::npos) 
    {   
        path << "chessboard_front.jpg";
        FRONT = true;
    }

    if (msg->header.frame_id.find("back") != string::npos) 
    {
        path << "chessboard_back.jpg";
        BACK = true;
    }

    if (msg->header.frame_id.find("left") != string::npos)  
    {
        path << "chessboard_left.jpg";
        LEFT = true;
    }

    if (msg->header.frame_id.find("right") != string::npos) 
    {
        path << "chessboard_right.jpg";
        RIGHT = true;
    }

    if (msg->header.frame_id.find("top") != string::npos)   
    {
        path << "chessboard_top.jpg";
        TOP = true;
    }
                
    ROS_ASSERT(cv::imwrite(path.str(), cv_ptr->image));     
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);


    // Get the images for calibrating ----------------------------

    image_transport::Subscriber front_sub = it.subscribe("camera_front/color/image_raw", 1, imageCallback);
    image_transport::Subscriber back_sub = it.subscribe("camera_back/color/image_raw", 1, imageCallback);
    image_transport::Subscriber left_sub = it.subscribe("camera_left/color/image_raw", 1, imageCallback);
    image_transport::Subscriber right_sub = it.subscribe("camera_right/color/image_raw", 1, imageCallback);
    image_transport::Subscriber top_sub = it.subscribe("camera_top/color/image_raw", 1, imageCallback);

    while (!(FRONT && BACK && LEFT && RIGHT && TOP)) ros::spinOnce();


    // Get the object points from the 5 cameras-------------------

    vector<Point3f> left_object_points; 
    for (int y = 0; y < HEIGHT; y++)
        for (int x = 0; x < WIDTH; x++)
            left_object_points.push_back(Point3f(float(x), float(y), float(0)));

    vector<Point3f> right_object_points; 
    for (int y = HEIGHT; y > 0; y--)
        for (int x = WIDTH; x > 0; x--)
            left_object_points.push_back(Point3f(float(x-1), float(y-1), float(0)));

    vector<Point3f> front_object_points; 
    for (int y = 0; y < WIDTH; y++)
        for (int x = HEIGHT; x > 0; x--)
            left_object_points.push_back(Point3f(float(x-1), float(y), float(0)));

    vector<Point3f> back_object_points; 
    for (int y = WIDTH; y > 0; y--)
        for (int x = 0; x < HEIGHT; x++)
            left_object_points.push_back(Point3f(float(x), float(y-1), float(0)));

    vector<vector<Point3f> > object_points;
    object_points.push_back(front_object_points);
    object_points.push_back(back_object_points);
    object_points.push_back(left_object_points);
    object_points.push_back(right_object_points);
    object_points.push_back(left_object_points);  // left orientation same as top


    // Find image points ------------------------------------------
    
    vector<vector<Point2f> > image_points;
    vector<Point2f> temp;      
    Mat image; 
    Size board_size(WIDTH, HEIGHT);

    vector<string> filelist;
    filelist.push_back("chessboard_front.jpg");
    filelist.push_back("chessboard_back.jpg");
    filelist.push_back("chessboard_left.jpg");
    filelist.push_back("chessboard_right.jpg");
    filelist.push_back("chessboard_top.jpg");
 
    std::stringstream path;                               
    path << ros::package::getPath("synchronize_pointclouds");
    path << "/calibration_images/";

    for (int i = 0; i < filelist.size(); i++) 
    {
        image = imread(path.str().c_str() + filelist[i], 0);
        bool found = findChessboardCorners(image, board_size, temp);

        if (found)  
        {
            image_points.push_back(temp);
            cornerSubPix(image, image_points[i], Size(5, 5), Size(-1, -1), TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1)); 
        }
    
        if (image_points[i].size() == board_size.area()) 
        {
            drawChessboardCorners(image, board_size, image_points[i], found);
            imshow(windowName, image);
            waitKey(100);
        }
    }

    return 0;
}

//////////////////////////////////////////////////////////////

