//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: transformer_node.cpp
//
// Purpose: This node transforms the raw cloud tata into a 
// global reference frame & publishes the result
//
//////////////////////////////////////////////////////////////

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#define FRONT 1
#define BACK 2
#define LEFT 3
#define RIGHT 4
#define TOP 5

using namespace pcl;
using namespace std;
using namespace tf;

ros::Subscriber raw_cloud_front_sub;
ros::Subscriber raw_cloud_back_sub;
ros::Subscriber raw_cloud_left_sub;
ros::Subscriber raw_cloud_right_sub;
ros::Subscriber raw_cloud_top_sub;
        
ros::Publisher  rotated_cloud_front_pub;
ros::Publisher  rotated_cloud_back_pub;
ros::Publisher  rotated_cloud_left_pub;
ros::Publisher  rotated_cloud_right_pub;
ros::Publisher  rotated_cloud_top_pub;

// RAW CLOUD CALLBACK FUNCTION
// transform the pointcloud so that it is in a global reference 
// frame (world) and then republish the newly rotated point cloud
void raw_cloud_callback(sensor_msgs::PointCloud2 msg) 
{
    // put the pointcloud in a format we can process
    PointCloud<PointXYZRGB> temp;
    fromROSMsg(msg, temp);

    // figure out which camera it is
    int camera_id = -1;
    if (temp.header.frame_id.find("front") != string::npos) camera_id = FRONT;
    if (temp.header.frame_id.find("back") != string::npos) camera_id = BACK;
    if (temp.header.frame_id.find("left") != string::npos) camera_id = LEFT;
    if (temp.header.frame_id.find("right") != string::npos) camera_id = RIGHT;
    if (temp.header.frame_id.find("top") != string::npos) camera_id = TOP;

    // transform it into the world frame
    ros::Time stamp;
    pcl_conversions::toPCL(stamp, temp.header.stamp);
    TransformListener tf_listener;
    tf_listener.waitForTransform("/world", temp.header.frame_id, stamp, ros::Duration(10.0));
    pcl_ros::transformPointCloud("/world", temp, temp, tf_listener);
 
    // put the pointcloud in a format we can publish
    sensor_msgs::PointCloud2 transformed_cloud;
    toROSMsg(temp, transformed_cloud);

    // publish it
    if (camera_id == FRONT) rotated_cloud_front_pub.publish(transformed_cloud);
    if (camera_id == BACK) rotated_cloud_back_pub.publish(transformed_cloud);
    if (camera_id == LEFT) rotated_cloud_left_pub.publish(transformed_cloud);
    if (camera_id == RIGHT) rotated_cloud_right_pub.publish(transformed_cloud);
    if (camera_id == TOP) rotated_cloud_top_pub.publish(transformed_cloud);
}

int main(int argc, char * argv[]) 
{
    ros::init(argc, argv, "transformer_node");
    ros::NodeHandle handle;

    raw_cloud_front_sub = handle.subscribe("camera_front/depth_registered/points", 1, &raw_cloud_callback);
    raw_cloud_back_sub = handle.subscribe("camera_back/depth_registered/points", 1, &raw_cloud_callback);
    raw_cloud_left_sub = handle.subscribe("camera_left/depth_registered/points", 1, &raw_cloud_callback);
    raw_cloud_right_sub = handle.subscribe("camera_right/depth_registered/points", 1, &raw_cloud_callback);
    raw_cloud_top_sub = handle.subscribe("camera_top/depth_registered/points", 1, &raw_cloud_callback);

    rotated_cloud_front_pub = handle.advertise<sensor_msgs::PointCloud2>("camera_front/depth_registered/points_rotated", 1);
    rotated_cloud_back_pub = handle.advertise<sensor_msgs::PointCloud2>("camera_back/depth_registered/points_rotated", 1);
    rotated_cloud_left_pub = handle.advertise<sensor_msgs::PointCloud2>("camera_left/depth_registered/points_rotated", 1);
    rotated_cloud_right_pub = handle.advertise<sensor_msgs::PointCloud2>("camera_right/depth_registered/points_rotated", 1);
    rotated_cloud_top_pub = handle.advertise<sensor_msgs::PointCloud2>("camera_top/depth_registered/points_rotated", 1);

    ros::spin();

    return 0;
}

//////////////////////////////////////////////////////////////

