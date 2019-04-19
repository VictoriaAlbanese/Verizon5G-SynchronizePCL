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

using namespace pcl;
using namespace std;
using namespace tf;

ros::Subscriber raw_cloud1_sub;
ros::Subscriber raw_cloud2_sub;
ros::Subscriber raw_cloud3_sub;
ros::Subscriber raw_cloud4_sub;
ros::Subscriber raw_cloud5_sub;
ros::Subscriber raw_cloud6_sub;
ros::Subscriber raw_cloud7_sub;
ros::Subscriber raw_cloud8_sub;
ros::Subscriber raw_cloud9_sub;
        
ros::Publisher  rotated_cloud1_pub;
ros::Publisher  rotated_cloud2_pub;
ros::Publisher  rotated_cloud3_pub;
ros::Publisher  rotated_cloud4_pub;
ros::Publisher  rotated_cloud5_pub;
ros::Publisher  rotated_cloud6_pub;
ros::Publisher  rotated_cloud7_pub;
ros::Publisher  rotated_cloud8_pub;
ros::Publisher  rotated_cloud9_pub;

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
    if (temp.header.frame_id.find("camera_1") != string::npos) camera_id = 1;
    if (temp.header.frame_id.find("camera_2") != string::npos) camera_id = 2;
    if (temp.header.frame_id.find("camera_3") != string::npos) camera_id = 3;
    if (temp.header.frame_id.find("camera_4") != string::npos) camera_id = 4;
    if (temp.header.frame_id.find("camera_5") != string::npos) camera_id = 5;
    if (temp.header.frame_id.find("camera_6") != string::npos) camera_id = 6;
    if (temp.header.frame_id.find("camera_7") != string::npos) camera_id = 7;
    if (temp.header.frame_id.find("camera_8") != string::npos) camera_id = 8;
    if (temp.header.frame_id.find("camera_9") != string::npos) camera_id = 9;

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
    if (camera_id == 1) rotated_cloud1_pub.publish(transformed_cloud);
    if (camera_id == 2) rotated_cloud2_pub.publish(transformed_cloud);
    if (camera_id == 3) rotated_cloud3_pub.publish(transformed_cloud);
    if (camera_id == 4) rotated_cloud4_pub.publish(transformed_cloud);
    if (camera_id == 5) rotated_cloud5_pub.publish(transformed_cloud);
    if (camera_id == 6) rotated_cloud6_pub.publish(transformed_cloud);
    if (camera_id == 7) rotated_cloud7_pub.publish(transformed_cloud);
    if (camera_id == 8) rotated_cloud8_pub.publish(transformed_cloud);
    if (camera_id == 9) rotated_cloud9_pub.publish(transformed_cloud);
}

int main(int argc, char * argv[]) 
{
    ros::init(argc, argv, "transformer_node");
    ros::NodeHandle handle;

    raw_cloud1_sub = handle.subscribe("camera_1/depth_registered/points", 1, &raw_cloud_callback);
    raw_cloud2_sub = handle.subscribe("camera_2/depth_registered/points", 1, &raw_cloud_callback);
    raw_cloud3_sub = handle.subscribe("camera_3/depth_registered/points", 1, &raw_cloud_callback);
    raw_cloud4_sub = handle.subscribe("camera_4/depth_registered/points", 1, &raw_cloud_callback);
    raw_cloud5_sub = handle.subscribe("camera_5/depth_registered/points", 1, &raw_cloud_callback);
    raw_cloud6_sub = handle.subscribe("camera_6/depth_registered/points", 1, &raw_cloud_callback);
    raw_cloud7_sub = handle.subscribe("camera_7/depth_registered/points", 1, &raw_cloud_callback);
    raw_cloud8_sub = handle.subscribe("camera_8/depth_registered/points", 1, &raw_cloud_callback);
    raw_cloud9_sub = handle.subscribe("camera_9/depth_registered/points", 1, &raw_cloud_callback);

    rotated_cloud1_pub = handle.advertise<sensor_msgs::PointCloud2>("camera_1/depth_registered/points_rotated", 1);
    rotated_cloud2_pub = handle.advertise<sensor_msgs::PointCloud2>("camera_2/depth_registered/points_rotated", 1);
    rotated_cloud3_pub = handle.advertise<sensor_msgs::PointCloud2>("camera_3/depth_registered/points_rotated", 1);
    rotated_cloud4_pub = handle.advertise<sensor_msgs::PointCloud2>("camera_4/depth_registered/points_rotated", 1);
    rotated_cloud5_pub = handle.advertise<sensor_msgs::PointCloud2>("camera_5/depth_registered/points_rotated", 1);
    rotated_cloud6_pub = handle.advertise<sensor_msgs::PointCloud2>("camera_6/depth_registered/points_rotated", 1);
    rotated_cloud7_pub = handle.advertise<sensor_msgs::PointCloud2>("camera_7/depth_registered/points_rotated", 1);
    rotated_cloud8_pub = handle.advertise<sensor_msgs::PointCloud2>("camera_8/depth_registered/points_rotated", 1);
    rotated_cloud9_pub = handle.advertise<sensor_msgs::PointCloud2>("camera_9/depth_registered/points_rotated", 1);

    ros::spin();

    return 0;
}

//////////////////////////////////////////////////////////////

