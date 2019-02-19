////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: cloud_class.cpp
//
// Purpose: Implements a class which handles operations on the 
// pointclouds created by the Intel RealSense D435i sensors
//
////////////////////////////////////////////////////////////////

#include "cloud_class.hpp"

////////////////////////////////////////////////////////////////

// Public Members

// DEFAULT CONSTRUCTOR
// does not do ros initialization
Cloud::Cloud() 
{
    this->c1_initialized = false;
    this->c2_initialized = false;
    this->c3_initialized = false;
}

// CONSTRUCTOR
// does the ros initialization
Cloud::Cloud(ros::NodeHandle handle) 
{
    this->c1_initialized = false;
    this->c2_initialized = false;
    this->c3_initialized = false;

    this->cloud_pub = handle.advertise<sensor_msgs::PointCloud2>("combined_cloud", 1);
    this->cloud1_sub = handle.subscribe("cam_1/depth_registered/points_filtered", 1, &Cloud::cloud1_callback, this);
    this->cloud2_sub = handle.subscribe("cam_2/depth_registered/points_filtered", 1, &Cloud::cloud2_callback, this);
    this->cloud3_sub = handle.subscribe("cam_3/depth_registered/points_filtered", 1, &Cloud::cloud3_callback, this);

    while (!this->initialized()) ros::spinOnce();
}

// PUBLISH MASTER CLOUD FUNCTION
// this function converts the pcl XYZRGB cloud to a 
// sensor_msg PointCloud2 for easy publishing
void Cloud::publish_master_cloud() 
{
    sensor_msgs::PointCloud2 cloud;
    toROSMsg(this->master_cloud, cloud);

    this->concatenate_clouds();
    this->cloud_pub.publish(cloud);
}

////////////////////////////////////////////////////////////////

// Private Members & Callbacks

// CLOUD1 CALLBACK FUNCTION
// initialize cloud1 with the information from cam_1
// converts the cloud message to an pcl XYBRGB pointcloud 
void Cloud::cloud1_callback(const sensor_msgs::PointCloud2 msg) 
{
    fromROSMsg(msg, this->cloud1);
    this->c1_initialized = true;
    
    ROS_INFO("Cloud 1 is in frame %s", this->cloud1.header.frame_id.c_str());
}

// CLOUD2 CALLBACK FUNCTION
// initialize cloud1 with the information from cam_2
// converts the cloud message to an pcl XYBRGB pointcloud 
void Cloud::cloud2_callback(const sensor_msgs::PointCloud2 msg) 
{
    fromROSMsg(msg, this->cloud2);
    this->c2_initialized = true;
    
    ROS_INFO("Cloud 2 is in frame %s", this->cloud2.header.frame_id.c_str());
}

// CLOUD3 CALLBACK FUNCTION
// initialize cloud1 with the information from cam_3
// converts the cloud message to an pcl XYBRGB pointcloud 
void Cloud::cloud3_callback(const sensor_msgs::PointCloud2 msg) 
{
    fromROSMsg(msg, this->cloud3);
    this->c3_initialized = true;
    
    ROS_INFO("Cloud 3 is in frame %s", this->cloud3.header.frame_id.c_str());
}

// CONCATENATE CLOUDS FUNCTION
// concatenates the (points | fields) of all the 
// cloud members into the master pointcloud
void Cloud::concatenate_clouds() 
{
    // this concatenates the points
    this->master_cloud = this->cloud1;
    this->master_cloud+= this->cloud2;
    this->master_cloud+= this->cloud3;

    ROS_INFO("MasterC is in frame %s", this->master_cloud.header.frame_id.c_str());
}

////////////////////////////////////////////////////////////////

