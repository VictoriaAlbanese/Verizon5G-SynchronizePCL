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
	this->cloud1_sub = handle.subscribe("cam_1/depth_registered/points", 1, &Cloud::cloud1_callback, this);
	this->cloud2_sub = handle.subscribe("cam_2/depth_registered/points", 1, &Cloud::cloud2_callback, this);
	this->cloud3_sub = handle.subscribe("cam_3/depth_registered/points", 1, &Cloud::cloud3_callback, this);

    while (!this->initialized()) ros::spinOnce();
}

// CONCATENATE CLOUDS FUNCTION
// concatenates the (points | fields) of all the 
// cloud members into the master pointcloud
void Cloud::concatenate_clouds() 
{
	PointCloud<PointXYZ> xyz_master_cloud;
	PointCloud<PointXYZ> xyz_cloud1;
	PointCloud<PointXYZ> xyz_cloud2;
	PointCloud<PointXYZ> xyz_cloud3;

	fromROSMsg(this->cloud1, xyz_cloud1);
	fromROSMsg(this->cloud2, xyz_cloud2);
	fromROSMsg(this->cloud3, xyz_cloud3);

	xyz_master_cloud = xyz_cloud1;
	xyz_master_cloud+= xyz_cloud2;
	xyz_master_cloud+= xyz_cloud3;

    toROSMsg(xyz_master_cloud, this->master_cloud);
}

////////////////////////////////////////////////////////////////

// Private Members & Callbacks

// CLOUD1 CALLBACK FUNCTION
// gets the information from the pointcloud associated with cam_1
// adds that information to the master pointcloud & sets c1 as initialzied
void Cloud::cloud1_callback(const sensor_msgs::PointCloud2 msg) 
{
    this->cloud1 = msg;
	this->c1_initialized = true;
	
	ROS_INFO("Cloud 1 is of size %zu", this->cloud1.data.size());
}

// CLOUD2 CALLBACK FUNCTION
// gets the information from the pointcloud associated with cam_2
// adds that information to the master pointcloud & sets c2 as initialzied
void Cloud::cloud2_callback(const sensor_msgs::PointCloud2 msg) 
{
    this->cloud2 = msg;
	this->c2_initialized = true;
	
	ROS_INFO("Cloud 2 is of size %zu", this->cloud2.data.size());
}

// CLOUD3 CALLBACK FUNCTION
// gets the information from the pointcloud associated with cam_3
// adds that information to the master pointcloud & sets c3 as initialzied
void Cloud::cloud3_callback(const sensor_msgs::PointCloud2 msg) 
{
    this->cloud3 = msg;
	this->c3_initialized = true;
	
	ROS_INFO("Cloud 3 is of size %zu", this->cloud3.data.size());
}

////////////////////////////////////////////////////////////////

