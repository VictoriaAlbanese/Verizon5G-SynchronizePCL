////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: cloud_class.hpp
//
// Purpose: Declares a class which handles operations on the 
// pointclouds created by the Intel RealSense D435i sensors
//
////////////////////////////////////////////////////////////////

#ifndef CLOUD_CLASS_HPP
#define CLOUD_CLASS_HPP

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace pcl;

class Cloud 
{
    private: 

        // members
        bool c1_initialized;
        bool c2_initialized;
        bool c3_initialized;
	PointCloud<PointXYZRGB> master_cloud;
	PointCloud<PointXYZRGB> cloud1;
	PointCloud<PointXYZRGB> cloud2;
	PointCloud<PointXYZRGB> cloud3;
        ros::Publisher  cloud_pub;
        ros::Subscriber cloud1_sub;
        ros::Subscriber cloud2_sub;
        ros::Subscriber cloud3_sub;

        // functions
    	void cloud1_callback(const sensor_msgs::PointCloud2 msg);
        void cloud2_callback(const sensor_msgs::PointCloud2 msg);
        void cloud3_callback(const sensor_msgs::PointCloud2 msg);
	void concatenate_clouds();
	bool initialized() { return c1_initialized && c2_initialized && c3_initialized; }

    public:

        // functions
        Cloud();
        Cloud(ros::NodeHandle handle);
	void publish_master_cloud();	
};

#endif // CLOUD_CLASS_HPP

////////////////////////////////////////////////////////////////
