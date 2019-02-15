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
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

using namespace pcl;
typedef pcl::PointXYZ PointXYZ;

class Cloud 
{
    private: 

        // members
        bool c1_initialized;
        bool c2_initialized;
        bool c3_initialized;
		sensor_msgs::PointCloud2 master_cloud;
        sensor_msgs::PointCloud2 cloud1;
        sensor_msgs::PointCloud2 cloud2;
        sensor_msgs::PointCloud2 cloud3;
        ros::Publisher cloud_pub;
        ros::Subscriber cloud1_sub;
        ros::Subscriber cloud2_sub;
        ros::Subscriber cloud3_sub;

        // functions
    	void cloud1_callback(const sensor_msgs::PointCloud2 msg);
        void cloud2_callback(const sensor_msgs::PointCloud2 msg);
        void cloud3_callback(const sensor_msgs::PointCloud2 msg);

    public:

        // functions
        Cloud();
        Cloud(ros::NodeHandle handle);
		bool initialized() { return c1_initialized && c2_initialized && c3_initialized; }
};

#endif // CLOUD_CLASS_HPP

////////////////////////////////////////////////////////////////
