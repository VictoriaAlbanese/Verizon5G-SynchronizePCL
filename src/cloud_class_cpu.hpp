////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: cloud_class.hpp
//
// Purpose: Declares a class which handles operations on the 
// pointclouds created by the Intel RealSense D435i sensors
//
////////////////////////////////////////////////////////////////

#ifndef CLOUD_CLASS_CPU_HPP
#define CLOUD_CLASS_CPU_HPP

#include <boost/filesystem.hpp>
#include <cstring>
#include <ctime> 
#include <fstream>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/obj_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>


#define BEFORE 0
#define AFTER 1

using namespace pcl;
using namespace std;
using namespace tf;

class Cloud 
{
    private: 

        // members-------------------------------------------

        PolygonMesh master_mesh;
	PointCloud<PointXYZRGB> master_cloud;
	PointCloud<PointXYZRGB> cloud1;
	PointCloud<PointXYZRGB> cloud2;
	PointCloud<PointXYZRGB> cloud3;
	PointCloud<PointXYZRGB> cloud4;
	PointCloud<PointXYZRGB> cloud5;
	PointCloud<PointXYZRGB> cloud6;
	PointCloud<PointXYZRGB> cloud7;
	PointCloud<PointXYZRGB> cloud8;
	PointCloud<PointXYZRGB> cloud9;
        bool cloud1_initialized;
        bool cloud2_initialized;
        bool cloud3_initialized;
        bool cloud4_initialized;
        bool cloud5_initialized;
        bool cloud6_initialized;
        bool cloud7_initialized;
        bool cloud8_initialized;
        bool cloud9_initialized;
        
        ros::Publisher  cloud_pub;
        ros::Publisher  obj_file_pub;
       
        ros::Subscriber filtered_cloud1_sub;
        ros::Subscriber filtered_cloud2_sub;
        ros::Subscriber filtered_cloud3_sub;
        ros::Subscriber filtered_cloud4_sub;
        ros::Subscriber filtered_cloud5_sub;
        ros::Subscriber filtered_cloud6_sub;
        ros::Subscriber filtered_cloud7_sub;
        ros::Subscriber filtered_cloud8_sub;
        ros::Subscriber filtered_cloud9_sub;
        
        int counter;
        string timestamp; 
        clock_t start_time;

        // functions-----------------------------------------
       	
        void concatenate_clouds();
        void triangulate_clouds();
        void move_least_squares();       
        void voxel_filter();
        void output_file(string model_name);

    	void filtered_cloud1_callback(sensor_msgs::PointCloud2 msg);
    	void filtered_cloud2_callback(sensor_msgs::PointCloud2 msg);
    	void filtered_cloud3_callback(sensor_msgs::PointCloud2 msg);
    	void filtered_cloud4_callback(sensor_msgs::PointCloud2 msg);
    	void filtered_cloud5_callback(sensor_msgs::PointCloud2 msg);
    	void filtered_cloud6_callback(sensor_msgs::PointCloud2 msg);
    	void filtered_cloud7_callback(sensor_msgs::PointCloud2 msg);
    	void filtered_cloud8_callback(sensor_msgs::PointCloud2 msg);
    	void filtered_cloud9_callback(sensor_msgs::PointCloud2 msg);

	bool initialized();
        string get_timestamp();
        double durationMS(clock_t before, clock_t after);
        void log_event(clock_t before, clock_t after, string description, bool when); 

    public:

        // functions-----------------------------------------
        
        Cloud();
        Cloud(ros::NodeHandle handle);
        void produce_model(string model_name = "model");
	void publish_master_cloud();	
        int count() { return counter; }
};

#endif // CLOUD_CLASS_CPU_HPP

////////////////////////////////////////////////////////////////
