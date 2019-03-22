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

#include <boost/filesystem.hpp>
#include <cstring>
#include <ctime> 
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
#include <tf/transform_listener.h>

using namespace pcl;
using namespace std;
using namespace tf;

class Cloud 
{
    private: 

        // members
        PolygonMesh master_mesh;
	PointCloud<PointNormal> master_cloud;
	PointCloud<PointXYZRGB> cloud_front;
	PointCloud<PointXYZRGB> cloud_back;
	PointCloud<PointXYZRGB> cloud_left;
	PointCloud<PointXYZRGB> cloud_right;
	PointCloud<PointXYZRGB> cloud_top;
	PointCloud<PointXYZRGB> colored_master_cloud;
        bool front_initialized;
        bool back_initialized;
        bool left_initialized;
        bool right_initialized;
        bool top_initialized;
        ros::Publisher  cloud_pub;
        ros::Subscriber cloud_front_sub;
        ros::Subscriber cloud_back_sub;
        ros::Subscriber cloud_left_sub;
        ros::Subscriber cloud_right_sub;
        ros::Subscriber cloud_top_sub;
        TransformListener tf_listener;
        int counter;
        string timestamp; 

        // functions
    	void cloud_front_callback(sensor_msgs::PointCloud2 msg);
        void cloud_back_callback(const sensor_msgs::PointCloud2 msg);
        void cloud_left_callback(const sensor_msgs::PointCloud2 msg);
        void cloud_right_callback(const sensor_msgs::PointCloud2 msg);
        void cloud_top_callback(const sensor_msgs::PointCloud2 msg);
       	void concatenate_clouds();
        void triangulate_clouds();
        void move_least_squares();
        void voxel_filter();
        void output_file(string model_name);
	bool initialized();
        string get_timestamp();

    public:

        // functions
        Cloud();
        Cloud(ros::NodeHandle handle);
        void produce_model(string model_name = "model");
	void publish_master_cloud();	
};

#endif // CLOUD_CLASS_HPP

////////////////////////////////////////////////////////////////
