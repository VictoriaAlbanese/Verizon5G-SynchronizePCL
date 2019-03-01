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
    this->front_initialized = false;
    this->back_initialized = false;
    this->left_initialized = false;
    this->right_initialized = false;

    this->counter = 1;
    this->timestamp = get_timestamp();
}

// CONSTRUCTOR
// does the ros initialization
Cloud::Cloud(ros::NodeHandle handle)
{
    this->front_initialized = false;
    this->back_initialized = false;
    this->left_initialized = false;
    this->right_initialized = false;

    this->counter = 1;
    this->timestamp = get_timestamp();

    this->cloud_pub = handle.advertise<sensor_msgs::PointCloud2>("combined_cloud", 1);
    this->cloud_front_sub = handle.subscribe("camera_front/depth_registered/points_filtered", 1, &Cloud::cloud_front_callback, this);
    this->cloud_back_sub = handle.subscribe("camera_back/depth_registered/points_filtered", 1, &Cloud::cloud_back_callback, this);
    this->cloud_left_sub = handle.subscribe("camera_left/depth_registered/points_filtered", 1, &Cloud::cloud_left_callback, this);
    this->cloud_right_sub = handle.subscribe("camera_right/depth_registered/points_filtered", 1, &Cloud::cloud_right_callback, this);

    while (!this->initialized()) ros::spinOnce();
    
    this->concatenate_clouds();
    this->triangulate_clouds();
}

// PUBLISH MASTER CLOUD FUNCTION
// this function converts the pcl XYZRGB cloud to a 
// sensor_msg PointCloud2 for easy publishing
void Cloud::publish_master_cloud() 
{
    sensor_msgs::PointCloud2 cloud;
    toROSMsg(this->master_cloud, cloud);

    this->cloud_pub.publish(cloud);
}

// OUTPUT FILE FUNCTION
// this function converts the polymesh to a parsable file format
// current options for output file formats include
//      - 0 : VTK
void Cloud::output_file(string model_name) 
{
    std::stringstream path;
    path << ros::package::getPath("synchronize_pointclouds");
    path << "/object_models/";
    path << model_name << "_" << this->timestamp << ".obj";
    
    // make a directory for multiple model files
    // boost::filesystem::create_directory(path.str() + "/model_" + this->timestamp);
    // path << "/model_" << this->timestamp;
   
    io::saveOBJFile(path.str(), this->master_mesh); 
    cout << ".";
}

////////////////////////////////////////////////////////////////

// Private Members & Callbacks

// CLOUD FRONT CALLBACK FUNCTION
// initialize cloud front with the information from camera_front
// converts the cloud message to a pcl XYZRGB pointcloud 
void Cloud::cloud_front_callback(sensor_msgs::PointCloud2 msg) 
{
    fromROSMsg(msg, this->cloud_front);

    ros::Time stamp;
    pcl_conversions::toPCL(stamp, this->cloud_front.header.stamp);
    this->tf_listener.waitForTransform("/world", this->cloud_front.header.frame_id, stamp, ros::Duration(10.0));
    pcl_ros::transformPointCloud("/world", this->cloud_front, this->cloud_front, this->tf_listener);

    this->front_initialized = true;
}

// CLOUD BACK CALLBACK FUNCTION
// initialize cloud back with the information from camera_back
// converts the cloud message to a pcl XYZRGB pointcloud 
void Cloud::cloud_back_callback(const sensor_msgs::PointCloud2 msg) 
{
    fromROSMsg(msg, this->cloud_back);
 
    ros::Time stamp;
    pcl_conversions::toPCL(stamp, this->cloud_back.header.stamp);
    this->tf_listener.waitForTransform("/world", this->cloud_back.header.frame_id, stamp, ros::Duration(10.0));
    pcl_ros::transformPointCloud("/world", this->cloud_back, this->cloud_back, this->tf_listener);
    
    this->back_initialized = true;
}

// CLOUD LEFT CALLBACK FUNCTION
// initialize cloud left with the information from camera_left
// converts the cloud message to a pcl XYZRGB pointcloud 
void Cloud::cloud_left_callback(const sensor_msgs::PointCloud2 msg) 
{
    fromROSMsg(msg, this->cloud_left);
 
    ros::Time stamp;
    pcl_conversions::toPCL(stamp, this->cloud_left.header.stamp);
    this->tf_listener.waitForTransform("/world", this->cloud_left.header.frame_id, stamp, ros::Duration(10.0));
    pcl_ros::transformPointCloud("/world", this->cloud_left, this->cloud_left, this->tf_listener);

    this->left_initialized = true;
}

// CLOUD RIGHT CALLBACK FUNCTION
// initialize cloud right with the information from camera_right
// converts the cloud message to a pcl XYZRGB pointcloud 
void Cloud::cloud_right_callback(const sensor_msgs::PointCloud2 msg) 
{
    fromROSMsg(msg, this->cloud_right);
 
    ros::Time stamp;
    pcl_conversions::toPCL(stamp, this->cloud_right.header.stamp);
    this->tf_listener.waitForTransform("/world", this->cloud_right.header.frame_id, stamp, ros::Duration(10.0));
    pcl_ros::transformPointCloud("/world", this->cloud_right, this->cloud_right, this->tf_listener);

    this->right_initialized = true;
}

// CONCATENATE CLOUDS FUNCTION
// concatenates the points of all the 
// cloud members into the master pointcloud
void Cloud::concatenate_clouds() 
{
    this->master_cloud = this->cloud_front;
    this->master_cloud+= this->cloud_back;
    this->master_cloud+= this->cloud_left;
    this->master_cloud+= this->cloud_right;
}

// TRIANGULATE CLOUD FUNCTION
// creates a mesh from all the clouds
void Cloud::triangulate_clouds() 
{
    // put the master cloud in a pointer
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    *cloud = this->master_cloud;
    
    // normal estimation
    NormalEstimation<PointXYZ, Normal> n;                                       // create a normal estimator utility
    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);                    // create a recepticle for the normals
    search::KdTree<PointXYZ>::Ptr normal_tree(new search::KdTree<PointXYZ>);    // create a receptical for the search tree
    normal_tree->setInputCloud(cloud);                                          // initialize the tree's cloud
    n.setInputCloud(cloud);                                                     // initialize the normal calculator's cloud
    n.setSearchMethod (normal_tree);                                            // initialize the normal caclulator's tree
    n.setKSearch (20);                                                          // TODO: find out what this parameter does
    n.compute (*normals);                                                       // compute the normals of the cloud with the calculator

    // concatenate the XYZ and normal fields
    PointCloud<PointNormal>::Ptr normal_cloud(new PointCloud<PointNormal>);     // create a recepticle for the comnbined xyz and normal information
    concatenateFields(this->master_cloud, *normals, *normal_cloud);             // combine the normal and xyz information

    // create search tree
    search::KdTree<PointNormal>::Ptr mesh_tree(new search::KdTree<PointNormal>);   
    mesh_tree->setInputCloud(normal_cloud);                                  

    // set typical values for triangulation parameters
    GreedyProjectionTriangulation<pcl::PointNormal> gp3;        // make a greedy projection triangulation object
    gp3.setSearchRadius (0.025);                                // set the maximum edge length between connected points (mm?)
    gp3.setMu (2.5);                                            // maximum distance for a point to be considered relative to the distance to the nearest point
    gp3.setMaximumNearestNeighbors (100);                       // defines how many neighbors are searched for
    gp3.setMinimumAngle(M_PI/18);                               //  10 degrees : minimum angle in each triangle
    gp3.setMaximumAngle(2*M_PI/3);                              // 120 degrees : maximim angle in each triangle
    gp3.setMaximumSurfaceAngle(M_PI/4);                         //  45 degrees : helps keep jarring transitions smooth 
    gp3.setNormalConsistency(false);                            // also helps keep jarring transitions smooth

    // produce mesh
    gp3.setInputCloud(normal_cloud);    // initialize the input cloud of the gp3
    gp3.setSearchMethod(mesh_tree);     // initialize the input tree of the gp3
    gp3.reconstruct(this->master_mesh); // triangulize that shit
}


// GET TIMESTAMP FUNCTION
// This function gets the current date & time and returns it as a string  
string Cloud::get_timestamp()
{
    char buffer[80];
    time_t t; 
    struct tm* now; 

    time(&t); 
    now = localtime(&t); 
    strftime(buffer, 80, "%Y-%m-%H%M%S", now);
    string timestamp(buffer);

    return timestamp;
}

////////////////////////////////////////////////////////////////

