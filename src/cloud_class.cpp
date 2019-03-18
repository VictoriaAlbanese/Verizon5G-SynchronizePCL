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
    this->cloud_top_sub = handle.subscribe("camera_top/depth_registered/points_filtered", 1, &Cloud::cloud_top_callback, this);

    while (!this->initialized()) ros::spinOnce();
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

// CONCATENATE CLOUDS FUNCTION
// concatenates the points of all the 
// cloud members into the master pointcloud
void Cloud::concatenate_clouds() 
{
    PointCloud<PointXYZ>::Ptr temp_cloud(new PointCloud<PointXYZ>);
    
    *temp_cloud = this->cloud_front;
    *temp_cloud+= this->cloud_back;
    *temp_cloud+= this->cloud_left;
    *temp_cloud+= this->cloud_right;
    *temp_cloud+= this->cloud_top;

    *temp_cloud = this->voxel_filter(temp_cloud);
    this->master_cloud = this->move_least_squares(temp_cloud); 
}

// TRIANGULATE CLOUD FUNCTION
// creates a mesh from all the clouds
void Cloud::triangulate_clouds() 
{
    // concatenate the XYZ and normal fields
    PointCloud<PointNormal>::Ptr normal_cloud(new PointCloud<PointNormal>);
    *normal_cloud = this->master_cloud;

    // create search tree
    search::KdTree<PointNormal>::Ptr mesh_tree(new search::KdTree<PointNormal>);   
    mesh_tree->setInputCloud(normal_cloud);                                  

    // set typical values for triangulation parameters
    GreedyProjectionTriangulation<pcl::PointNormal> gp3;        // make a greedy projection triangulation object
    gp3.setSearchRadius(0.025);                                 // set the maximum edge length between connected points (mm?)
    gp3.setMu(2.5);                                             // maximum distance for a point to be considered relative to the distance to the nearest point
    gp3.setMaximumNearestNeighbors(200);                        // defines how many neighbors are searched for
    gp3.setMinimumAngle(M_PI/18);                               //  10 degrees : minimum angle in each triangle
    gp3.setMaximumAngle(5*M_PI/6);                              // 150 degrees : maximim angle in each triangle
    gp3.setMaximumSurfaceAngle(M_PI/4);                         //  45 degrees : helps keep jarring transitions smooth 
    gp3.setNormalConsistency(true);                             // also helps keep jarring transitions smooth

    // produce mesh
    gp3.setInputCloud(normal_cloud);    // initialize the input cloud of the gp3
    gp3.setSearchMethod(mesh_tree);     // initialize the input tree of the gp3
    gp3.reconstruct(this->master_mesh); // triangulize that shit
}

// OUTPUT FILE FUNCTION
// this function converts the polymesh to a parsable file format
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
}

////////////////////////////////////////////////////////////////

// Private Members & Callbacks

// INITIALIZED FUNCTION
// this function checks that all the pointclouds are initialized
bool Cloud::initialized() 
{ 
    return (this->front_initialized 
            && this->back_initialized 
            && this->left_initialized 
            && this->right_initialized); 
}

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

// CLOUD TOP CALLBACK FUNCTION
// initialize cloud top with the information from camera_top
// converts the cloud message to a pcl XYZRGB pointcloud 
void Cloud::cloud_top_callback(const sensor_msgs::PointCloud2 msg) 
{
    fromROSMsg(msg, this->cloud_top);
 
    ros::Time stamp;
    pcl_conversions::toPCL(stamp, this->cloud_top.header.stamp);
    this->tf_listener.waitForTransform("/world", this->cloud_top.header.frame_id, stamp, ros::Duration(10.0));
    pcl_ros::transformPointCloud("/world", this->cloud_top, this->cloud_top, this->tf_listener);

    this->top_initialized = true;
}

// VOXEL FILTER FUNCTION
// downsamples point cloud to make the resulting model cleaner
PointCloud<PointXYZ> Cloud::voxel_filter(PointCloud<PointXYZ>::Ptr cloud)
{
    PointCloud<PointXYZ>::Ptr filtered_cloud(new PointCloud<PointXYZ>);

    VoxelGrid<PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.0075, 0.0075, 0.0075); 
    sor.filter(*filtered_cloud);

    return *filtered_cloud;
}

// MOVE LEAST SQUARES FUNCTION
// aligns the surface normals to eliminate noise
PointCloud<PointNormal> Cloud::move_least_squares(PointCloud<PointXYZ>::Ptr cloud)
{
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
    PointCloud<PointNormal> mls_points;
    MovingLeastSquares<PointXYZ, PointNormal> mls;
 
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(4);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.05);
    mls.process(mls_points);

    return mls_points;
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

