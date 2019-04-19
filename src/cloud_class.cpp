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
#include "MLS.h"

////////////////////////////////////////////////////////////////

// Public Members

// DEFAULT CONSTRUCTOR
// does not do ros initialization
Cloud::Cloud()
{
    this->cloud1_initialized = false;
    this->cloud2_initialized = false;
    this->cloud3_initialized = false;
    this->cloud4_initialized = false;
    this->cloud5_initialized = false;
    this->cloud6_initialized = false;
    this->cloud7_initialized = false;
    this->cloud8_initialized = false;
    this->cloud9_initialized = false;

    this->counter = 0;
    this->timestamp = get_timestamp();
    this->start_time = clock();
}

// CONSTRUCTOR
// does the ros initialization
Cloud::Cloud(ros::NodeHandle handle)
{
    this->cloud1_initialized = false;
    this->cloud2_initialized = false;
    this->cloud3_initialized = false;
    this->cloud4_initialized = false;
    this->cloud5_initialized = false;
    this->cloud6_initialized = false;
    this->cloud7_initialized = false;
    this->cloud8_initialized = false;
    this->cloud9_initialized = false;

    this->obj_file_pub = handle.advertise<std_msgs::String>("object_model", 1);
    this->cloud_pub = handle.advertise<sensor_msgs::PointCloud2>("combined_cloud", 1);

    this->filtered_cloud1_sub = handle.subscribe("camera_1/depth_registered/points_filtered", 1, &Cloud::filtered_cloud1_callback, this);
    this->filtered_cloud2_sub = handle.subscribe("camera_2/depth_registered/points_filtered", 1, &Cloud::filtered_cloud2_callback, this);
    this->filtered_cloud3_sub = handle.subscribe("camera_3/depth_registered/points_filtered", 1, &Cloud::filtered_cloud3_callback, this);
    this->filtered_cloud4_sub = handle.subscribe("camera_4/depth_registered/points_filtered", 1, &Cloud::filtered_cloud4_callback, this);
    this->filtered_cloud5_sub = handle.subscribe("camera_5/depth_registered/points_filtered", 1, &Cloud::filtered_cloud5_callback, this);
    this->filtered_cloud6_sub = handle.subscribe("camera_6/depth_registered/points_filtered", 1, &Cloud::filtered_cloud6_callback, this);
    this->filtered_cloud7_sub = handle.subscribe("camera_7/depth_registered/points_filtered", 1, &Cloud::filtered_cloud7_callback, this);
    this->filtered_cloud8_sub = handle.subscribe("camera_8/depth_registered/points_filtered", 1, &Cloud::filtered_cloud8_callback, this);
    this->filtered_cloud9_sub = handle.subscribe("camera_9/depth_registered/points_filtered", 1, &Cloud::filtered_cloud9_callback, this);

    this->counter = 0;
    this->timestamp = get_timestamp();
    this->start_time = clock();

    clock_t before;
    clock_t after;

    cout << endl << endl;
    before = clock();
    this->log_event(before, after, "Initializing cameras", BEFORE); 
    while (!this->initialized()) ros::spinOnce();
    after = clock();
    this->log_event(before, after, "Initializing cameras", AFTER); 
}

// PRODUCE MODEL FUNCTION
// this function concatenates the pointclouds, does preprocessing on 
// the clouds (including smoothing, up/downsampling, etc), creates a polygon 
// mesh from the resulting cloud, and saves the output to an obj file
void Cloud::produce_model(string model_name) 
{
    clock_t before;
    clock_t after;

    before = clock();
    this->concatenate_clouds();
    after = clock();
    this->log_event(before, after, "Concatenate function", AFTER); 
 
    before = clock();
    this->voxel_filter(0.0022);
    after = clock();
    this->log_event(before, after, "Voxel function", AFTER); 

    before = clock();
    this->move_least_squares<Device>();
    after = clock();
    this->log_event(before, after, "Move least squares function", AFTER); 

    before = clock();
    this->voxel_filter(0.0075);
    after = clock();
    this->log_event(before, after, "Voxel function", AFTER); 

    before = clock();
    this->triangulate_clouds();
    after = clock();
    this->log_event(before, after, "Triangulate function", AFTER); 

    before = clock();
    this->output_file(model_name); 
    after = clock();
    this->log_event(before, after, "Creation of output file", AFTER); 

    this->log_event(this->start_time, after, "Full process", AFTER); 
    cout << endl << "Publishing cloud forever..." << endl;
    cout << "Press Ctrl+C to quit" << endl << endl;

    this->counter++;
}


// PUBLISH MASTER CLOUD FUNCTION
// this function converts the pcl XYZRGB cloud to a 
// sensor_msg PointCloud2 for easy publishing
void Cloud::publish_master_cloud() 
{
    sensor_msgs::PointCloud2 proc_cloud;
    toROSMsg(this->master_cloud, proc_cloud);
    this->cloud_pub.publish(proc_cloud);
}

////////////////////////////////////////////////////////////////

// Private Members & Callbacks

// INITIALIZED FUNCTION
// this function checks that all the pointclouds are initialized
bool Cloud::initialized() 
{ 
    return (this->cloud1_initialized 
            && this->cloud2_initialized 
            && this->cloud3_initialized 
            && this->cloud4_initialized 
            && this->cloud5_initialized 
            && this->cloud6_initialized 
            && this->cloud7_initialized 
            && this->cloud8_initialized 
            );
}

// FILTERED CLOUD1 CALLBACK FUNCTION
// initialize cloud1 with the filtered pointcloud information
void Cloud::filtered_cloud1_callback(sensor_msgs::PointCloud2 msg) 
{
    fromROSMsg(msg, this->cloud1);
    if (this->cloud1_initialized == false)
        this->log_event(clock(), clock(), "Cloud 1", BEFORE); 
    this->cloud1_initialized = true;
}

// FILTERED CLOUD2 CALLBACK FUNCTION
// initialize cloud2 with the filtered pointcloud information
void Cloud::filtered_cloud2_callback(sensor_msgs::PointCloud2 msg) 
{
    fromROSMsg(msg, this->cloud2);
    if (this->cloud2_initialized == false)
        this->log_event(clock(), clock(), "Cloud 2", BEFORE); 
    this->cloud2_initialized = true;
}

// FILTERED CLOUD3 CALLBACK FUNCTION
// initialize cloud3 with the filtered pointcloud information
void Cloud::filtered_cloud3_callback(sensor_msgs::PointCloud2 msg) 
{
    fromROSMsg(msg, this->cloud3);
    if (this->cloud3_initialized == false)
        this->log_event(clock(), clock(), "Cloud 3", BEFORE); 
    this->cloud3_initialized = true;
}

// FILTERED CLOUD4 CALLBACK FUNCTION
// initialize cloud1 with the filtered pointcloud information
void Cloud::filtered_cloud4_callback(sensor_msgs::PointCloud2 msg) 
{
    fromROSMsg(msg, this->cloud4);
    if (this->cloud4_initialized == false)
        this->log_event(clock(), clock(), "Cloud 4", BEFORE); 
    this->cloud4_initialized = true;
}

// FILTERED CLOUD5 CALLBACK FUNCTION
// initialize cloud5 with the filtered pointcloud information
void Cloud::filtered_cloud5_callback(sensor_msgs::PointCloud2 msg) 
{
    fromROSMsg(msg, this->cloud5);
    if (this->cloud5_initialized == false)
        this->log_event(clock(), clock(), "Cloud 5", BEFORE); 
    this->cloud5_initialized = true;
}

// FILTERED CLOUD6 CALLBACK FUNCTION
// initialize cloud6 with the filtered pointcloud information
void Cloud::filtered_cloud6_callback(sensor_msgs::PointCloud2 msg) 
{
    fromROSMsg(msg, this->cloud6);
    if (this->cloud6_initialized == false)
        this->log_event(clock(), clock(), "Cloud 6", BEFORE); 
    this->cloud6_initialized = true;
}

// FILTERED CLOUD7 CALLBACK FUNCTION
// initialize cloud7 with the filtered pointcloud information
void Cloud::filtered_cloud7_callback(sensor_msgs::PointCloud2 msg) 
{
    fromROSMsg(msg, this->cloud7);
    if (this->cloud7_initialized == false)
        this->log_event(clock(), clock(), "Cloud 7", BEFORE); 
    this->cloud7_initialized = true;
}

// FILTERED CLOUD8 CALLBACK FUNCTION
// initialize cloud8 with the filtered pointcloud information
void Cloud::filtered_cloud8_callback(sensor_msgs::PointCloud2 msg) 
{
    fromROSMsg(msg, this->cloud8);
    if (this->cloud8_initialized == false)
        this->log_event(clock(), clock(), "Cloud 8", BEFORE); 
    this->cloud8_initialized = true;
}

// FILTERED CLOUD9 CALLBACK FUNCTION
// initialize cloud9 with the filtered pointcloud information
void Cloud::filtered_cloud9_callback(sensor_msgs::PointCloud2 msg) 
{
    fromROSMsg(msg, this->cloud9);
    if (this->cloud9_initialized == false)
        this->log_event(clock(), clock(), "Cloud 9", BEFORE); 
    this->cloud9_initialized = true;
}


// CONCATENATE CLOUDS FUNCTION
// concatenates the points of all the 
// cloud members into the master pointcloud
void Cloud::concatenate_clouds() 
{
    this->master_cloud = this->cloud1;
    this->master_cloud+= this->cloud2;
    this->master_cloud+= this->cloud3;
    this->master_cloud+= this->cloud4;
    this->master_cloud+= this->cloud5;
    this->master_cloud+= this->cloud6;
    this->master_cloud+= this->cloud7;
    this->master_cloud+= this->cloud8;
}

// MOVE LEAST SQUARES FUNCTION
// aligns the surface normals to eliminate noise
// also does upsampling ro reduce noise & fill holes
template <template <typename> class Storage>
void Cloud::move_least_squares()
{
    cout << "\t# Points in Cloud Before : " << this->master_cloud.points.size() << " --------------------------------" << endl;
    std_msgs::Header old_head;
    pcl_conversions::fromPCL(this->master_cloud.header, old_head);

    PointCloud<pcl::PointXYZRGB>::Ptr output (new PointCloud<pcl::PointXYZRGB>);
    PointCloudAOS<Host> data_host;
    PointCloudAOS<Device>::Ptr data;
    PointCloudAOS<Device>::Ptr data_out;
   
    // convert pcl cloud to cuda cloud 
    data_host.points.resize(this->master_cloud.points.size());
    for (size_t i = 0; i < this->master_cloud.points.size (); ++i)
    {
        pcl::cuda::PointXYZRGB pt;
        pt.x = this->master_cloud.points[i].x;
        pt.y = this->master_cloud.points[i].y;
        pt.z = this->master_cloud.points[i].z;
        pt.rgb = *(float*)(&this->master_cloud.points[i].rgb); // Pack RGB into a float
        data_host.points[i] = pt;
    }
    data_host.width = this->master_cloud.width;
    data_host.height = this->master_cloud.height;
    data_host.is_dense = this->master_cloud.is_dense;

    // make sure there is enough memory in the gpu for this 
    size_t free, total;
    cudaMemGetInfo(&free, &total);
    if (free/MEGA < 15)
    {
    	cout << "Not enough memory!   ";
    	cout << "Free: " << free/MEGA << " of " << total/MEGA << " mb" << endl;
    	return;
    }

    // thrust the cloud through the gpu mls stuff
    data = toStorage<Host, Storage> (data_host); 
    data_out = toStorage<Host, Storage> (data_host);
    thrustPCL_AOS(data, data_out, NN_CONNECTIVITY, SMOOTHNESS);

    // save the cloud to a pcl cloud 
    pcl::cuda::toPCL(*data_out, *output);
    this->master_cloud = *output;
    pcl_conversions::toPCL(old_head, this->master_cloud.header);
    //cout << "\t# Points in Cloud After : " << this->master_cloud.points.size() << " --------------------------------" << endl;
}


// VOXEL FILTER FUNCTION
// downsamples point cloud to make the resulting model cleaner
void Cloud::voxel_filter(float leaf_size)
{
    PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new PointCloud<pcl::PointXYZRGB>);
    PointCloud<pcl::PointXYZRGB>::Ptr cloud(new PointCloud<pcl::PointXYZRGB>);
    *cloud = this->master_cloud;

    VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size); 
    sor.filter(this->master_cloud);
}

// TRIANGULATE CLOUD FUNCTION
// creates a mesh from all the clouds
void Cloud::triangulate_clouds() 
{
    PointCloud<pcl::PointXYZ>::Ptr cloud(new PointCloud<pcl::PointXYZ>);
    copyPointCloud(this->master_cloud, *cloud);

    NormalEstimation<pcl::PointXYZ, pcl::Normal> n;                                     // create a normal estimator utility
    PointCloud<pcl::Normal>::Ptr normals(new PointCloud<pcl::Normal>);                  // create a recepticle for the normals
    search::KdTree<pcl::PointXYZ>::Ptr normal_tree(new search::KdTree<pcl::PointXYZ>);  // create a receptical for the search tree
    normal_tree->setInputCloud(cloud);                                                  // initialize the tree's cloud
    n.setInputCloud(cloud);                                                             // initialize the normal calculator's cloud
    n.setSearchMethod (normal_tree);                                                    // initialize the normal caclulator's tree
    n.setKSearch (20);                                                                  // TODO: find out what this parameter does
    n.compute (*normals);                                                               // compute the normals of the cloud with the calculator

    PointCloud<pcl::PointNormal>::Ptr normal_cloud(new PointCloud<pcl::PointNormal>);   // create a recepticle for the comnbined xyz and normal information
    concatenateFields(*cloud, *normals, *normal_cloud);                     // combine the normal and xyz information

    search::KdTree<pcl::PointNormal>::Ptr mesh_tree(new search::KdTree<pcl::PointNormal>);   
    mesh_tree->setInputCloud(normal_cloud);                                  

    GreedyProjectionTriangulation<pcl::PointNormal> gp3;        // make a greedy projection triangulation object
    gp3.setSearchRadius(0.05);                                  // set the maximum edge length between connected points (m)
    gp3.setMu(2.5);                                             // maximum distance for a point to be considered relative to the distance to the nearest point
    gp3.setMaximumNearestNeighbors(500);                        // defines how many neighbors are searched for
    gp3.setMinimumAngle(M_PI/18);                               //  10 degrees : minimum angle in each triangle
    gp3.setMaximumAngle(5*M_PI/6);                              // 150 degrees : maximim angle in each triangle
    gp3.setMaximumSurfaceAngle(M_PI/4);                         //  90 degrees : helps keep jarring transitions smooth 
    gp3.setNormalConsistency(true);                             // also helps keep jarring transitions smooth

    gp3.setInputCloud(normal_cloud);    // initialize the input cloud of the gp3
    gp3.setSearchMethod(mesh_tree);     // initialize the input tree of the gp3
    gp3.reconstruct(this->master_mesh); // triangulize that shit
    
    /*
    cout << "begin poisson reconstruction" << endl;
    Poisson<PointNormal> poisson;
    poisson.setDepth(10);
    poisson.setInputCloud(normal_cloud);
    poisson.reconstruct(this->master_mesh);
    */
}

// OUTPUT FILE FUNCTION
// this function converts the polymesh to a parsable file format
void Cloud::output_file(string model_name) 
{
    stringstream path;
    path << ros::package::getPath("synchronize_pointclouds");
    path << "/object_models/";
    path << model_name << "_" << this->timestamp << ".obj";
    
    // make a directory for multiple model files
    // boost::filesystem::create_directory(path.str() + "/model_" + this->timestamp);
    // path << "/model_" << this->timestamp;
   
    io::saveOBJFile(path.str(), this->master_mesh);
 
    ifstream t(path.str().c_str());             
    stringstream file_contents;         
    file_contents << t.rdbuf();         

    std_msgs::String msg;               
    msg.data = file_contents.str();
    obj_file_pub.publish(msg);
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

// DURATION MS FUNCTION
// gets the duration of the passed clock times in milleseconds
double Cloud::durationMS(clock_t before, clock_t after) 
{
    double duration;
    
    duration = (double)(after - before);
    duration/= (double)CLOCKS_PER_SEC;
    duration = round(duration * 1000.0) / 1000.0;

    return duration;
}

// LOG EVENT FUNCTION
// log an event & some time information
void Cloud::log_event(clock_t before, clock_t after, string description, bool when) 
{
    cout.setf(ios::fixed, ios::floatfield);
    cout.precision(3); 

    if (when == BEFORE) 
    {
        cout << "[" << setw(6) << durationMS(this->start_time, before) << "] ";
        cout << description << " started..." << endl; 
    }

    if (when == AFTER) 
    {
        cout << "[" << setw(6) << durationMS(this->start_time, after) << "] ";
        cout << description << " completed in ";
        cout << durationMS(before, after) << " seconds..." << endl;
    }
}

////////////////////////////////////////////////////////////////

