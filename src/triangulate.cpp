//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: greedy_projection.cpp
//
// Purpose: Tutorial code taken from the below source
// http://pointclouds.org/documentation/tutorials/greedy_projection.php
//
//////////////////////////////////////////////////////////////

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

int main (int argc, char** argv)
{
    ////////////////////////////////////////////////////////////

    // This needs to get adapted & reduced since I'm not reading in my pointcloud
    // NOTE: I should change the initial type of my pointcloud to make this all work
    // assuming normal information is automatically calculated which why shouldn't it be?

    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);     // make a recepticle for the xyz cloud
    pcl::PCLPointCloud2 cloud_blob;                                                     // make a recepticle for the raw input cloud
    pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);                                      // input the raw input cloud from a file
    pcl::fromPCLPointCloud2 (cloud_blob, *cloud);                                       // convert the raw input cloud to the xyz cloud
       
    // Normal estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;                                        // create a normal estimator utility
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);               // create a recepticle for the normals
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);      // create a receptical for the search tree
    tree->setInputCloud (cloud);                                                                // initialize the tree's cloud
    n.setInputCloud (cloud);                                                                    // initialize the normal calculator's cloud
    n.setSearchMethod (tree);                                                                   // initialize the normal caclulator's tree
    n.setKSearch (20);                                                                          // TODO: find out what this parameter does
    n.compute (*normals);                                                                       // compute the normals of the cloud with the calculator

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);  // create a recepticle for the comnbined xyz and normal information
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);                                     // combine the normal and xyz information

    ////////////////////////////////////////////////////////////

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);       // create a search tree
    tree2->setInputCloud (cloud_with_normals);                                                          // initialize the search tree with the cloud with normals

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   // make a greedy projection triangulation object
    pcl::PolygonMesh triangles;                                 // make a polygon mesh object

    // Set typical values for the parameters
    gp3.setSearchRadius (0.025);                // set the maximum edge length between connected points (mm?)
    gp3.setMu (2.5);                            // maximum distance for a point to be considered relative to the distance to the nearest point
    gp3.setMaximumNearestNeighbors (100);       // defines how many neighbors are searched for
    gp3.setMinimumAngle(M_PI/18);               //  10 degrees : minimum angle in each triangle
    gp3.setMaximumAngle(2*M_PI/3);              // 120 degrees : maximim angle in each triangle
    gp3.setMaximumSurfaceAngle(M_PI/4);         //  45 degrees : helps keep jarring transitions smooth 
    gp3.setNormalConsistency(false);            // also helps keep jarring transitions smooth

    // Get result
    gp3.setInputCloud (cloud_with_normals);     // initialize the input cloud of the gp3
    gp3.setSearchMethod (tree2);                // initialize the input tree of the gp3
    gp3.reconstruct (triangles);                // triangulize that shit


    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    // Save to a VTK file
    pcl::io::saveVTKFile ("mesh.vtk", triangles); 

    ////////////////////////////////////////////////////////////

    return (0);
}

//////////////////////////////////////////////////////////////

