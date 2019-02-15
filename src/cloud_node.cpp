//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: cloud_node.cpp
//
// Purpose: Convert a depth cloud to a pointcloud
//
//////////////////////////////////////////////////////////////

#include "cloud_class.hpp"

int main(int argc, char ** argv) 
{
	// Initialize ros
	ros::init(argc, argv, "cloud_node");
	ros::NodeHandle nh;

    // Make the cloud & spin
    Cloud cloud(nh);
    ros::spin();    

  	return 0;
}

//////////////////////////////////////////////////////////////

