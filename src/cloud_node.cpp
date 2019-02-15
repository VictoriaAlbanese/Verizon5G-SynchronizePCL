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

	// Loop
	while (ros::ok())
	{
		// concatenate the clouds
		cloud.concatenate_clouds();
		ROS_INFO("Master cloud is of size %zu", cloud.master_cloud.data.size());

		// spin & sleep
	    ros::spinOnce();    
	}

  	return 0;
}

//////////////////////////////////////////////////////////////

