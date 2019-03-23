//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: receiver_node.cpp
//
// Purpose: This function recieves the object model obj file
// as a string and writes it back into an obj file
//
//////////////////////////////////////////////////////////////

#include <iostream>
#include <fstream>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std;

void obj_file_callback(const std_msgs::String::ConstPtr& msg)
{
    char buffer[80];
    time_t t; 
    struct tm* now; 

    time(&t); 
    now = localtime(&t); 
    strftime(buffer, 80, "%Y-%m-%H%M%S", now);
    string timestamp(buffer);

    stringstream path;
    path << ros::package::getPath("synchronize_pointclouds");
    path << "/object_models/";
    path << "object_model_" << timestamp << ".obj";

    ofstream obj_file(path.str().c_str());
    obj_file << msg->data.c_str();
    obj_file.close(); 
    
    ROS_INFO("WROTE FILE----------------------------------------------------------");
}

int main(int argc, char * argv[]) 
{
    ros::init(argc, argv, "receiver_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("object_model", 10, obj_file_callback);
    ros::spin();

    return 0;
}

//////////////////////////////////////////////////////////////

