//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: cloud_node.cpp
//
// Purpose: This is the main driver; a cloud is created 
// and a mesh is outputted via the inner workings of that 
// cloud object
//
//////////////////////////////////////////////////////////////

#include <iostream>
#include "cloud_class.hpp"

using namespace std;

int main(int argc, char * argv[]) 
{
    int i;
    for (i = 0; i < argc; i++) cout << argc << " " << argv[argc] << endl;

    ros::init(argc, argv, "cloud_node");
    ros::NodeHandle nh;

    Cloud cloud(nh);

    while (ros::ok())
    {
        cloud.publish_master_cloud();
    
        if (argc == 2) cloud.output_file(argv[1]);
        else cloud.output_file();
    
        cout << ".";
        ros::spinOnce();    
    }

    return 0;
}

//////////////////////////////////////////////////////////////

