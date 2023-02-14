#include <iostream>
#include <string>

#include <ros/ros.h>
#include "rostopic_node/rostopic_communicator.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "rostopic_communicator_node");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("rostopic_communicator_node - starts.");

	try{
        ROSTopicCommunicator sensor_publisher(nh);
	}
	catch (std::exception& e){
        ROS_ERROR(e.what());
	}

    ROS_INFO_STREAM("rostopic_communicator_node - terminated.");
	return 0;
}