#include <iostream>
#include <string>

#include <ros/ros.h>

#include "control_node/geometry_controller.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "geometry_controller_node");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("geometry_controller_node - starts.");

	try {
        GeometryControllerROS gc(nh);
	}
	catch (std::exception& e) {
        ROS_ERROR(e.what());
	}

    ROS_INFO_STREAM("geometry_controller_node - terminated.");
	return 0;
}