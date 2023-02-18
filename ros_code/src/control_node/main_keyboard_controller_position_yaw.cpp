#include <iostream>
#include <string>

#include <ros/ros.h>

#include "control_node/keyboard_controller_position_yaw.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_controller_position_yaw_node");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("keyboard_controller_position_yaw_node - starts.");

	try{
        KeyboardControllerPositionYawROS kc(nh);
	}
	catch (std::exception& e){
        ROS_ERROR(e.what());
	}

    ROS_INFO_STREAM("keyboard_controller_position_yaw_node - terminated.");
	return 0;
}