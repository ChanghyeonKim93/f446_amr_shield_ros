#ifndef _KEYBOARD_CONTROLLER_POSITION_YAW_H_
#define _KEYBOARD_CONTROLLER_POSITION_YAW_H_

#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

// keyboard input tool
#include "keyinput.h"

class KeyboardControllerPositionYawROS 
{
public:
    KeyboardControllerPositionYawROS(const ros::NodeHandle& nh);

    void run();

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_position_yaw_;


private:
    std::string user_manual_;
};


#endif