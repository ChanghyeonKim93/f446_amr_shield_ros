#ifndef _GEOMETRY_CONTROLLER_H_
#define _GEOMETRY_CONTROLLER_H_

#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

#include "utility/geometry_library.h"

class GeometryControllerROS
{
public:
    GeometryControllerROS(const ros::NodeHandle& nh);

    void run();

private:
    void callbackOptitrack(const geometry_msgs::PoseStamped::ConstPtr& msg); // eight PWM signals
    void callbackDesiredPositionYaw(const std_msgs::Float32MultiArray::ConstPtr& msg); // data[0] = pos.x , data[1] = pos.y, data[2] = yaw angle (radian)

private:
    float B_;       // B_left + B_right (distance between two wheels)
    float B_left_;  // distance to the left wheel from the IMU 
    float B_right_; // distance to the right wheel from the IMU 

    float diameter_left_; // left wheel diameter [meter]
    float diameter_right_; // right wheel diameter [meter]
    float R_left_;  // left wheel radius [meter]
    float R_right_; // right wheel radius [meter]
    float G_[2][2];
    float Ginv_[2][2];


// Controller related
private:
    float kv_; // veloicty gain
    float kw_; // body yawing rotation gain
    


private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_optitrack_; // from vicon or optitrack. geometry_msgs::PoseStamped
    std::string topicname_optitrack_;
    geometry_msgs::PoseStamped msg_optitrack_current_;
    bool flag_optitrack_current_updated_;

    ros::Subscriber sub_position_yaw_desired_;
    std::string topicname_position_yaw_desired_;

    ros::Publisher pub_wheels_desired_; // 
    std::string topicname_wheels_desired_;

};
#endif