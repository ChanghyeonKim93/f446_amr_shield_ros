#ifndef _ROSTOPIC_COMMUNICATOR_H_
#define _ROSTOPIC_COMMUNICATOR_H_

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h> // from nucleo, to nucleo

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/JointState.h>

#include <std_msgs/UInt16MultiArray.h> // Drone PWM messages
#include <std_msgs/Float32MultiArray.h> // AMR control messages

#include "../utility/union_struct.h"

#define CAMERA_TRIGGER_LOW  0b01010101
#define CAMERA_TRIGGER_HIGH 0b10101010

class ROSTopicCommunicator {
private:
    void callbackSerialFromNucleo(const std_msgs::Int8MultiArray::ConstPtr& msg);

    void callbackControlInputs_Drone(const std_msgs::UInt16MultiArray::ConstPtr& msg); // eight PWM signals
    void callbackControlInputs_AMR(const std_msgs::Float32MultiArray::ConstPtr& msg); // two desired angular velocities, and three motor gains (P,I,D). All data is in 'float'.
 
    void run();

public:
// Constructor
    ROSTopicCommunicator(ros::NodeHandle& nh) ;



// User command (desired wheel angular velocity only)
private:
    double w_left_desired_;
    double w_right_desired_;


        // datatype::FLOAT_UNION kp,kd,ki;
        // w_left_desired_.float_  = 0.0f;
        // w_right_desired_.float_ = 0.0f;
        // kp.float_ = 1.1;//1.5;
        // kd.float_ = 0.07;//10.5;
        // ki.float_ = 0.0;//0.8;

        // // 2. Send TCP/IP data (to MCU)
        // // sprintf(buff_snd_, "%d : %s", len_read, buff_rcv_);
        // for(int j = 0; j < 4; ++j) buff_snd_[j]    = w_left_desired_.bytes_[j];
        // for(int j = 0; j < 4; ++j) buff_snd_[j+4]  = w_right_desired_.bytes_[j];
        // for(int j = 0; j < 4; ++j) buff_snd_[j+8]  = kp.bytes_[j];
        // for(int j = 0; j < 4; ++j) buff_snd_[j+12] = kd.bytes_[j];
        // for(int j = 0; j < 4; ++j) buff_snd_[j+16] = ki.bytes_[j];

// Sensor data from the AMR
private:
// IMU related
    double time_; // in sec.
    double acc_[3]; // in m/s^2
    double gyro_[3]; // in radian per sec.
    double mag_[3]; // in milli Gauss

    double acc_scale_;
    double gyro_scale_;
    double mag_scale_;

    // ADC related
    float adc_data_[2]; // in volt

    // Sonar related
    float sonar_data_; // in meter

    // Encoder related
    float encoder_data_[2]; // in radian per sec.

    // ROS nodehandle
    ros::NodeHandle nh_;
    ros::Subscriber sub_serial_; 
    ros::Publisher  pub_serial_;

    ros::Subscriber sub_user_command_drone_;
    ros::Subscriber sub_user_command_amr_;

    std::string topicname_user_command_drone_;
    std::string topicname_user_command_amr_;

    // ROS Publishers
    ros::Publisher pub_imu_; // IMU data
    ros::Publisher pub_battery_state_[2]; // Battery voltage data
    ros::Publisher pub_sonar_; // Sonar data (only for DRONE_SHIELD)

    ros::Publisher pub_encoders_; // Wheel encoder data (only for AMR_SHIELD)


};
#endif