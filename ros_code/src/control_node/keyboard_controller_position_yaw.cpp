#include "control_node/keyboard_controller_position_yaw.h"

KeyboardControllerPositionYawROS::KeyboardControllerPositionYawROS(const ros::NodeHandle& nh)
: nh_(nh)
{
    // user input manual.
    std::stringstream ss;
    ss << "\n==============================================\n|" 
    << "  Press a key..." 
    << "\n|   DIRECTIONS : "
    << "\n|    q  w  e"
    << "\n|    a  s  d"
    << "\n|           "
    << "\n| Select an input: \n";
    user_manual_ = ss.str();
    std::cout << user_manual_;

    ss.clear();
    ss.flush();

    // ROS publisher
    pub_position_yaw_ = nh_.advertise<std_msgs::Float32MultiArray>("/amr/user_command/desired_position_yaw",1);

    // Run!
    this->run();
};


void KeyboardControllerPositionYawROS::run()
{   
    float x_des   = 0.0f;
    float y_des   = 0.0f;
    float yaw_des = 0.0f;

    float pos_step = 0.05;
    float yaw_step = 0.02;

    // Control gain 
    float kp = 2.3f;
    float ki = 0.7f;
    float kd = 12.0f;

    float step_kp = 0.01;
    float step_ki = 0.01;
    float step_kd = 0.01;

    ros::Rate rate(30); // 100 Hz
    while(ros::ok())
    {
        bool input_ok = true;
        int c = getch(); // call my own non-blocking input function.
        if(c == 'w') {
            x_des += pos_step;
            input_ok = false;
        }
        else if(c == 's') {
            x_des -= pos_step;
            input_ok = false;
        }
        else if(c == 'a') {
            y_des += pos_step;
            input_ok = false;
        }
        else if(c == 'd') {
            y_des -= pos_step;
            input_ok = false;
        }
        else if(c == 'e') {
            yaw_des -= yaw_step;
            input_ok = false;
        }
        else if(c == 'q') {
            yaw_des += yaw_step;
            input_ok = false;
        }
        else if(c == 'u') {
            kp += step_kp;
        }
        else if(c == 'j') {
            kp -= step_kp;
            if(kp < 0) kp = 0;
        }
        else if(c == 'i') {
            ki += step_ki;
        }
        else if(c == 'k') {
            ki -= step_ki;
            if(ki < 0) ki = 0;
        }
        else if(c == 'o') {
            kd += step_kd;
        }
        else if(c == 'l') {
            kd -= step_kd;
            if(kd < 0) kd = 0;
        }
        if(input_ok){
            std::cout << " des x y yaw : " << x_des << ", " << y_des << ", " << yaw_des << std::endl;
            std::cout << "pid gain : " << kp <<"," << ki <<"," << kd << std::endl;
            std::cout << user_manual_;
        }

        std_msgs::Float32MultiArray msg_position_yaw_desired;
        msg_position_yaw_desired.data.push_back(x_des);
        msg_position_yaw_desired.data.push_back(y_des);
        msg_position_yaw_desired.data.push_back(yaw_des);
        msg_position_yaw_desired.data.push_back(kp);
        msg_position_yaw_desired.data.push_back(ki);
        msg_position_yaw_desired.data.push_back(kd);

        pub_position_yaw_.publish(msg_position_yaw_desired);

        ros::spinOnce();
        rate.sleep();
    }
};