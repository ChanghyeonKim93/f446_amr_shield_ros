#include "keyboard_control_node/keyboard_controller.h"

KeyboardControllerROS::KeyboardControllerROS(const ros::NodeHandle& nh)
: nh_(nh)
{
    // user input manual.
    std::stringstream ss;
    ss << "\n==============================================\n|" 
    << "  Press a key..." 
    << "\n|   DIRECTIONS : "
    << "\n|    q  w  e"
    << "\n|    a  s  d"
    << "\n|    z     c"
    << "\n" 
    << "\n|    u: P gain ++ (0.05)"
    << "\n|    i: I gain ++ (0.05)"
    << "\n|    o: D gain ++ (0.05)"
    << "\n"
    << "\n|    j: P gain -- (0.05)"
    << "\n|    k: I gain -- (0.05)"
    << "\n|    l: D gain -- (0.05)"
    << "\n| Select an input: \n";
    user_manual_ = ss.str();
    std::cout << user_manual_;

    ss.clear();
    ss.flush();

    // ROS publisher
    pub_wheels_desired_ = nh_.advertise<std_msgs::Float32MultiArray>("/amr/user_command/wheels_desired",1);

    // Run!
    this->run();
};


void KeyboardControllerROS::run()
{   
    // AMR specification
    float R         = 0.14407 * 0.5f; // wheel radius
    float B         = 0.33; // disp. between two wheel centres
    float invr      = 1.0/R;
    float Bhalfinvr = B/(2.0*R);
    
    // Desired values (linear velocity & body yawing rate)
    float v_d = 0.0f;
    float w_d = 0.0f;
    
    float v_step = 0.0025/1.5;
    float w_step = 0.007/1.5;

    float V_MAX =  0.5;
    float V_MIN = -0.5; // maximum vel. [m/s]

    float W_MAX =  0.16; // maximum rotation rate [rad/s]
    float W_MIN = -0.16;

    // wheel rotational velocity
    float wheel_left_d  = 0.0f;
    float wheel_right_d = 0.0f;
    float THRES_WHEEL   = 2.0f;

    // Control gain 
    float kp = 1.0f;
    float ki = 0.0f;
    float kd = 1.0f;

    float step_kp = 0.05;
    float step_ki = 0.05;
    float step_kd = 0.05;

    
    ros::Rate rate(100); // 100 Hz
    while(ros::ok())
    {
        bool input_ok = true;
        int c = getch(); // call my own non-blocking input function.
        if(c == 'w') {
            v_d += v_step;
            if(v_d >= V_MAX) v_d = V_MAX;
        }
        else if(c == 's') {
            v_d -= v_step;
            if(v_d <= V_MIN) v_d = V_MIN;
        }
        else if(c == 'd') {
            w_d += w_step;
            if(w_d >= W_MAX) w_d = W_MAX;
        }
        else if(c == 'a') {
            w_d -= w_step;
            if(w_d <= W_MIN) w_d = W_MIN;
        }
        else if(c == 'q') {
            v_d += v_step;   
            w_d -= w_step;
            if(v_d >= V_MAX) v_d = V_MAX;
            if(w_d <= W_MIN) w_d = W_MIN;
        }
        else if(c == 'e') {
            v_d += v_step;   
            w_d += w_step;
            if(v_d >= V_MAX) v_d = V_MAX;
            if(w_d >= W_MAX) w_d = W_MAX;
        }
        else if(c == 'z') {
            v_d -= v_step;   
            w_d += w_step;
            if(v_d <= V_MIN) v_d = V_MIN;
            if(w_d >= W_MAX) w_d = W_MAX;
        }
        else if(c == 'c') {
            v_d -= v_step;   
            w_d -= w_step;
            if(v_d <= V_MIN) v_d = V_MIN;
            if(w_d <= W_MIN) w_d = W_MIN;
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
        else {
            // if no keyboard input, naturally deccelerate.
            float v_abs = fabs(v_d);
            v_abs -= 0.5*v_step;
            v_d = v_abs*((v_d > 0) - (v_d < 0));

            float w_abs = fabs(w_d);
            w_abs -= 0.5*w_step;
            w_d = w_abs*((w_d > 0) - (w_d < 0));

            if(fabs(v_d) < 1.5*v_step) v_d = 0;
            if(fabs(w_d) < 1.5*w_step) w_d = 0;

            input_ok = false;
        } 

        if(input_ok){
            std::cout << "\n";
            std::cout << "Desired values:\n";
            std::cout << "v, w: [" << v_d <<", " << w_d << "] / " 
                      << "kp,ki,kd: [" << kp <<", " << ki << ", " << kd << "]\n\n";
            std::cout << user_manual_;
        }


        wheel_left_d  = invr*v_d - Bhalfinvr*w_d;
        wheel_right_d = invr*v_d + Bhalfinvr*w_d;

        if(wheel_left_d >  THRES_WHEEL)  wheel_left_d  =  THRES_WHEEL;
        if(wheel_left_d < -THRES_WHEEL)  wheel_left_d  = -THRES_WHEEL;
        if(wheel_right_d >  THRES_WHEEL) wheel_right_d =  THRES_WHEEL;
        if(wheel_right_d < -THRES_WHEEL) wheel_right_d = -THRES_WHEEL;

        std_msgs::Float32MultiArray msg_wheels_desired;
        msg_wheels_desired.data.push_back(wheel_left_d);
        msg_wheels_desired.data.push_back(wheel_right_d);

        pub_wheels_desired_.publish(msg_wheels_desired);

        ros::spinOnce();
        rate.sleep();
    }
};