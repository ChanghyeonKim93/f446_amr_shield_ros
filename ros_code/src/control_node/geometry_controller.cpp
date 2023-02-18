#include "control_node/geometry_controller.h"

GeometryControllerROS::GeometryControllerROS(const ros::NodeHandle& nh)
: nh_(nh)
{
    flag_optitrack_current_updated_ = false;

    kv_ = 1.0f;
    kw_ = 8.0f;

    // Robot parameters
    if(!ros::param::has("~wheel_to_imu_dist_left"))
        std::runtime_error("In geometry controller node, there is no rosparam 'wheel_to_imu_dist_left'.");
    ros::param::get("~wheel_to_imu_dist_left", B_left_);
    if(!ros::param::has("~wheel_to_imu_dist_right"))
        std::runtime_error("In geometry controller node, there is no rosparam 'wheel_to_imu_dist_right'.");
    ros::param::get("~wheel_to_imu_dist_right", B_right_);

    if(!ros::param::has("~wheel_diameter_left"))
        std::runtime_error("In geometry controller node, there is no rosparam 'wheel_diameter_left'.");
    ros::param::get("~wheel_diameter_left", diameter_left_);
    R_left_ = diameter_left_*0.5f;
    if(!ros::param::has("~wheel_diameter_right"))
        std::runtime_error("In geometry controller node, there is no rosparam 'wheel_diameter_right'.");
    ros::param::get("~wheel_diameter_right", diameter_right_);
    R_right_ = diameter_right_*0.5f;

    B_ = B_left_ + B_right_;
    // [v_body;w_body] = G_*[wheel_left;wheel_right];
    G_[0][0] = B_right_/B_*R_left_;
    G_[0][1] = B_left_/B_*R_right_;
    G_[1][0] = -R_left_/B_;
    G_[1][1] = R_right_/B_;

    // [wheel_left;wheel_right] = Ginv_*[v_body;w_body];
    Ginv_[0][0] = 1.0f/R_left_;
    Ginv_[0][1] = -B_left_/R_left_;
    Ginv_[1][0] = 1.0f/R_right_;
    Ginv_[1][1] = B_right_/R_right_;

    // Topicnames
    if(!ros::param::has("~topicname_optitrack"))
        std::runtime_error("In geometry controller node, there is no rosparam 'topicname_optitrack'.");
    ros::param::get("~topicname_optitrack",topicname_optitrack_);

    if(!ros::param::has("~topicname_position_yaw_desired"))
        std::runtime_error("In geometry controller node, there is no rosparam 'topicname_position_yaw_desired'.");
    ros::param::get("~topicname_position_yaw_desired",topicname_position_yaw_desired_);
    
    if(!ros::param::has("~topicname_wheels_desired"))
        std::runtime_error("In geometry controller node, there is no rosparam 'topicname_wheels_desired'.");
    ros::param::get("~topicname_wheels_desired",topicname_wheels_desired_);

    // ROS subscriber for pose topic.
    sub_optitrack_ = nh_.subscribe<geometry_msgs::PoseStamped>
        (topicname_optitrack_, 1, &GeometryControllerROS::callbackOptitrack, this);

    sub_position_yaw_desired_ = nh_.subscribe<std_msgs::Float32MultiArray>
        (topicname_position_yaw_desired_, 1, &GeometryControllerROS::callbackDesiredPositionYaw, this);

    // ROS publisher
    pub_wheels_desired_ = nh_.advertise<std_msgs::Float32MultiArray>("/amr/user_command/wheels_desired",1);

    // Run while loop
    this->run();
};

void GeometryControllerROS::callbackOptitrack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    msg_optitrack_current_ = *msg;
    flag_optitrack_current_updated_ = true;

    // Eigen::Vector4d q_curr;
    // q_curr(0) = msg_optitrack_current_.pose.orientation.w;
    // q_curr(1) = msg_optitrack_current_.pose.orientation.x;
    // q_curr(2) = msg_optitrack_current_.pose.orientation.y;
    // q_curr(3) = msg_optitrack_current_.pose.orientation.z;

    // Eigen::Matrix3d R3d_curr = geometry::q2r(q_curr);
    //  Eigen::Matrix2d R_curr;
    // R_curr << R3d_curr.block<2,2>(0,0);
    // std::cout << "DIR CUR: " << R_curr(0,0) <<", " << R_curr(1,0) << std::endl;
    
};

void GeometryControllerROS::callbackDesiredPositionYaw(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    // data[0] = pos.x , data[1] = pos.y, data[2] = yaw angle (radian)

    if(flag_optitrack_current_updated_) {
        // If the optitrack pose was received in the short time,     
        flag_optitrack_current_updated_ = false;
        if(ros::Time::now().toSec() - msg_optitrack_current_.header.stamp.toSec() < 0.2) {
            // Do control!
            if(msg->data.size() != 3)
                std::runtime_error("In 'GeometryControllerROS::callbackDesiredPositionYaw()', msg->data.size() != 3");

            float V_MAX =  0.4;
            float V_MIN = -0.4; // maximum vel. [m/s]

            float W_MAX =  3.5; // maximum body rotation rate [rad/s]
            float W_MIN = -3.5;

            float THRES_WHEEL_ANGULAR_VELOCITY = 13.0f; // rad/s


            // Get desired
            float x_desired   = msg->data[0];
            float y_desired   = msg->data[1];
            float yaw_desired = msg->data[2];

            float kp = msg->data[3];
            float ki = msg->data[4];
            float kd = msg->data[5];

            // Get current
            float x_current   = msg_optitrack_current_.pose.position.x;
            float y_current   = msg_optitrack_current_.pose.position.y;
            // float yaw_current = msg_optitrack_current_.pose.orientation.w;

            // Calculate desired linear velocity and yawing rate
            Eigen::Vector2d err_pos;
            err_pos(0) = x_desired - x_current;
            err_pos(1) = y_desired - y_current;

            Eigen::Vector4d q_curr;
            q_curr(0) = msg_optitrack_current_.pose.orientation.w;
            q_curr(1) = msg_optitrack_current_.pose.orientation.x;
            q_curr(2) = msg_optitrack_current_.pose.orientation.y;
            q_curr(3) = msg_optitrack_current_.pose.orientation.z;

            Eigen::Matrix3d R3d_curr = geometry::q2r(q_curr);
            Eigen::Matrix2d R_curr;
            R_curr << R3d_curr.block<2,2>(0,0);
            Eigen::Matrix3d R3d_des  = geometry::a2r(0,0,yaw_desired);
            Eigen::Matrix2d R_des;
            R_des << R3d_des.block<2,2>(0,0);

            Eigen::Vector2d d_cur(R_curr(0,0),R_curr(1,0));
            Eigen::Vector2d d_des(R_des(0,0),R_des(1,0));
            std::cout << "d_des : " << d_des.transpose() << std::endl;
            std::cout << "d_cur : " << d_cur.transpose() << std::endl;
            
            float v_body_desired = kv_ * (d_cur.dot(err_pos));
            Eigen::Vector2d d_aug_des = d_des + err_pos;
            float w_body_desired = 0.5f * kw_ * (d_cur(0)*d_des(1)-d_cur(1)*d_des(0) ); 
            std::cout << "v w des : " << v_body_desired <<", " << w_body_desired << std::endl;

            // Convert v w to wheels desired.
            float wheel_desired_left  = Ginv_[0][0]*v_body_desired + Ginv_[0][1]*w_body_desired;
            float wheel_desired_right = Ginv_[1][0]*v_body_desired + Ginv_[1][1]*w_body_desired;


            // Control gain (wheel control values)

            if(wheel_desired_left >  THRES_WHEEL_ANGULAR_VELOCITY)  wheel_desired_left  =  THRES_WHEEL_ANGULAR_VELOCITY;
            if(wheel_desired_left < -THRES_WHEEL_ANGULAR_VELOCITY)  wheel_desired_left  = -THRES_WHEEL_ANGULAR_VELOCITY;
            if(wheel_desired_right >  THRES_WHEEL_ANGULAR_VELOCITY) wheel_desired_right =  THRES_WHEEL_ANGULAR_VELOCITY;
            if(wheel_desired_right < -THRES_WHEEL_ANGULAR_VELOCITY) wheel_desired_right = -THRES_WHEEL_ANGULAR_VELOCITY;


            std_msgs::Float32MultiArray msg_wheels_desired;
            msg_wheels_desired.data.push_back(wheel_desired_left);
            msg_wheels_desired.data.push_back(wheel_desired_right);
            msg_wheels_desired.data.push_back(kp);
            msg_wheels_desired.data.push_back(ki);
            msg_wheels_desired.data.push_back(kd);

            pub_wheels_desired_.publish(msg_wheels_desired);

        }
        else{
            ROS_WARN_STREAM("Optitrack (Vicon) pose is too old to be used for controlling the robot!!\n Optitrack node might be died or too low update rate is set. Please check it.");
        }


    }
};


void GeometryControllerROS::run()
{
    ros::Rate rate(5000); // run fast!
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
};