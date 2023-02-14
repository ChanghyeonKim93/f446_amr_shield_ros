#include "rostopic_node/rostopic_communicator.h"

ROSTopicCommunicator::ROSTopicCommunicator(ros::NodeHandle& nh) 
: nh_(nh) 
{
    acc_scale_   = 9.8065; // m/s2
    gyro_scale_  = M_PI/180.0f; // rad/s
    mag_scale_   = 10.0*4219.0/32760.0; // milliGauss

    // serial_
    sub_serial_ = nh_.subscribe<std_msgs::Int8MultiArray>("/serial/pc/from_nucleo",1, &ROSTopicCommunicator::callbackSerialFromNucleo, this);
    pub_serial_ = nh_.advertise<std_msgs::Int8MultiArray>("/serial/pc/to_nucleo", 1);

    // publisher
    pub_imu_ = nh_.advertise<sensor_msgs::Imu>("/icm42605/imu",1);

    pub_battery_state_[0] = nh.advertise<sensor_msgs::BatteryState>("/battery_state/0",1);
    pub_battery_state_[1] = nh.advertise<sensor_msgs::BatteryState>("/battery_state/1",1);

    pub_sonar_ = nh_.advertise<sensor_msgs::Range>("/hcsr04/range",1);

    pub_encoders_ = nh_.advertise<sensor_msgs::JointState>("/wheel_encoders",1);

    
    this->run();
};


void ROSTopicCommunicator::callbackSerialFromNucleo(const std_msgs::Int8MultiArray::ConstPtr& msg){
    
    if(msg->data.size() == 57)
    {
        // AMR shield.
        
        FLOAT_UNION val;
        int idx = 0;
        val.bytes_[0] = msg->data[idx]; val.bytes_[1] = msg->data[++idx]; val.bytes_[2] = msg->data[++idx]; val.bytes_[3] = msg->data[++idx];
            acc_[0] = (double)val.float_ * acc_scale_;

        idx = 4; val.bytes_[0] = msg->data[idx]; val.bytes_[1] = msg->data[++idx]; val.bytes_[2] = msg->data[++idx]; val.bytes_[3] = msg->data[++idx];
            acc_[1] = (double)val.float_ * acc_scale_;

        idx = 8; val.bytes_[0] = msg->data[idx]; val.bytes_[1] = msg->data[++idx]; val.bytes_[2] = msg->data[++idx]; val.bytes_[3] = msg->data[++idx];
            acc_[2] = (double)val.float_ * acc_scale_;

        idx = 12; val.bytes_[0] = msg->data[idx]; val.bytes_[1] = msg->data[++idx]; val.bytes_[2] = msg->data[++idx]; val.bytes_[3] = msg->data[++idx];
            gyro_[0] = (double)val.float_ * gyro_scale_;

        idx = 16; val.bytes_[0] = msg->data[idx]; val.bytes_[1] = msg->data[++idx]; val.bytes_[2] = msg->data[++idx]; val.bytes_[3] = msg->data[++idx];
            gyro_[1] = (double)val.float_ * gyro_scale_;

        idx = 20; val.bytes_[0] = msg->data[idx]; val.bytes_[1] = msg->data[++idx]; val.bytes_[2] = msg->data[++idx]; val.bytes_[3] = msg->data[++idx];
            gyro_[2] = (double)val.float_ * gyro_scale_;

        idx = 24; val.bytes_[0] = msg->data[idx]; val.bytes_[1] = msg->data[++idx]; val.bytes_[2] = msg->data[++idx]; val.bytes_[3] = msg->data[++idx];
            mag_[0] = (double)val.float_ * mag_scale_;

        idx = 28; val.bytes_[0] = msg->data[idx]; val.bytes_[1] = msg->data[++idx]; val.bytes_[2] = msg->data[++idx]; val.bytes_[3] = msg->data[++idx];
            mag_[1] = (double)val.float_ * mag_scale_;

        idx = 32; val.bytes_[0] = msg->data[idx]; val.bytes_[1] = msg->data[++idx]; val.bytes_[2] = msg->data[++idx]; val.bytes_[3] = msg->data[++idx];
            mag_[2] = (double)val.float_ * mag_scale_;

        USHORT_UNION sec;
        UINT_UNION   usec;
        idx = 36;
        sec.bytes_[0]  = msg->data[idx];  sec.bytes_[1] = msg->data[++idx];

        idx = 37;
        usec.bytes_[0] = msg->data[idx]; usec.bytes_[1] = msg->data[++idx];
        usec.bytes_[2] = msg->data[++idx]; usec.bytes_[3] = msg->data[++idx];

        idx = 42;
        uint8_t cam_trigger_state = msg->data[idx];
        
            time_ = ((double)sec.ushort_ + (double)usec.uint_/1000000.0);

        // AnalogRead data
        USHORT_UNION adc[2];
        idx = 43;
        adc[0].bytes_[0] = msg->data[idx];   adc[0].bytes_[1] = msg->data[++idx];
        adc[1].bytes_[0] = msg->data[++idx]; adc[1].bytes_[1] = msg->data[++idx];

            adc_data_[0] = (float)(adc[0].ushort_)*3.3f/4095.0f;
            adc_data_[1] = (float)(adc[1].ushort_)*3.3f/4095.0f;

        // Sonar distance
        idx = 47;
        USHORT_UNION sonar_dist_in_mm;
        sonar_dist_in_mm.bytes_[0] = msg->data[idx]; sonar_dist_in_mm.bytes_[1] = msg->data[++idx]; 
        
            sonar_data_ = (float)(sonar_dist_in_mm.ushort_) * 0.001f; // meter

        // Encoder data
        FLOAT_UNION encoder_A, encoder_B;
        idx = 49; encoder_A.bytes_[0] = msg->data[idx]; encoder_A.bytes_[1] = msg->data[++idx]; encoder_A.bytes_[2] = msg->data[++idx]; encoder_A.bytes_[3] = msg->data[++idx];
        idx = 53; encoder_B.bytes_[0] = msg->data[idx]; encoder_B.bytes_[1] = msg->data[++idx]; encoder_B.bytes_[2] = msg->data[++idx]; encoder_B.bytes_[3] = msg->data[++idx]; 

            encoder_data_[0] = encoder_A.float_;
            encoder_data_[1] = -encoder_B.float_;



        // Fill & publish IMU data
        ros::Time time_cur = ros::Time::now();

        sensor_msgs::Imu msg;
        msg.header.stamp = time_cur;

        msg.angular_velocity.x = gyro_[0];
        msg.angular_velocity.y = gyro_[1];
        msg.angular_velocity.z = gyro_[2];
        msg.linear_acceleration.x = acc_[0];
        msg.linear_acceleration.y = acc_[1];
        msg.linear_acceleration.z = acc_[2];
        
        pub_imu_.publish(msg);

        // Fill & publish battery state data
        float analog_in_scaler = 3.3f/65535.0f;
        sensor_msgs::BatteryState msg_bat[4];
        for(int j = 0; j < 2; ++j){
            msg_bat[j].header.stamp = time_cur;
            msg_bat[j].voltage = (float)adc[j].ushort_*analog_in_scaler;

            pub_battery_state_[j].publish(msg_bat[j]);
        }

        // Fill & publish sonar range data
        sensor_msgs::Range msg_sonar;
        msg_sonar.header.stamp = time_cur;
        msg_sonar.radiation_type = sensor_msgs::Range::ULTRASOUND;
        msg_sonar.field_of_view = 10/180.0*M_PI; // radian
        msg_sonar.range = (float)(sonar_dist_in_mm.ushort_)*0.001f; // meters
        msg_sonar.min_range = 0.05f; // meters
        msg_sonar.max_range = 3.3f; // meters

        pub_sonar_.publish(msg_sonar);

        // Fill & publish encoder data
        sensor_msgs::JointState msg_encoders;
        msg_encoders.header.stamp = time_cur;
        msg_encoders.name.push_back("encoder_left");
        msg_encoders.name.push_back("encoder_right");
        msg_encoders.position.push_back(0.075);
        msg_encoders.position.push_back(0.075);
        msg_encoders.effort.push_back(0);
        msg_encoders.effort.push_back(0);
        msg_encoders.velocity.push_back(encoder_data_[0]);
        msg_encoders.velocity.push_back(encoder_data_[1]);

        pub_encoders_.publish(msg_encoders);
    }
};  

void ROSTopicCommunicator::run(){
    ros::Rate rate(20000);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}