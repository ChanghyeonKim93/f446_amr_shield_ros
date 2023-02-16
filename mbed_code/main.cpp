/*
    F446_DRONE_SHIELD
        Designed by Changhyeon Kim, 2023.01.
        e-mail: hyun91015 at gmail.com
    See documentation and software in the below github link:
        https://github.com/ChanghyeonKim93/f446_drone_shield_ros
*/

/*  YOU CAN CHANGE THE PARAMETERS IN 'parameter.h'
    PLEASE SELECT THE MODE: 
        - DRONE_MODE
        - AMR_MODE
    
*/
#include "parameters.h" 

// #define DRONE_MODE
#define AMR_MODE

// ==============================================================================
// ==============================================================================
// ======================== DO NOT MODIFY FROM THIS LINE ========================
// ==============================================================================
// ==============================================================================
#include "mbed.h"
#include "union_struct.h"
#include <chrono>




// Set event queue and worker thread
Thread thread_poll(osPriorityRealtime); // polling all interrupt signals. https://os.mbed.com/docs/mbed-os/v6.10/apis/thread.html
EventQueue event_queue(128 * EVENTS_EVENT_SIZE);

// LED signal functions
DigitalOut led_signal(LED1); // On-board green LED. (PA_5)
void startSignalLED(DigitalOut& led);
void ledSignals_OK(DigitalOut& led, int n_blink);

#ifdef DRONE_MODE
    // Sonar distance sensor parameters
    #include "ultrasonic_library/ultrasonic.h"
    USHORT_UNION sonar_dist_mm;
    void sonarCallbackFunction(int dist) { return; };
    ULTRASONIC ultra_sonic(SONAR_TRG, SONAR_ECHO, SONAR_SAMPLING_PERIOD_MS, SONAR_TIMEOUT_MS, &sonarCallbackFunction);

    // Drone Motors PWM signals
    #include "drone_pwm_library/drone_motor_pwm.h"
    uint16_t pwm_values[8] = {0,0,0,0,0,0,0,0};
    DroneMotorPwm motor_pwm;
    void setDroneMotorPWM_01234567(uint16_t pwm_ushort[8]) {
        for(int i = 0; i < 8; ++i){
            if(pwm_ushort[i] < 0 )   pwm_ushort[i] = 0;
            if(pwm_ushort[i] > 4095) pwm_ushort[i] = 4095;
        }
        motor_pwm.setPWM_all(pwm_ushort);
    };
#endif

#ifdef AMR_MODE
    volatile uint8_t control_step  = 8;
    volatile uint8_t control_count = 0;

    // AMR Motors PWM signals
    #include "amr_motor_library/dcmotor.h"
    float wheels_desired[2] = {0,0};
    float wheels_current[2] = {0,0};
    float pid_gains[3]      = {0,0,0};
    DCMOTOR motor_left(MOTOR_LEFT_PWM, IN0_LEFT, IN1_LEFT);
    DCMOTOR motor_right(MOTOR_RIGHT_PWM, IN1_RIGHT, IN0_RIGHT);

    // Encoder library
    #include "encoder_library/motor_encoder.h"
    // TIM_Encoder_InitTypeDef encoder3, encoder4;
    // TIM_HandleTypeDef       timer3, timer4;
    FLOAT_UNION radian_per_sec_A;
    FLOAT_UNION radian_per_sec_B;
    MotorEncoder motor_encoder(MOTOR_ENCODER_PERIOD_MS);
#endif

// Analog-Digital Converter (Battery Voltage and current)
USHORT_UNION adc1_voltage_ushort;
USHORT_UNION adc2_voltage_ushort;
AnalogIn adc1(ADC1_PIN);
AnalogIn adc2(ADC2_PIN);

// Camera Trigger signal
#define CAMERA_TRIGGER_LOW  0b01010101
#define CAMERA_TRIGGER_HIGH 0b10101010
volatile uint8_t trigger_step  = 25;
volatile uint8_t trigger_count = 0;
volatile uint8_t flag_camera_trigger = CAMERA_TRIGGER_LOW;
DigitalOut signal_trigger(TRIGGER_PIN);


// Serial communication
#include "serial_library/serial_comm_mbed.h"
Timeout timeout_serial_read;
void flipLED() { led_signal = !led_signal; };

uint8_t  packet_send[256] = {0};
uint8_t  packet_recv[256] = {0};
uint32_t len_packet_recv = 0;
uint32_t len_packet_send = 0;
enum MessageTypeByLength {
    PWMCOMMAND = 17,
    AMRCOMMAND = 21
};
SerialCommunicatorMbed serial_usb(BAUD_RATE, SERIAL_TX_PIN, SERIAL_RX_PIN);

// IMU
#include "icm42605_library/icm42605_spi.h"
ICM42605_SPI imu;


// Define the behavior function when the serial data is received.
void workfunction_readSerialUSB() {
    if(serial_usb.tryToReadSerialBuffer()) { // packet ready!
        // If the data is received from the PC, toggle the LED (the green LED on the Nucleo board)

        // Get message from the PC
        int len_recv_message = 0;
        len_recv_message = serial_usb.getReceivedMessage(packet_recv); 

        if( len_recv_message > 0) {
            led_signal = 1;
            timeout_serial_read.attach(callback(&flipLED), 5ms);

#ifdef DRONE_MODE
            if(packet_recv[0] == 0) 
            {
                // For safety, initialize all signals with zero.
                pwm_values[0] = 0; pwm_values[1] = 0;
                pwm_values[2] = 0; pwm_values[3] = 0;
                pwm_values[4] = 0; pwm_values[5] = 0;
                pwm_values[6] = 0; pwm_values[7] = 0;
                if( len_recv_message == MessageTypeByLength::PWMCOMMAND ) 
                { 
                    // Successfully received the packet.
                    // In case of 8 PWM signals.
                    // ======== USER-DEFINED CODE START ======== 
                    USHORT_UNION pwm_tmp;
                    pwm_tmp.bytes_[0] = packet_recv[1];  pwm_tmp.bytes_[1] = packet_recv[2];     pwm_values[0] = pwm_tmp.ushort_;
                    pwm_tmp.bytes_[0] = packet_recv[3];  pwm_tmp.bytes_[1] = packet_recv[4];     pwm_values[1] = pwm_tmp.ushort_;
                    pwm_tmp.bytes_[0] = packet_recv[5];  pwm_tmp.bytes_[1] = packet_recv[6];     pwm_values[2] = pwm_tmp.ushort_;
                    pwm_tmp.bytes_[0] = packet_recv[7];  pwm_tmp.bytes_[1] = packet_recv[8];     pwm_values[3] = pwm_tmp.ushort_;
                    pwm_tmp.bytes_[0] = packet_recv[9];  pwm_tmp.bytes_[1] = packet_recv[10];    pwm_values[4] = pwm_tmp.ushort_;
                    pwm_tmp.bytes_[0] = packet_recv[11]; pwm_tmp.bytes_[1] = packet_recv[12];    pwm_values[5] = pwm_tmp.ushort_;
                    pwm_tmp.bytes_[0] = packet_recv[13]; pwm_tmp.bytes_[1] = packet_recv[14];    pwm_values[6] = pwm_tmp.ushort_;
                    pwm_tmp.bytes_[0] = packet_recv[15]; pwm_tmp.bytes_[1] = packet_recv[16];    pwm_values[7] = pwm_tmp.ushort_;

                    setDroneMotorPWM_01234567(pwm_values);     
                }
            }
#endif

#ifdef AMR_MODE
            if (packet_recv[0] == 1)// AMR mode
            {
                // For safety, initialize all signals with zero. (desired only)
                wheels_desired[0] = 0;
                wheels_desired[1] = 0;

                if( len_recv_message == MessageTypeByLength::AMRCOMMAND)
                {
                    FLOAT_UNION ftmp;
                    ftmp.bytes_[0] = packet_recv[1]; 
                    ftmp.bytes_[1] = packet_recv[2]; 
                    ftmp.bytes_[2] = packet_recv[3]; 
                    ftmp.bytes_[3] = packet_recv[4];   
                    wheels_desired[0] = ftmp.float_;

                    ftmp.bytes_[0] = packet_recv[5]; 
                    ftmp.bytes_[1] = packet_recv[6]; 
                    ftmp.bytes_[2] = packet_recv[7]; 
                    ftmp.bytes_[3] = packet_recv[8];   
                    wheels_desired[1] = ftmp.float_;

                    ftmp.bytes_[0] = packet_recv[9]; 
                    ftmp.bytes_[1] = packet_recv[10]; 
                    ftmp.bytes_[2] = packet_recv[11]; 
                    ftmp.bytes_[3] = packet_recv[12];   
                    pid_gains[0] = ftmp.float_;

                    ftmp.bytes_[0] = packet_recv[13]; 
                    ftmp.bytes_[1] = packet_recv[14]; 
                    ftmp.bytes_[2] = packet_recv[15]; 
                    ftmp.bytes_[3] = packet_recv[16];   
                    pid_gains[1] = ftmp.float_;

                    ftmp.bytes_[0] = packet_recv[17]; 
                    ftmp.bytes_[1] = packet_recv[18]; 
                    ftmp.bytes_[2] = packet_recv[19]; 
                    ftmp.bytes_[3] = packet_recv[20];   
                    pid_gains[2] = ftmp.float_;

                    // controlDCMotor(wheels_desired, pid_gains);
                }
                else { // for safety.
                    wheels_desired[0] = 0;
                    wheels_desired[1] = 0;
                    // controlDCMotor(wheels_desired, pid_gains);
                }
            }
#endif
        }

    }
};
void workfunction_sendSerialUSB() {
    if(serial_usb.writable()){
        serial_usb.send_withChecksum(packet_send, len_packet_send);
        len_packet_send = 0;
    }
};
void tryToReadSerialUSB(){ event_queue.call(workfunction_readSerialUSB); };
void tryToSendSerialUSB(){ event_queue.call(workfunction_sendSerialUSB); };

// Time Reference 
Timer timer;
uint64_t us_curr;
USHORT_UNION tsec;
UINT_UNION   tusec;

// MAIN FUNCTION
int main()
{ 
    // printf("Start F446 DRONE SHIELD\r\n");
    startSignalLED(led_signal);

    // Start the event queue
    thread_poll.start(callback(&event_queue, &EventQueue::dispatch_forever));

    // Start Timer
    timer.start();
    std::chrono::microseconds time_send_prev = timer.elapsed_time();
    std::chrono::microseconds time_recv_prev = timer.elapsed_time();
    std::chrono::microseconds time_curr;
    ledSignals_OK(led_signal, 1);

    // Ultrasonic initialize
#ifdef DRONE_MODE
    ultra_sonic.startUpdates();
#endif
    ledSignals_OK(led_signal, 2);

    // IMU initialize
    /* Specify sensor parameters (sample rate is twice the bandwidth)
    * choices are:
        AFS_2G, AFS_4G, AFS_8G, AFS_16G  
        GFS_15_125DPS, GFS_31_25DPS, GFS_62_5DPS, GFS_125DPS, GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS 
        AODR_1_5625Hz, AODR_3_125Hz, AODR_6_25Hz, AODR_12_5Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_500Hz, AODR_1000Hz, AODR_2000Hz, AODR_4000Hz, AODR_8000Hz
        GODR_12_5Hz, GODR_25Hz, GODR_50Hz, GODR_100Hz, GODR_200Hz, GODR_500Hz, GODR_1000Hz, GODR_2000Hz, GODR_4000Hz, GODR_8000Hz
    */ 
    uint8_t Ascale = AFS_4G, Gscale = GFS_1000DPS, AODR = AODR_1000Hz, GODR = GODR_1000Hz;
    FLOAT_UNION ax, ay, az, gx, gy, gz, mx, my, mz;
    FLOAT_UNION aRes, gRes, mRes;
    int16_t imu_raw[7];        // Stores the 16-bit signed sensor output
    imu.reset(); // software reset ICM42605 to default registers
    aRes.float_ = imu.getAres(Ascale);
    gRes.float_ = imu.getGres(Gscale);
    imu.init(Ascale, Gscale, AODR, GODR);
    ledSignals_OK(led_signal, 3);

    // Encoder initialize
    ledSignals_OK(led_signal, 4);


    // Do while loop
    while (true) {
        time_curr = timer.elapsed_time();
        std::chrono::duration<int, std::micro> delta_time = time_curr - time_send_prev;

        if(delta_time.count() > LOOP_PERIOD_US) { // 2.5 ms interval (400 Hz)
#ifdef DRONE_MODE
            // Get ultrasonic distance
            ultra_sonic.checkDistance(); //call checkDistance() as much as possible, as this is where the class checks if dist needs to be called.
            sonar_dist_mm.ushort_ = ultra_sonic.getCurrentDistance(); // timeout 
#endif

#ifdef AMR_MODE
            // Get encoder values
            int32_t dcnt_A = motor_encoder.getDeltaCounter_A();
            int32_t dcnt_B = motor_encoder.getDeltaCounter_B();
            radian_per_sec_A.float_ = motor_encoder.getAngularVelocity_A();
            radian_per_sec_B.float_ = motor_encoder.getAngularVelocity_B();
#endif
            // Get ADC values
            adc1_voltage_ushort.ushort_ = adc1.read_u16();
            adc2_voltage_ushort.ushort_ = adc2.read_u16();

            // Get IMU data
            imu.readData(imu_raw); 
            ax.float_ = (float)imu_raw[1]*aRes.float_; // get actual g value, this depends on scale being set
            ay.float_ = (float)imu_raw[2]*aRes.float_;   
            az.float_ = (float)imu_raw[3]*aRes.float_;  
            gx.float_ = (float)imu_raw[4]*gRes.float_; // get actual gyro value, this depends on scale being set
            gy.float_ = (float)imu_raw[5]*gRes.float_;
            gz.float_ = (float)imu_raw[6]*gRes.float_; 
            mx.float_ = 0*mRes.float_;
            my.float_ = 0*mRes.float_;
            mz.float_ = 0*mRes.float_;

            // If serial USB can be written,
            if(serial_usb.writable()) { 
                 /*
                    <Packet structure>
                     Total 57 bytes
                     data : ax ay az gx gy gz mx my mz tsec tusec trgr ADC0 ADC1 SONAR ENC0 ENC1
                     byte : 4f 4f 4f 4f 4f 4f 4f 4f 4f  2ui   4ui   1c  2ui  2ui   2ui   4f   4f
                */

                // Time 
                us_curr      = timer.elapsed_time().count(); // Current time
                tsec.ushort_ = (uint16_t)(us_curr/1000000);
                tusec.uint_  = (uint32_t)(us_curr-((uint32_t)tsec.ushort_)*1000000);

                // Camera trigger signal output.
                ++trigger_count;
                if(trigger_count >= trigger_step) {
                    trigger_count = 0;
                    flag_camera_trigger = CAMERA_TRIGGER_HIGH;
                    signal_trigger = 1;
                    wait_us(100);
                    signal_trigger = 0;
                }
#ifdef AMR_MODE
                motor_left.updateAngularVelocity(radian_per_sec_A.float_, 0.0025);
                motor_right.updateAngularVelocity(-radian_per_sec_B.float_, 0.0025);
                
                ++control_count;
                if(control_count >= control_step) {
                    control_count = 0;
                    motor_left.setPIDGains(pid_gains[0],pid_gains[1],pid_gains[2]);
                    motor_right.setPIDGains(pid_gains[0],pid_gains[1],pid_gains[2]);
                    motor_left.controlAngularVelocity(wheels_desired[0]);
                    motor_right.controlAngularVelocity(wheels_desired[1]);
                }
#endif
                // IMU data (3D acc, 3D gyro, 3D magnetometer)
                packet_send[0] = ax.bytes_[0]; packet_send[1] = ax.bytes_[1];  packet_send[2] = ax.bytes_[2];  packet_send[3] = ax.bytes_[3];
                packet_send[4] = ay.bytes_[0]; packet_send[5] = ay.bytes_[1];  packet_send[6] = ay.bytes_[2];  packet_send[7] = ay.bytes_[3];
                packet_send[8] = az.bytes_[0]; packet_send[9] = az.bytes_[1]; packet_send[10] = az.bytes_[2]; packet_send[11] = az.bytes_[3];

                packet_send[12] = gx.bytes_[0]; packet_send[13] = gx.bytes_[1]; packet_send[14] = gx.bytes_[2]; packet_send[15] = gx.bytes_[3];
                packet_send[16] = gy.bytes_[0]; packet_send[17] = gy.bytes_[1]; packet_send[18] = gy.bytes_[2]; packet_send[19] = gy.bytes_[3];
                packet_send[20] = gz.bytes_[0]; packet_send[21] = gz.bytes_[1]; packet_send[22] = gz.bytes_[2]; packet_send[23] = gz.bytes_[3];

                packet_send[24] = mx.bytes_[0]; packet_send[25] = mx.bytes_[1]; packet_send[26] = mx.bytes_[2]; packet_send[27] = mx.bytes_[3];
                packet_send[28] = my.bytes_[0]; packet_send[29] = my.bytes_[1]; packet_send[30] = my.bytes_[2]; packet_send[31] = my.bytes_[3];
                packet_send[32] = mz.bytes_[0]; packet_send[33] = mz.bytes_[1]; packet_send[34] = mz.bytes_[2]; packet_send[35] = mz.bytes_[3];

                packet_send[36]  = tsec.bytes_[0];  // time (second part, low)
                packet_send[37]  = tsec.bytes_[1];  // time (second part, high)

                packet_send[38]  = tusec.bytes_[0]; // time (microsecond part, lowest)
                packet_send[39]  = tusec.bytes_[1]; // time (microsecond part, low)
                packet_send[40]  = tusec.bytes_[2]; // time (microsecond part, high)
                packet_send[41]  = tusec.bytes_[3]; // time (microsecond part, highest)

                // Camera trigger state
                packet_send[42]     = flag_camera_trigger; // 'CAMERA_TRIGGER_HIGH' or 'CAMERA_TRIGGER_LOW'
                flag_camera_trigger = CAMERA_TRIGGER_LOW;

                // AnalogIn (battery voltage data)
                packet_send[43]  = adc1_voltage_ushort.bytes_[0];
                packet_send[44]  = adc1_voltage_ushort.bytes_[1];
                packet_send[45]  = adc2_voltage_ushort.bytes_[0];
                packet_send[46]  = adc2_voltage_ushort.bytes_[1];

#ifdef DRONE_MODE
                // Sonar distance 
                packet_send[47] = sonar_dist_mm.bytes_[0];
                packet_send[48] = sonar_dist_mm.bytes_[1];
#endif

#ifdef AMR_MODE
                // Encoder signals (4 bytes per encoder, values in float)
                packet_send[49] = radian_per_sec_A.bytes_[0];
                packet_send[50] = radian_per_sec_A.bytes_[1];
                packet_send[51] = radian_per_sec_A.bytes_[2];
                packet_send[52] = radian_per_sec_A.bytes_[3];

                packet_send[53] = radian_per_sec_B.bytes_[0];
                packet_send[54] = radian_per_sec_B.bytes_[1];
                packet_send[55] = radian_per_sec_B.bytes_[2];
                packet_send[56] = radian_per_sec_B.bytes_[3]; // total 
#endif
                // Send length
                len_packet_send = 57;

                tryToSendSerialUSB(); // Send!

                // flag_imu_ready = false;
                time_send_prev = time_curr;

            }
        }

        // Read data if data exists.
        if(serial_usb.readable()) {
            tryToReadSerialUSB();
        }
    }
}

void startSignalLED(DigitalOut& led){
    for(int i = 0; i < 16; ++i) {
        led = 1; ThisThread::sleep_for(20ms);
        led = 0; ThisThread::sleep_for(20ms);
    }
    ThisThread::sleep_for(400ms);

    // Guk-Bbong bit
    // led = 1;    ThisThread::sleep_for(100ms);
    // led = 0;    ThisThread::sleep_for(100ms);
    // led = 1;    ThisThread::sleep_for(100ms);
    // led = 0;    ThisThread::sleep_for(300ms);
    // led = 1;    ThisThread::sleep_for(100ms);
    // led = 0;    ThisThread::sleep_for(100ms);
    // led = 1;    ThisThread::sleep_for(100ms);
    // led = 0;    ThisThread::sleep_for(300ms);
    // led = 1;    ThisThread::sleep_for(100ms);
    // led = 0;    ThisThread::sleep_for(50ms);
    // ThisThread::sleep_for(500ms);
};

void ledSignals_OK(DigitalOut& led, int n_blink){
    for(int i = 0; i < n_blink; ++i) {
        led = 1; ThisThread::sleep_for(150ms);
        led = 0; ThisThread::sleep_for(75ms);
    }
    ThisThread::sleep_for(200ms);
};

