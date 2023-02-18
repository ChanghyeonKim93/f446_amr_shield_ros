#ifndef _DCMOTOR_H_
#define _DCMOTOR_H_

#include "mbed.h"
#include "math.h"
#include "platform/mbed_thread.h"

#include "../parameters.h"

#include "kalman_filter.h"

// in1  in2  pwm
//  0    0    x   brake
//  1    1    x   floating
//  1    0    1   right rotation
//  0    1    1   reverse rotation
//  1    0   PWM  right rot. pwm
//  0    1   PWM  reverse rot. pwm.
class DCMOTOR
{
public:
    DCMOTOR(PinName pin_pwm, PinName pin_in0, PinName pin_in1);

    void setPIDGains(float kp, float ki, float kd);
    void getPIDGains(float& kp, float& ki, float& kd);
 
    float getFilteredAngularVelocity();
    float getFilteredAngularAcceleration();

    void controlAngularVelocity(float w_desired);
    void updateAngularVelocity(float w_current, float dt);

// Motor hardware control functions
private:
    void setMotorBreak();
    void setMotorFloat();
    void setMotorRotate(float pwm_signal);

// Time related
private:
    void getTimeElapsed();

// Private members
private:
    // Motor PWM & direction signals
    PwmOut     pwm_out_;
    DigitalOut in0_;
    DigitalOut in1_;

    KalmanFilter kf_;

    // PID controller related
    // PID gains
    float kp_;
    float ki_;
    float kd_;

    // PID controller, left: 0 , right: 1
    float err_prev_;
    float err_accum_;

    float pwm_value_;
    float pwm_prev_;

    // Timer to know the elapsed time.
    Timer timer_;    
    float dt_;
};

#endif