#include "dcmotor.h"

DCMOTOR::DCMOTOR(PinName pin_pwm, PinName pin_in0, PinName pin_in1)
: pwm_out_(pin_pwm), in0_(pin_in0), in1_(pin_in1)
{
    kp_ = 0.0f; ki_ = 0.0f; kd_ = 0.0f;

    err_prev_  = 0.0f; err_accum_ = 0.0f;

    pwm_prev_  = 0.0f; pwm_value_ = 0.0f;

    timer_.start();

// Initialize the motor PWM with zero for safety.
    pwm_out_ = 0.0f;
    in0_ = false;
    in1_ = false;
};

void DCMOTOR::setPIDGains(float kp, float ki, float kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
};

void DCMOTOR::getPIDGains(float& kp, float& ki, float& kd) {
    kp = kp_; ki = ki_; kd = kd_;
};


// in1  in2  pwm
//  0    0    x   brake
//  1    1    x   floating
//  1    0    1   right rotation
//  0    1    1   reverse rotation
//  1    0   PWM  right rot. pwm
//  0    1   PWM  reverse rot. pwm.
void DCMOTOR::setMotorBreak(){ in0_ = false; in1_ = false; };
void DCMOTOR::setMotorFloat(){ in0_ = true; in1_ = true; };
    
void DCMOTOR::setMotorRotate(float pwm_signal)
{
    float eps = 0.005; // dead-zone.
    if(pwm_signal >  1.0f) pwm_signal =  1.0f;
    if(pwm_signal < -1.0f) pwm_signal = -1.0f;
    if(pwm_signal > eps){ // Clock-wise Rotation (CW)
        in0_     = false; in1_ = true;
        pwm_out_ =  pwm_signal;
    }
    else if(pwm_signal < -eps){ // Counter Clock-wise Rotation (CCW)
        in0_     = true;  in1_ = false;
        pwm_out_ = -pwm_signal;
    }
    else setMotorFloat();    
};

void DCMOTOR::getTimeElapsed(){
    dt_        = (float)((uint32_t)timer_.elapsed_time().count())/1000000.0f;
    timer_.reset();

    if(dt_ > 0.1) dt_ = 0.1;
};

void DCMOTOR::controlAngularVelocity(float w_current, float w_desired) {
    // Get elapsed time from the last control
    getTimeElapsed();

    // PID control        
    float err_p = w_desired - w_current; // position error
    float err_d = err_p - err_prev_; // d error
    
    err_accum_ += err_p*dt_; // I error with clipping
    if(err_accum_ >  0.2) err_accum_ =  0.2;
    if(err_accum_ < -0.2) err_accum_ = -0.2;
    float u = 0.1*(kp_ * err_p + kd_ * err_d/dt_ + ki_ * err_accum_);
    
    pwm_value_ += u;
    float diff_pwm = pwm_value_ - pwm_prev_;
    if(diff_pwm > 0.5) pwm_value_ = pwm_prev_ + 0.5;
    if(diff_pwm < -0.5) pwm_value_ = pwm_prev_ - 0.5;
    
    this->setMotorRotate(pwm_value_);  
    
    err_prev_ = err_p;      
    pwm_prev_ = pwm_value_;
};

