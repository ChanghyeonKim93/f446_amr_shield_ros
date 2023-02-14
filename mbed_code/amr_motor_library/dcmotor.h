#ifndef _DCMOTOR_H_
#define _DCMOTOR_H_

#include "mbed.h"
#include "math.h"
#include "platform/mbed_thread.h"


// in1  in2  pwm
//  0    0    x   brake
//  1    1    x   floating
//  1    0    1   right rotation
//  0    1    1   reverse rotation
//  1    0   PWM  right rot. pwm
//  0    1   PWM  reverse rot. pwm.
class DCMOTORS 
{
public:
    DCMOTORS(PinName pin_left_pwm, PinName pin_right_pwm);
}
#endif