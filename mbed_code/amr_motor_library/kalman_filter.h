#ifndef _KF_H_
#define _KF_H_
 
#include "mbed.h"
#include "math.h"
#include "platform/mbed_thread.h"

class KalmanFilter{
public:
    KalmanFilter(float sig_w, float sig_a, float sig_v) : sig_w_(sig_w), sig_a_(sig_a), sig_v_(sig_v)
    {
        P_[0][0] = 10; P_[0][1] = 0; P_[1][0] = 0; P_[1][1]=10;
        Q_[0][0] = sig_w_*sig_w_; Q_[1][1] = sig_a_*sig_a_;
        R_ = sig_v_*sig_v_;
        X_[0] = 0; X_[1] = 0;
    };
    void doFilter(float w_measure, float dt){
        float dtdt = dt*dt;
        // propagation (state)
        X_tmp_[0] = X_[0] + X_[1]*dt;
        X_tmp_[1] = X_[1];
        
        // propagation (covariance matrix)
        P_tmp_[0][0] = P_[0][0] + dt*(P_[1][0] + P_[0][1]) + dtdt*P_[1][1] + Q_[0][0];
        P_tmp_[0][1] = P_[0][1] + dt*P_[1][1];
        P_tmp_[1][0] = P_tmp_[0][1];
        P_tmp_[1][1] = P_[1][1] + Q_[1][1];
        
        
        // Correction by the measurement
        float scaler = 1.0f/(P_tmp_[0][0] + R_);
        K_[0] = P_tmp_[0][0]*scaler;
        K_[1] = P_tmp_[1][0]*scaler;
        
        X_[0] = X_tmp_[0] + K_[0]*(w_measure - X_tmp_[0]);
        X_[1] = X_tmp_[1] + K_[1]*(w_measure - X_tmp_[0]);
        
        P_[0][0] = (1-K_[0])*P_tmp_[0][0];
        P_[0][1] = (1-K_[0])*P_tmp_[0][1];
        P_[1][0] = -K_[1]*P_tmp_[0][0] + P_tmp_[1][0];
        P_[1][1] = -K_[1]*P_tmp_[0][1] + P_tmp_[1][1];              
    };
    
    float getFiltered_w(){ return X_[0];};
    float getFiltered_a(){ return X_[1];};
    
private:
    float sig_w_;
    float sig_a_;
    float sig_v_;
    float R_;
    float Q_[2][2];
    
private:
    float K_[2];

    float X_[2];
    float P_[2][2];
    
    float X_tmp_[2];
    float P_tmp_[2][2];
};

#endif