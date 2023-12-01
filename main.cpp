/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "platform/mbed_thread.h"
#include <math.h>

#define BLINKING_RATE_MS1                                                    10
#define VCC (3.3)

#define delta                                                                0.01
#define KP                                                                   0.3
#define KI                                                                   1.0
#define KD                                                                   0.03
#define PV1                                                                  0.3 //速度指定
#define PV2                                                                  0.7 //速度指定
#define PPR                                                                  10
#define G                                                                    1/38.2
#define r                                                                    10

class Counter {
public:
    Counter(PinName pin) : _interrupt(A1) {        // create the InterruptIn on the pin specified to Counter
        _interrupt.rise(callback(this, &Counter::increment)); // attach increment function of this counter instance
    }

    void increment() {
        _count++;
    }

    int read() {
        _count_pre = _count;
        _count = 0;
        return _count_pre;
    }

private:
    InterruptIn _interrupt;
    volatile int _count;
    volatile int _count_pre;
};

Counter counter_R(D13);     //?
Counter counter_L(D13);     //?

DigitalOut Digt1_R(D2);
PwmOut mypwm_R(D1);

DigitalOut Digt1_L(D5);
PwmOut mypwm_L(D9);

AnalogIn Ain1(A1);
AnalogIn Ain2(A2);
AnalogIn Ain3(A0);

float inverse_function(float vero){
    float D_pwm = 0;

    //printf("D_pwm = %lf\n",D_pwm);
    return D_pwm;
}

float v_R(){

    float p_R     = counter_R.read();
    float omega_R = (2 * 3.14 * p_R * G) / (PPR * delta);
    float v_R     = r * omega_R;
    //printf("v_R = %lf\n",v_R);

    return v_R;
}

float v_L(){

    float p_L     = counter_L.read();
    float omega_L = (2 * 3.14 * p_L * G) / (PPR * delta);
    float v_L     = r * omega_L;
    //printf("v_L = %lf\n",v_L);

    return v_L;
}

float max_min_control(float cont_val){

    if(cont_val>1){
        cont_val = 1;
    }else if(cont_val<0){
        cont_val = 0.1;
    }

    //printf("cont_val = %lf\n",cont_val);
    return cont_val;
}

float PID_R(float PV){
    
    float p = 0;
    float i = 0;
    float d = 0;
    float value[2] = {};
    float integ = 0;

    value[0] = value[1];
    value[1] = PV - v_R();          
    integ   += (value[0]+value[1]) / 2 * delta;

    p = KP * value[1];
    i = KI * integ;
    d = KD * (value[1]-value[0]) / delta;

    //printf("R_pid = %lf\n",p+i+d);

    return max_min_control( inverse_function(p+i+d) );
}

float PID_L(float PV){
    
    float p = 0;
    float i = 0;
    float d = 0;
    float value[2] = {};
    float integ = 0;

    value[0] = value[1];
    value[1] = PV - v_L();          
    integ   += (value[0]+value[1]) / 2 * delta;

    p = KP * value[1];
    i = KI * integ;
    d = KD * (value[1]-value[0]) / delta;

    return max_min_control( inverse_function(p+i+d) );
}

void L_R_control(float perc_R,float perc_L){

    if( (perc_R<20) && (perc_L<20) ){

        mypwm_R.write(0.7);
        mypwm_L.write(0.7);
    }
    else if( perc_R < perc_L ){

        mypwm_R.write( PID_R(PV2) );
        mypwm_L.write( PID_L(PV1) );
    }
    else if( perc_R > perc_L ){

        mypwm_R.write( PID_R(PV1) );
        mypwm_L.write( PID_L(PV2) );
    }
    else{

        mypwm_R.write(0.1);
        mypwm_L.write(0.1);
    }
}

int main()
{
    mypwm_R.period(0.01f);
    Digt1_R = 0;

    mypwm_L.period(0.01f);
    Digt1_L = 0;

    while(1){

        float norm_F = Ain3.read();
        float perc_F = norm_F * 100;
        float volt_F = norm_F * VCC;

        float norm_R = Ain1.read();
        float perc_R = norm_R * 100;
        float volt_R = norm_R * VCC;

        float norm_L = Ain2.read();
        float perc_L = norm_L * 100;
        float volt_L = norm_L * VCC;

        if(perc_F>=80){
            L_R_control(perc_R,perc_L);
        }
        else{
            mypwm_R.write(0.1);
            mypwm_L.write(0.1);
        }

        //printf("Analog_R = %3.1f, %3.2f, %3.2f\n",perc_R,norm_R,volt_R);
        //printf("Analog_L = %3.1f, %3.2f, %3.2f\n",perc_L,norm_L,volt_L);
        //printf("control_R = %3.1f,control_L = %3.1f\n",cont_val_R,cont_val_L);
        thread_sleep_for(BLINKING_RATE_MS1);
    }
}
