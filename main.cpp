///本番PID

#include "mbed.h"
#include "platform/mbed_thread.h"

#define BLINKING_RATE_MS1                                                    10
#define VCC (3.3)

#define delta                                                                0.01
#define KP                                                                   0.3
#define KI                                                                   0.1
#define KD                                                                   0.03
#define PV1                                                                  0.3 //速度指定
#define PV2                                                                  0.7 //速度指定
#define PPR                                                                  20
#define G                                                                    1/38.2
#define r                                                                    27.5

class Counter_R {
public:
    Counter_R(PinName pin) : _interrupt(A6) {        // create the InterruptIn on the pin specified to Counter
        _interrupt.rise(callback(this, &Counter_R::increment)); // attach increment function of this counter instance
    }

    void increment() {
        _count_R++;
    }

    float read() {
        _count_pre_R = _count_R;
        _count_R = 0;
        return _count_pre_R;
    }

private:
    InterruptIn _interrupt;
    volatile float _count_pre_R;
    volatile float _count_R;
};

class Counter_L {
public:
    Counter_L(PinName pin) : _interrupt(A4) {        // create the InterruptIn on the pin specified to Counter
        _interrupt.rise(callback(this, &Counter_L::increment)); // attach increment function of this counter instance
    }

    void increment() {
        _count_L++;
    }

    float read() {
        _count_pre_L = _count_L;
        _count_L = 0;
        return _count_pre_L;
    }

private:
    InterruptIn _interrupt;
    volatile float _count_pre_L;
    volatile float _count_L;
};

Counter_R counter_R(D13);
Counter_L counter_L(D13);

DigitalOut Digt1_R(D2);
PwmOut mypwm_R(D1);

DigitalOut Digt1_L(D5);
PwmOut mypwm_L(D9);

AnalogIn Ain1(A1);
AnalogIn Ain2(A2);
AnalogIn Ain3(A0);

float value_R[2] = {};
float integ_R = 0;
float value_L[2] = {};
float integ_L = 0;

float inverse_function_R(float vero){
    float D_pwm = 0;

    D_pwm = ( vero + 6.937) / 1175.8;

    //printf("D_pwm = %lf\n",D_pwm);
    return D_pwm;
}

float inverse_function_L(float vero){
    float D_pwm = 0;

    D_pwm = ( vero - 11.855) / 1026.1;

    //printf("D_pwm = %lf\n",D_pwm);
    return D_pwm;
}

float v_R(){

    //float p_R     = counter_R.read();
    float omega_R = (2 * 3.14 * counter_R.read() * G) / (PPR * delta);
    float v_R     = r * omega_R;
    printf("v_R = %lf\n",v_R);

    return v_R;
}

float v_L(){

    //float p_L     = counter_L.read();
    float omega_L = (2 * 3.14 * counter_L.read() * G) / (PPR * delta);
    float v_L     = r * omega_L;
    printf("v_L = %lf\n",v_L);

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
    //float d = 0;

    value_R[0] = value_R[1];
    value_R[1] = PV - v_R();          
    integ_R   += (value_R[0]+value_R[1]) / 2 * delta;

    p = KP * value_R[1];
    i = KI * integ_R;
    //d = KD * (value_R[1]-value_R[0]) / delta;

    //printf("R_pid = %lf\n",p+i+d);

    return max_min_control( inverse_function_R(p+i) );
}

float PID_L(float PV){
    
    float p = 0;
    float i = 0;
    //float d = 0;

    value_L[0] = value_L[1];
    value_L[1] = PV - v_L();          
    integ_L   += (value_L[0]+value_L[1]) / 2 * delta;

    p = KP * value_L[1];
    i = KI * integ_L;
    //d = KD * (value_L[1]-value_L[0]) / delta;

    return max_min_control( inverse_function_L(p+i) );
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
            mypwm_R.write(inverse_function_R(15));
            mypwm_L.write(inverse_function_L(15));
        }

        //printf("Analog_R = %3.1f, %3.2f, %3.2f\n",perc_R,norm_R,volt_R);
        //printf("Analog_L = %3.1f, %3.2f, %3.2f\n",perc_L,norm_L,volt_L);
        //printf("control_R = %3.1f,control_L = %3.1f\n",cont_val_R,cont_val_L);
        thread_sleep_for(BLINKING_RATE_MS1);
    }
}
