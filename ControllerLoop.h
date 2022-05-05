#pragma once

#include "mbed.h"
#include "EncoderCounter.h"
#include "LinearCharacteristics.h"
#include "ThreadFlag.h"
#include "PID_Cntrl.h"
#include "sensors_actuators.h"
#include "IIR_filter.h"
#include "GPA.h"
#include <cmath>



// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class ControllerLoop
{
public:
    ControllerLoop(sensors_actuators *,float Ts);
    virtual     ~ControllerLoop(void);
    void start_loop(void);
    void enable_vel_cntrl(void);
    void enable_bal_cntrl(void);
    void enable_wiggle(void);
    void reset_cntrl(void);
    void disable_all_cntrl(void);

    void reset_timer(void);

private:
    void loop(void);
    void sendSignal(void);
    float saturate (float,float,float);

    uint8_t m_printCnt;

    float m_phi_bd;

    float m_i_des;
    float m_M_des;
    float m_phi_bd_des;
    
    float m_km;
    float m_om;
    float m_V;

    float *m_K_mat;


    Thread m_thread;
    ThreadFlag m_threadFlag;

    Ticker m_ticker;
    Timer m_ti;

    PID_Cntrl m_flat_vel_cntrl;
    PID_Cntrl m_bal_vel_cntrl;

    float m_Ts;

    bool m_bal_cntrl_enabled;
    bool m_vel_cntrl_enabled;
    bool m_wiggle_enabled;

    sensors_actuators *m_sa;
    
};
