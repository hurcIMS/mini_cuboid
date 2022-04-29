#include "ControllerLoop.h"
#include "GPA.h"
#define PI 3.1415927
using namespace std;

extern GPA myGPA;      

// contructor for controller loop
ControllerLoop::ControllerLoop(sensors_actuators *sa, float Ts) : thread(osPriorityHigh,4096)
{
    this->Ts = Ts;
    this->m_sa = sa;
    bal_cntrl_enabled = false;
    vel_cntrl_enabled = false;
    //flat_vel_cntrl.setup(...);
    //bal_vel_cntrl.setup(...);
    m_sa->disable_escon();
    ti.reset();
    ti.start();
    }

// decontructor for controller loop
ControllerLoop::~ControllerLoop() {}

// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void ControllerLoop::loop(void){
    float i_des = 0;
    uint8_t k = 0;
    float km = 36.9e-3;
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        m_sa->read_sensors_calc_estimates();       // first read all sensors, calculate mtor speed
        //i_des = myGPA.update(i_des,m_sa->get_vphi_fw());
        float phi_bd = m_sa->get_phi_bd();            // see below, not implemented yet
        float Kmat[2] = {-1.4073,   -0.0875};
        if(bal_cntrl_enabled)
            {
                float M_des = -(Kmat[0]*phi_bd + Kmat[1] * m_sa->get_gz());
                i_des = saturate(M_des/km,-13,13); 
                m_sa->enable_escon();      
            }
        else
            {
                i_des = 0.0;
                m_sa->disable_escon();      
                if(++k == 0)          
                    printf("ax: %f ay: %f gz: %f phi_fw :%f phi_bd :%f\r\n",m_sa->get_ax(),m_sa->get_ay(),m_sa->get_gz(),m_sa->get_phi_fw(),m_sa->get_phi_bd());
            }
        
        // -------------------------------------------------------------
        //m_sa->enable_escon();
        m_sa->write_current(i_des);                   // write to motor 0 
        // handle enable
        }// endof the main loop
}

void ControllerLoop::sendSignal() {
    thread.flags_set(threadFlag);
}
void ControllerLoop::start_loop(void)
{
    thread.start(callback(this, &ControllerLoop::loop));
    ticker.attach(callback(this, &ControllerLoop::sendSignal), Ts);
}

void ControllerLoop::enable_vel_cntrl(void)
{
    vel_cntrl_enabled = true;
}
void ControllerLoop::enable_bal_cntrl(void)
{
    bal_cntrl_enabled = true;
}
void ControllerLoop::reset_cntrl(void)
{

}
void ControllerLoop::disable_all_cntrl()
{
    bal_cntrl_enabled = false;
    vel_cntrl_enabled = false;
}
float ControllerLoop::saturate(float x,float lo,float up)
{
    if(x < lo)
        return lo;
    else if(x > up)
        return up;
    return x;
}
