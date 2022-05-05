#include "ControllerLoop.h"

#define PI 3.1415927
using namespace std;

extern GPA myGPA;      

// contructor for controller loop
ControllerLoop::ControllerLoop(sensors_actuators *sa, float Ts) : m_thread(osPriorityHigh,4096), m_Ts(Ts), m_sa(sa), m_bal_cntrl_enabled(false), m_vel_cntrl_enabled(false)
{
    m_printCnt = 0;

    m_phi_bd = 0.0;

    m_i_des = 0.0;
    m_phi_bd_des = 0.0;

    // Controller parameter
    m_km = 36.9e-3;
    m_om = 6.5973; //2*PI/((2*60)/126); // 126 BPM
    m_V = -0.8642; // static prefilter

    m_K_mat = new float[2];    
    m_K_mat[0] = -1.4073;
    m_K_mat[1] = -0.0875;

    m_bal_vel_cntrl.setCoefficients(-3.69e-05, -1.84e-05, 0.0f, 1.0f, m_Ts, -0.037, 0.037);  // based on torque, max. 1 Amp
    
    //flat_vel_cntrl.setup(...);
    //bal_vel_cntrl.setup(...);
    m_sa->disable_escon();
    
    m_ti.reset();
    m_ti.start();
    }

// decontructor for controller loop
ControllerLoop::~ControllerLoop() {}

// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void ControllerLoop::loop(void){    
    
    while(1)
        {
        ThisThread::flags_wait_any(m_threadFlag);
        // THE LOOP ------------------------------------------------------------
        float tim = m_ti.read();//std::chrono::duration<float>(m_ti.elapsed_time()).count();
        
        m_sa->read_sensors_calc_estimates();       // first read all sensors, calculate motor speed
        
        m_phi_bd_des = 0.0;
        
        //i_des = myGPA.update(i_des,m_sa->get_vphi_fw());
        
        m_phi_bd = m_sa->get_phi_bd();            // see below, not implemented yet
        
        
        if(m_bal_cntrl_enabled)
        {
                if(m_wiggle_enabled)
                    m_phi_bd_des += .1 * sinf(m_om*tim);
                
                m_M_des = m_V*m_phi_bd_des - (m_K_mat[0]*m_phi_bd + m_K_mat[1]*m_sa->get_gz());
                m_M_des += m_bal_vel_cntrl(-m_sa->get_vphi_fw());    // the velocity cntrl. is based on torque input!                
                m_i_des = saturate(m_M_des/m_km, -13, 13);    // need at least 9 Amps to lift up!
                
                m_sa->enable_escon();
                
        }
        else
        {
                m_i_des = 0.0;

                m_sa->disable_escon();      
                if(++m_printCnt >= 10)          
                {
                    //printf("ax: %f ay: %f gz: %f phi_fw :%f phi_bd :%f\r\n",m_sa->get_ax(),m_sa->get_ay(),m_sa->get_gz(),m_sa->get_phi_fw(),m_sa->get_phi_bd());
                    //printf("%f %f %f %f %f\r\n",ti.read(),m_sa->get_ax(),m_sa->get_ay(),m_sa->get_gz(),m_sa->get_phi_bd());
                    //printf("%f %f %f %f\r\n",tim,m_sa->get_vphi_fw(),m_sa->get_gz(),m_sa->get_phi_fw());
                    //printf("%f %f %f %f\r\n",ti.read(),m_sa->get_vphi_fw(),m_sa->get_gz(),m_sa->get_phi_fw());
                    //printf("%f\r\n",std::sqrt(std::pow(m_sa->get_ax(),2)+std::pow(m_sa->get_ay(),2)));
                    m_printCnt = 0;
                }
        }
        
        // -------------------------------------------------------------
        //m_sa->enable_escon();
        m_sa->write_current(m_i_des);                   // write to motor 0 
        // handle enable
        }// endof the main loop
}

void ControllerLoop::sendSignal() {
    m_thread.flags_set(m_threadFlag);
}

void ControllerLoop::start_loop(void)
{
    m_thread.start(callback(this, &ControllerLoop::loop));
    m_ticker.attach(callback(this, &ControllerLoop::sendSignal), std::chrono::milliseconds(static_cast<long long>(1000*m_Ts)));
}

void ControllerLoop::enable_vel_cntrl(void)
{
    m_vel_cntrl_enabled = true;
}

void ControllerLoop::enable_bal_cntrl(void)
{
    m_bal_cntrl_enabled = true;
}

void ControllerLoop::enable_wiggle(void)
{
    m_wiggle_enabled = true;
}

void ControllerLoop::reset_timer(void)
{
    m_ti.stop();
    m_ti.reset();
    m_ti.start();
}

void ControllerLoop::reset_cntrl(void)
{

}

void ControllerLoop::disable_all_cntrl()
{
    m_bal_cntrl_enabled = false;
    m_vel_cntrl_enabled = false;
    m_wiggle_enabled = false;
}

float ControllerLoop::saturate(float x,float lo,float up)
{
    if(x < lo)
        return lo;
    else if(x > up)
        return up;
    return x;
}
