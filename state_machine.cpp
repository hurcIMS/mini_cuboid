#include "state_machine.h"
using namespace std;

// contructor for controller loop
state_machine::state_machine(sensors_actuators *sa, ControllerLoop *loop, float Ts) : m_thread(osPriorityNormal,4096)
{
    m_Ts = Ts;
    m_CS = INIT;
    m_sa = sa;
    m_loop = loop;
    m_ti.reset();
    m_ti.start();
    }

// decontructor for controller loop
state_machine::~state_machine() {}

// ----------------------------------------------------------------------------
void state_machine::loop(void){
    
    while(1)
        {
        ThisThread::flags_wait_any(m_threadFlag);
        // THE LOOP ------------------------------------------------------------
        // this statemachine is for later use, here, just test sensors
        switch(m_CS)
            {
            case INIT:
                if(m_sa->key_was_pressed && std::chrono::duration<float>(m_ti.elapsed_time()).count() > 0.5f)
                    {
                    printf("switch to FLAT, rotate\r\n");
                    m_loop->start_loop();
                    m_sa->key_was_pressed = false;
                    m_ti.reset();
                    m_CS = FLAT;
                    }
                break;

            case FLAT:
                //if(m_sa->key_was_pressed && std::chrono::duration<float>(m_ti.elapsed_time()).count() > .5f)
                if(std::sqrt(std::pow(m_sa->get_ax(),2)+std::pow(m_sa->get_ay(),2)) > 11.0 && std::chrono::duration<float>(m_ti.elapsed_time()).count() > 0.5f & fabs(m_sa->get_vphi_fw()) < 1.0f) 
                    {
                    printf("switch to BALANCE\r\n");
                    m_loop->enable_bal_cntrl();
                    m_sa->key_was_pressed = false;
                    m_CS = BALANCE;
                    m_ti.reset();
                    }
                break;

            case BALANCE:
                // if(m_sa->key_was_pressed && std::chrono::duration<float>(m_ti.elapsed_time()).count() > .5f)
                if(std::sqrt(std::pow(m_sa->get_ax(),2)+std::pow(m_sa->get_ay(),2)) > 11.0 && std::chrono::duration<float>(m_ti.elapsed_time()).count() > 1.0f)
                    {
                    printf("switch to WIGGLE\r\n");
                    m_sa->key_was_pressed = false;
                    m_loop->enable_wiggle();
                    m_CS = WIGGLE;
                    m_ti.reset();
                    m_loop->reset_timer();
                    }
                break;

            case WIGGLE:
                if(std::sqrt(std::pow(m_sa->get_ax(),2)+std::pow(m_sa->get_ay(),2)) > 11.0 && std::chrono::duration<float>(m_ti.elapsed_time()).count() > 1.0f)
                    {
                    printf("switch to FLAT\r\n");
                    m_sa->key_was_pressed = false;
                    m_loop->disable_all_cntrl();
                    m_CS = FLAT;
                    m_ti.reset();
                    }
                break;
            default:
                break;
            }   // end switch
        }// endof the main loop
}

void state_machine::sendSignal() {
    m_thread.flags_set(m_threadFlag);
}

void state_machine::start_loop(void)
{
    m_thread.start(callback(this, &state_machine::loop));
    m_ticker.attach(callback(this, &state_machine::sendSignal), std::chrono::milliseconds(static_cast<long long>(1000*m_Ts)));
}
