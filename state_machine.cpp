#include "state_machine.h"
using namespace std;

// contructor for controller loop
state_machine::state_machine(sensors_actuators *sa, ControllerLoop *loop, float Ts) : thread(osPriorityNormal,4096)
{
    this->Ts = Ts;
    this->CS = INIT;
    this->m_sa = sa;
    this->m_loop = loop;
    ti.reset();
    ti.start();
    }

// decontructor for controller loop
state_machine::~state_machine() {}

// ----------------------------------------------------------------------------
void state_machine::loop(void){
    
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        // this statemachine is for later use, here, just test sensors
        switch(CS)
            {
            case INIT:
                if(m_sa->key_was_pressed && ti.read()>.5)
                    {
                    printf("switch to FLAT, rotate\r\n");
                    m_loop->start_loop();
                    m_sa->key_was_pressed = false;
                    ti.reset();
                    CS = FLAT;
                    }
                break;
            case FLAT:
                if(m_sa->key_was_pressed && ti.read()>.5)
                    {
                    printf("switch to BALANCE\r\n");
                    m_loop->enable_bal_cntrl();
                    m_sa->key_was_pressed = false;
                    CS = BALANCE;
                    ti.reset();
                    }
                break;
            case BALANCE:
                if(m_sa->key_was_pressed && ti.read()>.5)
                    {
                    printf("switch to INIT\r\n");
                    m_sa->key_was_pressed = false;
                    m_loop->disable_all_cntrl();
                    CS = INIT;
                    ti.reset();
                    }
                break;
            default:
                break;
            }   // end switch
        }// endof the main loop
}

void state_machine::sendSignal() {
    thread.flags_set(threadFlag);
}
void state_machine::start_loop(void)
{
    thread.start(callback(this, &state_machine::loop));
    ticker.attach(callback(this, &state_machine::sendSignal), Ts);
}
