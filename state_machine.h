#include "mbed.h"
#include "sensors_actuators.h"
#include "ControllerLoop.h"

enum States{
    INIT = 1,
    FLAT,
    BALANCE,
    WIGGLE
};



// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class state_machine
{
public:
    state_machine(sensors_actuators *,ControllerLoop *,float Ts);
    virtual     ~state_machine(void);
    void start_loop(void);

private:
    void loop(void);
    void sendSignal(void);

    uint8_t m_CS;             // the current state
    Thread m_thread;
    Ticker m_ticker;
    ThreadFlag m_threadFlag;
    Timer m_ti;
    float m_Ts;
    sensors_actuators *m_sa;
    ControllerLoop *m_loop;
};
