#include "ControllerLoop.h"
#include "GPA.h"
#include "math.h"
#include "mbed.h"
#include "sensors_actuators.h"
#include "state_machine.h"
#include <stdint.h>

#include "minicube_parametermap.h"

static BufferedSerial serial_port(USBTX, USBRX);

float Ts = 0.002f;                // sampling time, typically approx 1/500

GPA myGPA(.7, 250, 30, 4, 4, Ts); // para for plant

//******************************************************************************
//---------- main loop -------------
//******************************************************************************

int main() {

    // --------- mini cuboid,
    sensors_actuators hardware(Ts); // in this class all the physical ios are handled
    ControllerLoop loop(&hardware, Ts); // this is for the main controller loop
    state_machine sm(&hardware, &loop, 0.01);
    ThisThread::sleep_for(200ms);
    // ----------------------------------
        
    loop.start_loop();
    ThisThread::sleep_for(20ms);  
    sm.start_loop();

    while (1)
        ThisThread::sleep_for(10h);

} // END OF main
