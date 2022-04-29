/*  
*/

#include "unwrap_2pi.h"
#define   pi 3.141592653589793
using namespace std;

unwrap_2pi::unwrap_2pi(void)
{   
    last_value = 0.0;
    turns = 0;
}

unwrap_2pi::~unwrap_2pi() {}

void unwrap_2pi::reset(void)
{
    last_value = 0.0;
    turns = 0;
}

float unwrap_2pi::doStep(float in)
{
    float temp = in + 2*pi*(float)turns;
    if((temp - last_value) > pi){
        temp -= 2*pi;
        turns--;
        }
    else if((temp - last_value) < -pi){
        temp += 2*pi;
        turns++;
        }
    last_value = temp;
    return (temp);
}