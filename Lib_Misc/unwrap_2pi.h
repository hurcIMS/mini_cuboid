/*  
*/

using namespace std;

class unwrap_2pi
{
public:

    unwrap_2pi(void);
    
    float operator()(float in) {
        return doStep(in);
    }
    
    virtual     ~unwrap_2pi();
    
    void        reset(void);
    float       doStep(float inc);

private:

    long turns;
    float last_value;

};