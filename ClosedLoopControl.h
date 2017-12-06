// File: ClosedLoopControl.h

// pwm pin atmega 2560   OC3A to PE3 pin to board ardiono mega 2560  PWML pin 6.
#import "PwmPort.h"

class ClosedLoopControl
{
  public:
    ClosedLoopControl(float propGain);
    void setup();
    void setup(float updateRate,float gain);  // rate in seconds, gain in 1/sec
    void update(float command, float feedback);
    float state();
 private:
    float m_propGain;
    float m_integratorState;
    PwmPort m_pwmPort;
};

