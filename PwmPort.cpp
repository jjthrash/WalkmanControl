//File PwmPort.cpp
#import "Arduino.h"
#import "wiring_private.h"

#import "PwmPort.h"

PwmPort::PwmPort(float freq):m_freq(freq)
{
  
}
const int TOP = 255;
  
void PwmPort::setup()
{
  // use timer2 B and OC2B (pin D) and phase correct pwm
  const int COUNT_PERIOD = 2 * TOP;
  int scale[] = {0,1,8,32,64,128,256,1024};
  const int SCALES = sizeof(scale)/sizeof(int); 
  const float CLOCK= 16e6;    // 16 mhz cpu clock
  // find presale factor based upon desired pwm frequency
  int i;
  for(i = 0; i < SCALES; i++)
  {
    if(scale[i] * m_freq * COUNT_PERIOD >= CLOCK)
    {
      i--;
      break;
    }
  }
  Serial.print("pwm freq = "); Serial.println(CLOCK / scale[i] / COUNT_PERIOD);   
  TCCR2B = i;      // set prescale selection
  sbi(TCCR2A,COM2B1); cbi(TCCR2A,COM2B0);                     
  cbi(TCCR2B,WGM22);  cbi(TCCR2A,WGM21); sbi(TCCR2A,WGM20);   // PWM,Phase Correct, TOP = OXFF, clear OC2B when up counting, set when down
  pinMode(3,OUTPUT); // OC2B direction
  // set duty cycle to 50%
  output(0.5);
}

void PwmPort::output(float value)
{
  OCR2B = TOP * value;
}

