//File ClosedLoopControl.cpp

#import "Arduino.h"
#import "ClosedLoopControl.h"

ClosedLoopControl::ClosedLoopControl(float propGain): m_pwmPort(15000), m_propGain(propGain),
                                                      m_integratorState(0.5)
{
  
}

void ClosedLoopControl::setup()
{
  m_pwmPort.setup();
}

void ClosedLoopControl::setup(float sampleRate, float gain)
{
  m_propGain = gain / sampleRate;
}

float ClosedLoopControl::state()
{
  return m_integratorState;
}

void ClosedLoopControl::update(float command, float feedback)
{
  const float MAX_BOUND = 0.99;
  const float MIN_BOUND = 0.01;
  
  float controlValue = m_propGain*(command - feedback);
  m_integratorState += controlValue;
  //Serial.print(controlValue); Serial.print(", ");
  if(m_integratorState > MAX_BOUND)
  {
    m_integratorState = MAX_BOUND;
  }
  else if(m_integratorState < MIN_BOUND)
  {
    m_integratorState = MIN_BOUND;
  }
  //Serial.println(m_integratorState);
  m_pwmPort.output(m_integratorState);
}

