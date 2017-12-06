

#include "Arduino.h"
#include "Task.h"
#include "AnalogAcquire.h"

class BlinkLed : public Task
{
  public:
    BlinkLed(const char *label, int led, float timeOut):m_led(led), Task(label,timeOut)
    {
       
    }
    void setup()
    {
      pinMode(m_led,OUTPUT);
    }
    void run(unsigned long time)
    {
      digitalWrite(m_led,!digitalRead(m_led));
      //Serial.println(taskName());
    }
    
  private:
    int m_led;
} blinkLed("Led",13,145), blinkLed1("Led1",13,235), blinkLed2("Led2",13,667);



void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  TaskSystem.addTask(AnalogAcquireTask);
  //TaskSystem.addTask(blinkLed);
  //TaskSystem.addTask(blinkLed1);
  //TaskSystem.addTask(blinkLed2);

 
}

void loop() 
{
  // put your main code here, to run repeatedly: 
  TaskSystem.execute();
  
}
