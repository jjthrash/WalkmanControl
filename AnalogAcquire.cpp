// File:  AnalogAcquire.cpp
#import "Arduino.h"
#import "AnalogAcquire.h"
#import "wiring_private.h"
//#import "assert.h"

#define LED 13

void pulseDigitalOutput(int pin,  unsigned long int time = 0)
{
  
  digitalWrite(pin,HIGH);
  if(time > 0)
  {
    for(unsigned long int i = 0; i < time; i++)
    {
      for(unsigned long int j = 0; j < 100; j++)
      {
        volatile int k = 1;
      }
    }
  }
  digitalWrite(pin,LOW);
}

void toggleDigitalOutput(int pin)
{
  digitalWrite(pin,!(digitalRead(pin)));
}

#if(0)
unsigned int sbi(int port, int bit)
{
  port |= 1 << bit;
  return port;
 
}
unsigned int cbi(int port, int bit)
{
  port &= ~(1 << bit);
  return port;
}
#endif

#define PRINT_IF_TRUE(logicalExpression) if(logicalExpression) Serial.println(__LINE__);

BufferedAnalogAcquireTask::BufferedAnalogAcquireTask(int input, int bufferSize, float _sampleRate,int minSamples)
              :Task("BufferAnalogAcquireTask",1000),
               m_state(STATE_STOPPED),
               m_input(input),m_sampleRate(_sampleRate),m_signal(false),
               m_buffer1(bufferSize), m_buffer2(bufferSize),
               m_loopControl(0.001),
               m_cycleState(CYCLE_STATE_START),m_adcState(ADC_STATE_ACQUIRE),
               m_overRun(false),
               m_periodsState(PERIODS_STATE_START), m_periods(0), m_samples(minSamples)
              
{
  m_pCurrentBuffer = &m_buffer1;
  m_pWorkingBuffer = &m_buffer2;
  // set up dc blocking filter
  // let time constant be Tc seconds
  // in samples tme consant d =  Tc * fs
  const float Tc = 0.1; // seconds    
  float d = Tc * m_sampleRate;
  float x = exp(-1/d);
  float a0 = (1+x)/2;
  float a1 = -a0;
  float b1 = x;
  m_dcBlockingFilter.setCoefficients(a0,a1,b1);
  
}
const int ARRAY_SIZE = 5;
static const float prescaleFactorArray[ARRAY_SIZE] = {1,8,4,256,1024};

void BufferedAnalogAcquireTask::setup()
{
   pinMode(LED,OUTPUT);
   pinMode(12,OUTPUT);
   pinMode(11,OUTPUT);
   //SREG = cbi(SREG,7);

   // determine optimum pre scaling and compare register values from desired sample rate
   // table of prescale factors
   const float CPU_CLOCK = 16E6;
   const unsigned int MAX_COMPARE_REG = 0xFFFF;
   float cpuClocksPerSamplePeriod = CPU_CLOCK / m_sampleRate;
   float timerClocks;
   
   unsigned int arrayIndex = 0;
       
   do
   {
      timerClocks = cpuClocksPerSamplePeriod / prescaleFactorArray[arrayIndex];
      Serial.print(timerClocks);Serial.print(",");
      arrayIndex++;
      Serial.println(arrayIndex);
   } while(timerClocks > MAX_COMPARE_REG &&  arrayIndex < ARRAY_SIZE );
   //assert(arrayIndex <= ARRAY_SIZE);
   unsigned int compareReg = (timerClocks-1) + 0.5;
   m_actualSampleRate = CPU_CLOCK / prescaleFactorArray[arrayIndex-1] / (compareReg + 1);
   m_loopControl.setup();
   m_loopControl.setup(m_actualSampleRate/m_buffer1.bufferSize(),0.001);
   Serial.print("setup ");
   Serial.print(compareReg); Serial.print(",");
   Serial.println(m_actualSampleRate);
   Serial.println(TCCR1B);
   TCCR1B &= ~7;
   TCCR1B |= arrayIndex;
   Serial.println(TCCR1B,HEX);
 
   // set mode (timer1 CTC)
   TCCR1A = cbi(TCCR1A,WGM10);
   TCCR1A = cbi(TCCR1A,WGM11);
   TCCR1B = sbi(TCCR1B,WGM12);
   TCCR1B = cbi(TCCR1B,WGM13);

   OCR1A = compareReg;
   OCR1B = compareReg;

   // select analog input
   Serial.println(m_input);
   ADMUX = m_input;
   // right adjust result
   ADMUX = cbi(ADMUX,ADLAR);
   // internal VCC ref
   ADMUX = sbi(ADMUX,REFS0);
   ADMUX = cbi(ADMUX,REFS1);
   Serial.println(ADMUX,HEX);

  
  // set adc prescalar to 16 for 1 mhz clock
  ADCSRA = sbi(ADCSRA,ADPS2);
  ADCSRA = cbi(ADCSRA,ADPS1);
  ADCSRA = cbi(ADCSRA,ADPS0);
  
  // enable adc auto trigger and interrupt
     // enable auto trigger
   ADCSRA = sbi(ADCSRA,ADATE);
   
   // set up adc for auto trigger on timer 1 compB match (101)
   ADCSRB = sbi(ADCSRB,ADTS2);
   ADCSRB = cbi(ADCSRB,ADTS1);
   ADCSRB = sbi(ADCSRB,ADTS0);

   // setup timer 2 for pwm
   
   // enable adc
   ADCSRA = sbi(ADCSRA,ADEN);
   // enable end of conversion interrupt
   ADCSRA = sbi(ADCSRA,ADIE);
   // start adc
   ADCSRA = sbi(ADCSRA,ADSC);

   //digitalWrite(LED,HIGH);
   // enable interrupts
   //SREG = sbi(SREG,7);
   //m_signal = true;
}
    
void BufferedAnalogAcquireTask::run(unsigned long time)
{
        int adcValue = -1;
        float frequency;
        toggleDigitalOutput(LED);
        switch(m_state)
        {
          case STATE_STOPPED:
            // to setup adc mux for input channel;
            //adcValue = analogRead(m_input);
            m_state = STATE_RUNNING; 
          break;
          case STATE_RUNNING:
            if(m_cyclePeriod == 0.0 || m_periods == 0.0)
            {
              frequency = 0;
            }
            else
            {
              frequency = m_actualSampleRate / (m_cyclePeriod/m_periods);
            }
            //Serial.print(averageValue);Serial.print(", ");
            //Serial.print(maxValue);Serial.print(", ");
            Serial.print(m_periods);Serial.print(", ");
            Serial.print(m_cyclePeriod);Serial.print(", ");
            Serial.print(frequency);Serial.print(", ");
            Serial.println(m_loopControl.state());
            //m_loopControl.update(400.0,frequency);
      

          break;
        }
        
 
}
float interpolate(int before, int after)
{
     float retVal = -before;
     retVal = retVal / (after - before);
     return retVal;
}

void BufferedAnalogAcquireTask::clockTick(unsigned long time)
{
    // called once on each call to loop() in main
    // check for full buffer
      
    if(m_signal)
    {
      digitalWrite(12,HIGH);
      int size = m_pWorkingBuffer->bufferSize();

#if(0)

      // a buffer has been filled,   
      // find average of last working buffer
      long int sum = 0;
      //averageValue = 0;
      int _maxValue = -10000;
      int _minValue = 10000;

      for(int i = 0; i < size; i++)
      {
 
        float dataValue = m_dcBlockingFilter.update(m_pWorkingBuffer->dataValue(i));

        sum += dataValue;
                 
        if(dataValue > _maxValue)
        {
          _maxValue = dataValue;
        }
        else if(dataValue < _minValue)
        {
          _minValue = dataValue;
        }
        
      }
      
      averageValue = sum;
      averageValue /= size;

      maxValue = _maxValue;
      minValue = _minValue;
      /*
      Serial.print(averageValue);Serial.print(", ");
      Serial.print(maxValue);Serial.print(", ");
      Serial.println(minValue);
      
      */
#endif
      m_cycleState = CYCLE_STATE_START;
      int acSignal, prevAcSignal;
      const int HYSTERISIS = 0;
      float startSample;
      m_cyclePeriod = 0;
      m_periods = 0;
      
          

#if(1)
      for(int i = 0; i < size; i++)
      {
        if(m_cycleState == CYCLE_STATE_END)
        {
          break;  // out of loop
        }
        prevAcSignal = acSignal;
        acSignal = m_dcBlockingFilter.update(m_pWorkingBuffer->dataValue(i)); //- averageValue;
        switch(m_cycleState)
        {
          case CYCLE_STATE_START:
            if (acSignal > HYSTERISIS)
            {
              m_cycleState = CYCLE_POS;
            }
            else if (acSignal < HYSTERISIS)
            {
              m_cycleState = CYCLE_NEG;
            }
          break;
          case CYCLE_NEG:
            // look for first transition to positive
            if(acSignal > HYSTERISIS)
            {
              // signal has passed through zero, find actual zeroCrossing
              startSample = i + interpolate(prevAcSignal,acSignal);
                
              // beginning first positive half cycle
              m_cycleState = CYCLE_POS_FIRST_HALF;
            }
          break;
          case CYCLE_POS_FIRST_HALF:
            if(acSignal < HYSTERISIS)
            {
              // beginning the next half cycle
              m_cycleState = CYCLE_NEG_SECOND_HALF;
            }
          break;
          case CYCLE_NEG_SECOND_HALF:
            if(acSignal > HYSTERISIS)
            {
              m_cyclePeriod =  i + interpolate(prevAcSignal,acSignal) - startSample;
              m_periods++;
              if(m_cyclePeriod > m_samples)
              {
                m_cycleState = CYCLE_STATE_END;
              }
              else
              {
                // measure next full cycle period
                m_cycleState = CYCLE_POS_FIRST_HALF;
              }
            }
          break;
          case CYCLE_POS:
            // look for first transition to negative
            if(acSignal < HYSTERISIS)
            {
              // beginning a negative half cycle
              m_cycleState = CYCLE_NEG_FIRST_HALF;
              startSample = i + interpolate(prevAcSignal,acSignal);
            }
          break;
          case CYCLE_NEG_FIRST_HALF:
          {
            if(acSignal > HYSTERISIS)
            {
              // beginning the next half cycle
              m_cycleState = CYCLE_POS_SECOND_HALF;
            }
          }
          break;
          case CYCLE_POS_SECOND_HALF:
            if(acSignal < HYSTERISIS)
            {
              // end of a cycle
              m_cyclePeriod = i + interpolate(prevAcSignal,acSignal) - startSample;
              m_periods++;
              if(m_cyclePeriod > m_samples)
              {
                m_cycleState = CYCLE_STATE_END;
              }
              else
              {
                // measure next full cycle period
                m_cycleState = CYCLE_NEG_FIRST_HALF;
              }

            }
          break;
        }
      }
#endif
      float frequency;      
            if(m_cyclePeriod == 0.0 || m_periods == 0.0)
            {
              frequency = 0;
            }
            else
            {
              frequency = m_actualSampleRate / (m_cyclePeriod/m_periods);
            }
            m_loopControl.update(800,frequency);
 
      m_signal = false;
      digitalWrite(12,LOW);

    }
    Task::clockTick(time);
}

//static int  interrupts = 0;
void BufferedAnalogAcquireTask::switchBuffers()
{
   if(m_pCurrentBuffer = &m_buffer1)
   {
      m_pCurrentBuffer = &m_buffer2;
   }
   else
   {
      m_pCurrentBuffer = &m_buffer1;
   }         
}

void BufferedAnalogAcquireTask::InterruptHandler()
{
  // stuffs most recently acquired sample into buffer.
  
  unsigned int value = ADCL + (ADCH << 8);     // 10 bit adc sample
  //unsigned char value = ADCH;   // 8 bit adc sample
  bool bufferFull;
  //pulseDigitalOutput(11,10);

  switch(m_adcState)
  {
    case ADC_STATE_ACQUIRE:
     bufferFull = m_pCurrentBuffer->AddData(value);
      if(bufferFull)
      {
          //pulseDigitalOutput(12,50);
          
        // check to see if still processing data from new buffer

        if(m_signal)
        {
          // need to wait for processing on working buffer data to end
          m_adcState = ADC_STATE_WAITING;
          //pulseDigitalOutput(12,100);
        }
        else
        {
          // current buffer new working buffer
          m_pWorkingBuffer = m_pCurrentBuffer;
          switchBuffers();
          m_signal = true;    //signal background to start processing working buffer
          //pulseDigitalOutput(12,3);
        }
      }
     break;
     case ADC_STATE_WAITING:
        
        //pulseDigitalOutput(12,5);
        if(!m_signal)
        {
          m_pWorkingBuffer = m_pCurrentBuffer;
          switchBuffers();
          m_adcState = ADC_STATE_ACQUIRE;
          m_signal = true;
          //pulseDigitalOutput(12,4);
        }
      break;
  }
 }

void BufferedAnalogAcquireTask::Start()
{
  // setup A/D
   
  
}

void BufferedAnalogAcquireTask::Stop()
{
  
}


//-----------------------------------------
Buffer::Buffer(int size):m_size(size),m_filled(false)
{
  m_pBuffer = new unsigned int[m_size];
  m_pBufferAdd = m_pBuffer;
}

bool Buffer::AddData(unsigned int data)
{
  *m_pBufferAdd++ = data;
  m_filled = (m_pBufferAdd - m_pBuffer) == m_size;
  if(m_filled)
  {
    m_pBufferAdd = m_pBuffer;
  }
  return m_filled;
}

bool &Buffer::filled()
{
  return m_filled;
}

unsigned int Buffer::dataValue(int index)
{
  //                                        assert(index < m_size && m_filled);
  return m_pBuffer[index];
}

//-----------------------
ISR(ADC_vect)
{
  
  //toggleDigitalOutput(12);
  //digitalWrite(12,HIGH);
  // clear the counter 1 output compare match flag
  TIFR1 = sbi(TIFR1,OCF1B);
  //pulseDigitalOutput(12);
  AnalogAcquireTask.InterruptHandler();
  //digitalWrite(12,LOW);

}

//---------------------- don't know how else do it with ISR
BufferedAnalogAcquireTask AnalogAcquireTask(3,300,10000,150);




