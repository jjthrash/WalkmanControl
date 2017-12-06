// File: AnalogAcquire.h


#ifndef ANALOG_ACQUIRE_H
#define ANALOG_ACQUIRE_H
#import "Task.h"
#import "ClosedLoopControl.h"
#import "Filters.h"

//---------------------------------------
class Buffer
{
  public:
    Buffer(int size);
    bool AddData(unsigned int data);
    int bufferSize(){return m_size;}
    unsigned int dataValue(int index);
    bool &filled();
  private:
    int m_size;
    unsigned int *m_pBuffer, *m_pBufferAdd;
    bool m_filled;
};
//-------------------------------------------------------
class BufferedAnalogAcquireTask : public Task
{
  public:
    BufferedAnalogAcquireTask(int inputChan, int bufferSize, float sampleRate, int minSamples);
    void InterruptHandler();

    void Start();
    void Stop();
    void setup();
    void run(unsigned long time);

   private:
     void clockTick(unsigned long time);
     void switchBuffers();

  private:
    enum {STATE_STOPPED, STATE_RUNNING} m_state;    // background timeout states
    enum {ADC_STATE_ACQUIRE, ADC_STATE_WAITING} m_adcState;// foreground
    enum {CYCLE_STATE_INIT,
          CYCLE_STATE_START, 
          CYCLE_POS, 
          CYCLE_NEG,
          CYCLE_NEG_FIRST_HALF , 
          CYCLE_POS_FIRST_HALF,
          CYCLE_NEG_SECOND_HALF,
          CYCLE_POS_SECOND_HALF,
          CYCLE_STATE_END} m_cycleState;  // background buffer individual cycle sates
    enum {
          PERIODS_STATE_START,
          PERIODS_COUNTING,
          PERIODS_END
         } m_periodsState;         // background buffers periods
    
    int m_periods, m_samples;
    float m_cyclePeriod;
    bool m_signal;
    bool m_overRun;
    int m_input;
    float m_sampleRate;
    float m_actualSampleRate;
    Buffer m_buffer1, m_buffer2, *m_pCurrentBuffer, *m_pWorkingBuffer, *m_pNextWorking;
    float averageValue, maxValue, minValue;
    ClosedLoopControl m_loopControl;
    
    FiltersIIR1 m_dcBlockingFilter;
};

//--------------------
extern BufferedAnalogAcquireTask AnalogAcquireTask;
#endif
