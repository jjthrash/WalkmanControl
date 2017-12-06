// File PwmPort.h
//


class PwmPort
{
  // uses Timer/Counter 2
  
  public:
    PwmPort(float frequency);
    void setup();
    void output(float value);
  private:
    float m_freq;
  
};

