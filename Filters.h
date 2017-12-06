// Filters.h   Digital filters

class Filters
{
  public:
    Filters();
    virtual float update(float x) = 0;
};


class FiltersIIR1: public Filters
{
  public:
    FiltersIIR1();
    FiltersIIR1(float a0, float a1, float b1);
    void setCoefficients(float a0, float a1, float b1);
    float update(float x);
  private:
    float m_a0, m_a1, m_b1;  // coeffs
    float x_1, y_1;          // state
  
};


