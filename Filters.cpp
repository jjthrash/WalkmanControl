// Digital Filter implementation

#import "Filters.h"
// Filters base class
Filters::Filters()
{
  
}

// ----------------------- FiltersIIR1
FiltersIIR1::FiltersIIR1()
{
  
}

FiltersIIR1::FiltersIIR1(float a0, float a1, float b1) : m_a0(a0), m_a1(a1),m_b1(b1),
                                                         x_1(0), y_1(0)
{
  
}

void FiltersIIR1::setCoefficients(float a0, float a1, float b1)
{
  m_a0 = a0;
  m_a1 = a1;
  m_b1 = b1;
}


float FiltersIIR1::update(float x)
{
    float y;
    y = m_a0 * x + m_a1 * x_1 + m_b1 * y_1;
    // save state
    x_1 = x;
    y_1 = y;
    return y;
}

