 
#ifndef BTS7960_h
#define BTS7960_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class BTS7960
{
  public:
	  void init(uint8_t L_PWM, uint8_t R_PWM);
    void Enable();
    void Disable();

  	void TurnLeft(uint8_t pwm);
  	void TurnRight(uint8_t pwm);
  	void MotorStop();
    void SetMotorSpeed(uint8_t pwm, uint8_t turnLeft);
    void runMotorTest();

  private:
    uint8_t _L_PWM;
    uint8_t _R_PWM;
};
#endif
