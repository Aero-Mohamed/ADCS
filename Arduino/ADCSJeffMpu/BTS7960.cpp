 
#include "BTS7960.h"

void BTS7960::init(uint8_t L_PWM, uint8_t R_PWM){
	_R_PWM = R_PWM;
	_L_PWM = L_PWM;
	pinMode(_R_PWM, OUTPUT);
	pinMode(_L_PWM, OUTPUT);
}

void BTS7960::TurnRight(uint8_t pwm){
     analogWrite(_L_PWM, 0);
	 delayMicroseconds(100);
     analogWrite(_R_PWM, pwm);
}

void BTS7960::TurnLeft(uint8_t pwm){
     analogWrite(_R_PWM, 0);
	 delayMicroseconds(100);
     analogWrite(_L_PWM, pwm);
}


void BTS7960::MotorStop(){
  analogWrite(_L_PWM, LOW);
  analogWrite(_R_PWM, LOW);
}

void BTS7960::SetMotorSpeed(uint8_t pwm, uint8_t turnLeft){
  if(pwm > 1){
    if(turnLeft == 1){
      TurnLeft(100);
      delay(50);
      TurnLeft(pwm);
    }else{
      TurnRight(100);
      delay(50);
      TurnRight(pwm);
    }  
  }else{
    MotorStop();
  }
  
}

void BTS7960::runMotorTest(){
  SetMotorSpeed(30, 1);
  delay(3000);
  MotorStop();
  delay(3000);


  SetMotorSpeed(30, 0);
  delay(3000);
  MotorStop();
}
