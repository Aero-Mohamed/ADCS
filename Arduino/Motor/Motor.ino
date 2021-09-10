
#define EN1 8
#define EN2 9


void setup() {
  // put your setup code here, to run once:
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
}

void loop() {

  runMotor(30);
  delay(10000);
  stopMotor();
  delay(7000);

  runMotor(25);
  delay(10000);
  stopMotor();
  delay(7000);

  runMotor(20);
  delay(10000);
  stopMotor();
  delay(7000);

  runMotor(15);
  delay(10000);
  stopMotor();
  delay(7000);

  runMotor(10);
  delay(10000);
  stopMotor();
  delay(7000);
  
}

void runMotor(int pwm){
  analogWrite(EN1, 0);
  delayMicroseconds(100);
  analogWrite(EN2, 100);
  delay(50);
  analogWrite(EN2, pwm);
}

void stopMotor(){
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
  
}
