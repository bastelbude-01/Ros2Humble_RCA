#include <Servo.h>

Servo motor;

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

void setup() {
  // put your setup code here, to run once:
  motor.attach(9);
  motor.write(125);
  
  motor1.attach(10);
  motor1.write(90);

  motor2.attach(11);
  motor2.write(85);

  motor3.attach(12);
  motor3.write(90);

  motor4.attach(13);
  motor4.write(90);

  Serial.begin(115200);
  Serial.setTimeout(1);

}

void loop() {
  // put your main code here, to run repeatedly:

  if(Serial.available())
  {
    int angle = Serial.readString().toInt();
    motor4.write(angle);
    
  }
  delay(0.1);

}
