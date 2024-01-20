#include <Servo.h>

Servo motor;
Servo motor1;



void setup() {
  // put your setup code here, to run once:
  motor.attach(8);
  motor.write(90);
  motor1.attach(9);
  motor1.write(90);

  Serial.begin(115200);
  Serial.setTimeout(1);

}

void loop() {
  // put your main code here, to run repeatedly:

  if(Serial.available())
  {
    int angle = Serial.readString().toInt();
    motor.write(angle);
    motor1.write(angle);
    
  }
  delay(0.1);

}
