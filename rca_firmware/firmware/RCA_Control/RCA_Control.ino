#include <Servo.h>

                            // Pin Belegung
#define SERVO_BASE_PIN 8
#define SERVO_SHOULDER_PIN 9
#define SERVO_ARM_PIN 10
#define SERVO_ROTATE_PIN 11
#define SERVO_ELBOW_PIN 12 
#define SERVO_GRIPPER_PIN 13
                            // Servo Start Position
#define BASE_START 90
#define SHOULDER_START 97 // 130
#define ARM_START 90 // 175
#define ROTATE_START 85
#define ELBOW_START 90
#define GRIPPER_START 90
                            // Servo Opjekt erstellen
Servo base;
Servo shoulder;
Servo arm;
Servo rotate;
Servo elbow;
Servo gripper;

uint8_t idx = 0;
uint8_t value_idx = 0;
char value[4] = "000";

void reach_goal(Servo& motor, int goal)
{
  if(goal>=motor.read())
  {
    for(int pos= motor.read(); pos<=goal; pos++)
    {
      motor.write(pos);
      delay(5);
    }  
  }
  else
  {
    for(int pos = motor.read(); pos>= goal; pos--)
    {
      motor.write(pos);
      delay(5);
    }  
  }
    
}


void setup() {
                            // Servo Anschluss
                            /*FEHLER SUCHE
                            DA DER ROBOTER ARM SEINE  POSITION NICHT HÄLT, SOLLTEN MIN UND MAX WERTE EINGETRAGEN WERDEN 
                            DEFAULT 544 - 2400
                            1000 - 2000 TESTEN
                            */
  base.attach(SERVO_BASE_PIN);
  shoulder.attach(SERVO_SHOULDER_PIN);
  arm.attach(SERVO_ARM_PIN);
  rotate.attach(SERVO_ROTATE_PIN);
  elbow.attach(SERVO_ELBOW_PIN);
  gripper.attach(SERVO_GRIPPER_PIN);
  
                            // Servo Start Position Aktivieren
  base.write(BASE_START);
  shoulder.write(SHOULDER_START);
  arm.write(ARM_START);
  rotate.write(ROTATE_START);
  elbow.write(ELBOW_START);
  gripper.write(GRIPPER_START);

  Serial.begin(115200);
  Serial.setTimeout(1);
  

}

void loop() {
  if(Serial.available())
  {
    char chr = Serial.read();  
    if(chr=='b')
    {
      idx = 0;
      value_idx = 0;
    }
    
    else if (chr == 's')
    {
      idx = 1;
      value_idx = 0;
    }

    else if (chr == 'a')
    {
      idx = 2;
      value_idx = 0;
    }

    else if (chr == 'r')
    {
      idx = 3;
      value_idx = 0;
    }
    
    else if (chr == 'e')
    {
      idx = 4;
      value_idx = 0;
    }
    
    else if (chr == 'g')
    {
      idx = 5;
      value_idx = 0;
    }
    else if (chr == ',')
    {
      int val = atoi (value);
      if(idx== 0)
      {
        reach_goal(base, val);  
      }
      else if(idx== 1)
      {
        reach_goal(shoulder, val);  
      }
      else if(idx== 2)
      {
        reach_goal(arm, val);  
      }
      else if(idx== 3)
      {
        reach_goal(rotate, val);  
      }
      else if(idx== 4)
      {
        reach_goal(elbow, val);  
      }
      else if(idx== 5)
      {
        reach_goal(gripper, val);  
      }

    value[0]='0';
    value[1]='0';
    value[2]='0';
    value[3]='0';
    value[4]='0';
    value[5]='\0';
      
    }
    else
    {
        value[value_idx] = chr;
        value_idx++;
    }
  }
  

}
