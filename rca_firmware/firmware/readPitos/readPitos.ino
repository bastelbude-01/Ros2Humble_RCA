int base_pito = A0;
int base_pito_wert = 0;
int base_ = 0;

int shoulder_pito = A1;
int shoulder_pito_wert = 0;
int shoulder_ = 0;

int arm_pito = A2;
int arm_pito_wert = 0;
int arm_ = 0;

int rotate_pito = A3;
int rotate_pito_wert = 0;
int rotate_ = 0;

int nick_pito = A4;
int nick_pito_wert = 0;
int nick_ = 0;

int gripper_pito = A5;
int gripper_pito_wert = 0;
int gripper_ = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  base_pito_wert = analogRead(base_pito);
  base_ = map(base_pito_wert, 0, 660, 0,180);

  shoulder_pito_wert = analogRead(shoulder_pito);
  shoulder_ = map(shoulder_pito_wert, 0, 660, 0,180);

  arm_pito_wert = analogRead(base_pito);
  arm_ = map(base_pito_wert, 0, 660, 0,180);

  rotate_pito_wert = analogRead(rotate_pito);
  rotate_ = map(rotate_pito_wert, 150, 460, 0,180);

  nick_pito_wert = analogRead(nick_pito);
  nick_ = map(nick_pito_wert, 150, 460, 0,180);

  gripper_pito_wert = analogRead(gripper_pito);
  gripper_ = map(gripper_pito_wert, 150, 460, 0,180);


  Serial.print("Base Pito : ");
  Serial.print(base_);
  Serial.print("; Shoulder Pito : ");
  Serial.print(shoulder_);
  Serial.print("; Arm Pito : ");
  Serial.print(arm_);
  Serial.print("; Rotate Pito : ");
  Serial.print(rotate_);
  Serial.print("; Nick Pito : ");
  Serial.print(nick_);
  Serial.print("; Gripper Pito : ");
  Serial.println(gripper_);
  delay(500);

}
