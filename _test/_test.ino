//include ==================================================================================
  //Ultrasonic
#include<UltraDistSensor.h>
UltraDistSensor UltrasonicSensor_1;
UltraDistSensor UltrasonicSensor_2;
UltraDistSensor UltrasonicSensor_3;
UltraDistSensor UltrasonicSensor_4;
  //MotorDriver
#include "RoboClaw.h"
RoboClaw roboclaw(&Serial2,10000);
#define address 0x80
    //MotorDriver - Velocity PID coefficients
#define Kp 1.0
#define Ki 0.5
#define Kd 0.25
#define qpps 44000

//pin name ==================================================================================
  //votages
int pin_V_logic = A1 , pin_V_motor = A0;

//var declarence ==================================================================================
  //Votages
float V_logic = 0,V_motor = 0;
  //Status
String Status_sys = "Ready",Status_charger = "OUT-OFF";
  //Serial
int Msg_Serial = 0;
char Msg_Serial_end = 's';
  //Ultrasonic
int Ultr_disCM[] = {150, 150, 150, 150};
int Ultr_limit[] = {40, 30, 30, 30};
int Ultr_sensity = 35;
  //motor speed
int motorSpeed_l = 500;
int motorSpeed_r = 500;
int motorSpeed_limit = 5000;

// setup ==================================================================================
void setup()   {  
  // serial to display data
  Serial.begin(19200);while(!Serial) {}delay(200);

  // serial connected to Bluetooth
  Serial1.begin(19200);while(!Serial1) {}delay(200);

  // serial connected to motorDriver
  roboclaw.begin(38400); //Serial2

  //Ultrasonic
  UltrasonicSensor_1.attach(7,3);//Trigger pin , Echo pin
  UltrasonicSensor_2.attach(7,4);//Trigger pin , Echo pin
  UltrasonicSensor_3.attach(7,5);//Trigger pin , Echo pin
  UltrasonicSensor_4.attach(7,6);//Trigger pin , Echo pin

  // motorDriver - Set PID Coefficients
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM2VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw.SpeedAccelM1(address,12000,0);
  roboclaw.SpeedAccelM2(address,12000,0);
}

// main loop ==================================================================================
void loop() {
  //Data collecting
    //Votages
  V_logic = analogRead(pin_V_logic)/51.6;
  V_motor = analogRead(pin_V_motor)/51.6;
    //Ultrasonic read
  Ultr_disCM[0] = UltrasonicSensor_1.distanceInCm();
  Ultr_disCM[1] = UltrasonicSensor_2.distanceInCm();
  Ultr_disCM[2] = UltrasonicSensor_3.distanceInCm();
  Ultr_disCM[3] = UltrasonicSensor_4.distanceInCm();
    //Ultrasonic correct
  for (int i = 0; i < 4; i++) {
    if (Ultr_disCM[i] == 0 || Ultr_disCM[i] > 150 ){
      Ultr_disCM[i] = 150;
      }
    }
    //anti-collision logic
  //stop
  if (motorSpeed_l == 500 && motorSpeed_r == 500) {
    Ultr_limit[0] = 0;
    Ultr_limit[1] = 0;
    Ultr_limit[2] = 0;
    Ultr_limit[3] = 0;
    }
  //go forward || sharp turn
  if (motorSpeed_l > 500 && motorSpeed_r > 500 || (motorSpeed_l-500)*(motorSpeed_r-500) <= 0) {
    Ultr_limit[0] = Ultr_sensity;
    Ultr_limit[1] = Ultr_sensity;
    Ultr_limit[2] = Ultr_sensity;
    Ultr_limit[3] = 0;
    }
  //go backward
  if (motorSpeed_l < 500 && motorSpeed_r < 500) {
    Ultr_limit[0] = 0;
    Ultr_limit[1] = 0;
    Ultr_limit[2] = 0;
    Ultr_limit[3] = Ultr_sensity;
    }
  if (Ultr_disCM[0] < Ultr_limit[0] || Ultr_disCM[1] < Ultr_limit[1] || Ultr_disCM[2] < Ultr_limit[2] || Ultr_disCM[3] < Ultr_limit[3]){    
    roboclaw.SpeedAccelM1(address,15000,0);
    roboclaw.SpeedAccelM2(address,15000,0);
    motorSpeed_l = 0;
    motorSpeed_r = 0;
    }  
  
  //Serial: transfer data
    //Ultrasonic
  Serial1.print(Ultr_disCM[0]);Serial1.print(" ");
  Serial1.print(Ultr_disCM[1]);Serial1.print(" ");
  Serial1.print(Ultr_disCM[2]);Serial1.print(" ");
  Serial1.print(Ultr_disCM[3]);Serial1.print(" ");
    //Votages
  Serial1.print(V_logic);Serial1.print(" ");
  Serial1.print(V_motor);Serial1.println(" ");

  //Serial: read command and do
  if (Serial1.available() > 3){
    Msg_Serial = Serial1.parseInt();
    Msg_Serial_end = Serial1.read();
    switch (Msg_Serial_end) {
      case 'g':
        if (Msg_Serial >= 0 && Msg_Serial < 1000){
          roboclaw.SpeedAccelM1(address,3000,map(Msg_Serial, 1, 999, motorSpeed_limit, -motorSpeed_limit));
          roboclaw.SpeedAccelM2(address,3000,map(Msg_Serial, 1, 999, -motorSpeed_limit, motorSpeed_limit));
          motorSpeed_l = Msg_Serial;
          motorSpeed_r = Msg_Serial;
        }
        break;
      case 'l':
        if (Msg_Serial >= 0 && Msg_Serial < 1000){
          roboclaw.SpeedAccelM1(address,3000,map(Msg_Serial, 1, 999, motorSpeed_limit, -motorSpeed_limit));
          motorSpeed_l = Msg_Serial;
        }
        break;
      case 'r':
        if (Msg_Serial >= 0 && Msg_Serial < 1000){
          roboclaw.SpeedAccelM2(address,3000,map(Msg_Serial, 1, 999, -motorSpeed_limit, motorSpeed_limit));
          motorSpeed_r = Msg_Serial;
        }
        break;
      case 's':
        roboclaw.SpeedAccelM1(address,7000,0);
        roboclaw.SpeedAccelM2(address,7000,0);
        motorSpeed_l = 0;
        motorSpeed_r = 0;
        break;
      case 'z':
        if (Msg_Serial >= 0 && Msg_Serial < 1000){
            motorSpeed_limit = map(Msg_Serial, 0, 999, 0, 12000);
          }
      case 'u':
        if (Msg_Serial >= 0 && Msg_Serial < 1000){
            Ultr_sensity = map(Msg_Serial, 0, 999, 0, 70);
          }
        break;
      default:
        break;
      }
    }
}
