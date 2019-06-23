//include ==================================================================================
  //ROS
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
  //Servo
#include <Servo.h>
Servo servo_cam;
  //OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
  //IMU
#include <MPU9250.h>
MPU9250 IMU(Wire,0x68);
  //MotorDriver
#include "RoboClaw.h"
RoboClaw roboclaw(&Serial2,10000);
#define address1 0x80
#define address2 0x81
#define Kp 1.0
#define Ki 0.5
#define Kd 0.25
#define qpps 44000

//pin name ==================================================================================
  //servo
int servo_cam_pin = 9;
  //acctuator
int ts_5v = 5,ts_gnd = 4,ts_s1 = 3,ts_s2 = 2;
  //votages
int pin_V_logic = A1, pin_V_motor = A0;

//var declarence ==================================================================================
  //ROS
long publisher_timer;
  //servo
int servo_cam_angle = 0;
  //Votages
float V_logic = 0,V_motor = 0;
  //IMU
int status;
float IMU_ax = 0,IMU_ay = 0,IMU_az = 0,IMU_gx = 0,IMU_gy = 0,IMU_gz = 0,IMU_mx = 0,IMU_my = 0,IMU_mz = 0,IMU_tempr = 0;
  //Status
String Status_sys = "Ready",Status_charger = "OUT-OFF";
  //Serial
int Msg_Serial = 0;
  //Display
long display_timer = 0;

//functions ==================================================================================
void move_actuator(const std_msgs::String& cmd){
  //go up
       if (int(cmd.data[0]) == 'u') {digitalWrite(ts_s1,LOW); digitalWrite(ts_s2,LOW);digitalWrite(ts_5v,LOW);digitalWrite(ts_gnd,LOW);}
  //go down
  else if (int(cmd.data[0]) == 'd') {digitalWrite(ts_s1,HIGH);digitalWrite(ts_s2,HIGH);digitalWrite(ts_5v,LOW);digitalWrite(ts_gnd,LOW);}
  //stop
  else if (int(cmd.data[0]) == 's') {digitalWrite(ts_5v,HIGH);digitalWrite(ts_gnd,HIGH);}
}

void move_servo_cam(const std_msgs::UInt16& cmd){
    servo_cam.write(cmd.data);
}

//setup the ros node and publisher ==================================================================================
std_msgs::String pub_msg;
ros::NodeHandle nh;
ros::Publisher pub_arduino("arduino_getData", &pub_msg);
ros::Subscriber<std_msgs::String> sub_actuator("arduino_actuator", move_actuator);
ros::Subscriber<std_msgs::UInt16> sub_servo_cam("arduino_servo", move_servo_cam);

//setup ==================================================================================
void setup()   {  
  //ROS node init.
  nh.initNode();
  nh.advertise(pub_arduino);
  nh.subscribe(sub_actuator);
  nh.subscribe(sub_servo_cam);

  //servo init
  servo_cam.attach(servo_cam_pin);

  //motorDriver serial
  roboclaw.begin(115200); delay(200);//Serial2
  roboclaw.SetM1VelocityPID(address1,Kd,Kp,Ki,qpps);
  roboclaw.SetM2VelocityPID(address1,Kd,Kp,Ki,qpps);
  roboclaw.SetM1VelocityPID(address2,Kd,Kp,Ki,qpps);
  roboclaw.SetM2VelocityPID(address2,Kd,Kp,Ki,qpps);
  roboclaw.SpeedAccelM1(address1,12000,0);
  roboclaw.SpeedAccelM2(address1,12000,0);
  roboclaw.SpeedAccelM1(address2,12000,0);
  roboclaw.SpeedAccelM2(address2,12000,0);
  
  //pinMode and Init
    // Actuator
      // switch 4 V+ // switch 3 V- // switch 2 // switch 1
  pinMode(ts_5v, OUTPUT);pinMode(ts_gnd, OUTPUT);pinMode(ts_s1, OUTPUT);pinMode(ts_s2, OUTPUT); 
      //Setup these pins to pull-up
  digitalWrite(ts_s1,HIGH);digitalWrite(ts_s2,HIGH);digitalWrite(ts_5v,HIGH);digitalWrite(ts_gnd,HIGH); 
    
  // serial to display data
  Serial.begin(57600);while(!Serial) {}delay(200);

  // IMU:start communication with IMU 
  status = IMU.begin();
  if (status < 0) {Status_sys = "IMU fails";}
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);

  // OLED:by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.display();  // show welcome screen
  delay(500);
    // welcome animation
  for (int i=1; i <= 3; i++){for (int j=1; j <= 50; j++){display.invertDisplay(true);delay(3-i);display.invertDisplay(false);delay(3-i);}}
  display.setTextSize(2);display.setTextColor(WHITE);
}

// main loop ==================================================================================
void loop() {
  //Data collecting
    //IMU: Data collecting
  IMU.readSensor();
  String IMU_ax = String(IMU.getAccelX_mss()); String IMU_ay = String(IMU.getAccelY_mss()); String IMU_az = String(IMU.getAccelZ_mss());
  String IMU_gx = String(IMU.getGyroX_rads()); String IMU_gy = String(IMU.getGyroY_rads()); String IMU_gz = String(IMU.getGyroZ_rads());
  String IMU_mx = String(IMU.getMagX_uT());    String IMU_my = String(IMU.getMagY_uT());    String IMU_mz = String(IMU.getMagZ_uT());
  String IMU_tempr = String(IMU.getTemperature_C());
    //Votages
  String V_logic = String(analogRead(pin_V_logic)/51.62);
  String V_motor = String(analogRead(pin_V_motor)/51.62);

  //Data organize
    //IMU
  String IMU_data = IMU_ax + " " + IMU_ay + " " + IMU_az + " " + IMU_gx + " " + IMU_gy + " " + IMU_gz + " " + IMU_mx + " " + IMU_my + " " + IMU_mz + " " + IMU_tempr;
    //Voltages
  String Vol_data = V_logic + " " + V_motor;
    //Data organize - final
  String data = IMU_data + " " + Vol_data + "Z";

  //ROS operating
  int length = data.indexOf("Z") +2;
  char data_final[length+1];
  data.toCharArray(data_final, length+1);
  if (millis() > publisher_timer) {
    // request reading from sensor
    pub_msg.data = data_final;
    pub_arduino.publish(&pub_msg);
    publisher_timer = millis() + 50; //publish frequency 20Hz
    nh.spinOnce();
  }

  //motor
  roboclaw.SpeedAccelM1(address1,3000,2000);
  roboclaw.SpeedAccelM2(address1,3000,-2000);
  roboclaw.SpeedAccelM1(address2,3000,2000);
  roboclaw.SpeedAccelM2(address2,3000,-2000);

  //Display
  if (millis() > display_timer) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("  TPRobot");
    display.println(String(millis()/1000) + "." + String(millis()%1000).substring(0,2) + "s");
    display.println(IMU_tempr + (char)247 +"C");
    display.print("L" + V_logic.substring(0, 4) + "M" + V_motor.substring(0,4));
    display.display();
    display_timer = millis() + 10000; //display frequency 0.1Hz
  }
}
