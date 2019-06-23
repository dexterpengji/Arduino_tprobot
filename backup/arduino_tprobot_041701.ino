//include ==================================================================================
  //ROS
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;
std_msgs::Float32 temp_msg;
ros::Publisher pub_temp("temperature", &temp_msg);

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

//pin name ==================================================================================
  //acctuator
int ts_5v = 5,ts_gnd = 4,ts_s1 = 3,ts_s2 = 2;
  //votages
int pin_V_logic = A1, pin_V_motor = A0;

//var declarence ==================================================================================
  //Votages
float V_logic = 0,V_motor = 0;
  //IMU
int status;
float Acc_ax = 0,Acc_ay = 0,Acc_az = 0,Acc_gx = 0,Acc_gy = 0,Acc_gz = 0,Acc_mx = 0,Acc_my = 0,Acc_mz = 0;
  //Status
String Status_sys = "Ready",Status_charger = "OUT-OFF";
  //Serial
int Msg_Serial = 0;

// setup ==================================================================================
void setup()   {  
  //pinMode and Init
    // Actuator
  pinMode(ts_5v, OUTPUT);pinMode(ts_gnd, OUTPUT);pinMode(ts_s1, OUTPUT);pinMode(ts_s2, OUTPUT); // switch 4 V+ // switch 3 V- // switch 2 // switch 1
  digitalWrite(ts_s1,HIGH);digitalWrite(ts_s2,HIGH);digitalWrite(ts_5v,HIGH);digitalWrite(ts_gnd,HIGH); //Setup these pins to pull-up
  
  // serial to display data
  Serial.begin(19200);while(!Serial) {}delay(200);

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
  display.display();delay(3000);// show welcome screen
  for (int i=1; i <= 3; i++){for (int j=1; j <= 50; j++){display.invertDisplay(true);delay(3-i);display.invertDisplay(false);delay(3-i);}}// welcome animation
  display.setTextSize(1);display.setTextColor(WHITE);
  
  //ROS temperature
  nh.initNode();
  nh.advertise(pub_temp);
}

//ROS publisher_timer
long publisher_timer;

// main loop ==================================================================================
void loop() {
  //Data collecting
    //IMU: Data collecting
  IMU.readSensor();
  Acc_ax = IMU.getAccelX_mss(); Acc_ay = IMU.getAccelY_mss(); Acc_az = IMU.getAccelZ_mss();
  Acc_gx = IMU.getGyroX_rads(); Acc_gy = IMU.getGyroY_rads(); Acc_gz = IMU.getGyroZ_rads();
  Acc_mx = IMU.getMagX_uT(); Acc_my = IMU.getMagY_uT(); Acc_mz = IMU.getMagZ_uT();
    //Votages
  V_logic = analogRead(pin_V_logic)/51.62;
  V_motor = analogRead(pin_V_motor)/51.62;

  //Serial: transfer data
    //IMU
  Serial.print(Acc_ax,3);Serial.print(" ");Serial.print(IMU.getAccelY_mss(),3);Serial.print(" ");Serial.print(IMU.getAccelZ_mss(),3);Serial.print(" ");
  Serial.print(IMU.getGyroX_rads(),3);Serial.print(" ");Serial.print(IMU.getGyroY_rads(),3);Serial.print(" ");Serial.print(IMU.getGyroZ_rads(),3);Serial.print(" ");
  Serial.print(IMU.getMagX_uT(),3);Serial.print(" ");Serial.print(IMU.getMagY_uT(),3);Serial.print(" ");Serial.print(IMU.getMagZ_uT(),3);Serial.print(" ");
  Serial.print(IMU.getTemperature_C(),3);Serial.print(" ");
    //Votages
  Serial.print(V_logic);Serial.print(" ");Serial.print(V_motor);Serial.println(" ");

  //Serial: read command and do
  if (Serial.available() > 0){Msg_Serial = Serial.read();}
  if (Msg_Serial != 0){move_actuator(Msg_Serial);Msg_Serial = 0;}
  
  //OLED: display
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("S_sys:");display.println(Status_sys);
  display.print("S_cha:");display.print(Status_charger);
  display.setCursor(0,21);
  display.print("V_l:");display.print(V_logic);display.print(" V_m:");display.println(V_motor);
  display.setCursor(0,35);
  display.print("A:");display.print(Acc_ax);display.print(" ");display.print(Acc_ax);display.print(" ");display.println(Acc_az);
  display.print("G:");display.print(Acc_gx);display.print(" ");display.print(Acc_gy);display.print(" ");display.println(Acc_gz);
  display.print("M:");display.print(Acc_mx);display.print(" ");display.print(Acc_my);display.print(" ");display.println(Acc_mz);
  display.display();
  
  //ROS
  if (millis() > publisher_timer) {

  temp_msg.data = IMU.getTemperature_C();
  pub_temp.publish(&temp_msg);
  
  publisher_timer = millis() + 1000;
  }
  
  //ROS spinOnce
  nh.spinOnce();
}

// functions ==================================================================================
int move_actuator(int cmd)
{      if (cmd == 'u') {digitalWrite(ts_s1,LOW); digitalWrite(ts_s2,LOW);digitalWrite(ts_5v,LOW);digitalWrite(ts_gnd,LOW);} //go up
  else if (cmd == 'd') {digitalWrite(ts_s1,HIGH);digitalWrite(ts_s2,HIGH);digitalWrite(ts_5v,LOW);digitalWrite(ts_gnd,LOW);} //go down
  else if (cmd == 's') {digitalWrite(ts_5v,HIGH);digitalWrite(ts_gnd,HIGH);} //stop
  else {return 0;}}
