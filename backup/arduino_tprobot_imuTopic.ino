//include ==================================================================================
  //ROS
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>
#include <sensor_msgs/Imu.h>
  //Servo
#include <Servo.h>
Servo servo_cam_pitch, servo_cam_yaw;
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
  //servo
int servo_cam_pitch_pin = 9, servo_cam_yaw_pin = 10;
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
  //Display
long display_timer = 0;

//functions ==================================================================================
void move_actuator(const std_msgs::UInt16& cmd){
  //go up
       if (cmd.data == 1) {digitalWrite(ts_s1,LOW); digitalWrite(ts_s2,LOW);digitalWrite(ts_5v,LOW);digitalWrite(ts_gnd,LOW);}
  //go down
  else if (cmd.data == 2) {digitalWrite(ts_s1,HIGH);digitalWrite(ts_s2,HIGH);digitalWrite(ts_5v,LOW);digitalWrite(ts_gnd,LOW);}
  //stop
  else if (cmd.data == 0) {digitalWrite(ts_5v,HIGH);digitalWrite(ts_gnd,HIGH);}
}

void move_servo_cam(const std_msgs::UInt16MultiArray& cmd){
    servo_cam_pitch.write(cmd.data[0]);
    servo_cam_yaw.write(cmd.data[1]);
}

//setup the ros node and publisher ==================================================================================
std_msgs::String pub_msg;
sensor_msgs::Imu imu_msg;
ros::NodeHandle nh;
ros::Publisher pub_IMU("IMU_data", &imu_msg);
ros::Publisher pub_arduino("arduino_getData", &pub_msg);
ros::Subscriber<std_msgs::UInt16> sub_actuator("arduino_actuator", move_actuator);
ros::Subscriber<std_msgs::UInt16MultiArray> sub_servo_cam("arduino_servo", move_servo_cam);

//setup ==================================================================================
void setup()   {  
  //ROS node init.
  nh.initNode();
  nh.advertise(pub_IMU);
  nh.advertise(pub_arduino);
  nh.subscribe(sub_actuator);
  nh.subscribe(sub_servo_cam);

  //servo init
  servo_cam_pitch.attach(servo_cam_pitch_pin);
  servo_cam_yaw.attach(servo_cam_yaw_pin);
  
  //pinMode and Init
    // Actuator
      // switch 4 V+ // switch 3 V- // switch 2 // switch 1
  pinMode(ts_5v, OUTPUT);pinMode(ts_gnd, OUTPUT);pinMode(ts_s1, OUTPUT);pinMode(ts_s2, OUTPUT); 
      //Setup these pins to pull-up
  digitalWrite(ts_s1,HIGH);digitalWrite(ts_s2,HIGH);digitalWrite(ts_5v,HIGH);digitalWrite(ts_gnd,HIGH); 

  // IMU:start communication with IMU 
  status = IMU.begin();
  if (status < 0) {Status_sys = "IMU fails";}
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 49 for a 20 Hz update rate
  IMU.setSrd(49);

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
  String IMU_mx = String(IMU.getMagX_uT());    String IMU_my = String(IMU.getMagY_uT());    String IMU_mz = String(IMU.getMagZ_uT());
  String IMU_tempr = String(IMU.getTemperature_C());
    //Votages
  String V_logic = String(analogRead(pin_V_logic)/50.2);
  String V_motor = String(analogRead(pin_V_motor)/49.0);

  //Data organize
    //IMU
  String IMU_data = IMU_mx + " " + IMU_my + " " + IMU_mz + " " + IMU_tempr;
    //Voltages
  String Vol_data = V_logic + " " + V_motor;
    //Data organize - final
  String data = IMU_data + " " + Vol_data + "Z";

  //ROS operating
  int length = data.indexOf("Z") +2;
  char data_final[length+1];
  data.toCharArray(data_final, length+1);
  if (millis() > publisher_timer) {
    // publish data of IMU
    imu_msg.header.frame_id = "base_link";
    //imu_msg.orientation.x = 0;
    //imu_msg.orientation.y = -1;
    //imu_msg.orientation.z = -5;
    //imu_msg.orientation.w = 6;
    imu_msg.linear_acceleration.x = IMU.getAccelX_mss(); 
    imu_msg.linear_acceleration.y = IMU.getAccelY_mss();
    imu_msg.linear_acceleration.z = IMU.getAccelZ_mss();
    imu_msg.angular_velocity.x = IMU.getGyroX_rads(); 
    imu_msg.angular_velocity.y = IMU.getGyroY_rads(); 
    imu_msg.angular_velocity.z = IMU.getGyroZ_rads();
    pub_IMU.publish(&imu_msg);
    
    // publish data of everything else
    pub_msg.data = data_final;
    pub_arduino.publish(&pub_msg);
    publisher_timer = millis() + 42; //publish frequency 20Hz
    nh.spinOnce();
  }

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
