//include ==================================================================================

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
float IMU_ax = 0,IMU_ay = 0,IMU_az = 0,IMU_gx = 0,IMU_gy = 0,IMU_gz = 0,IMU_mx = 0,IMU_my = 0,IMU_mz = 0,IMU_tempr = 0;
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
}


// main loop ==================================================================================
void loop() {
  //Data collecting
    //IMU: Data collecting
  IMU.readSensor();
  IMU_ax = IMU.getAccelX_mss(); IMU_ay = IMU.getAccelY_mss(); IMU_az = IMU.getAccelZ_mss();
  IMU_gx = IMU.getGyroX_rads(); IMU_gy = IMU.getGyroY_rads(); IMU_gz = IMU.getGyroZ_rads();
  IMU_mx = IMU.getMagX_uT(); IMU_my = IMU.getMagY_uT(); IMU_mz = IMU.getMagZ_uT();
  IMU_tempr = IMU.getTemperature_C();
    //Votages
  V_logic = analogRead(pin_V_logic)/51.62;
  V_motor = analogRead(pin_V_motor)/51.62;

  //Serial: transfer data
    //IMU
  Serial.print(IMU_ax,3);Serial.print(" ");Serial.print(IMU_ay,3);Serial.print(" ");Serial.print(IMU_az,3);Serial.print(" ");
  Serial.print(IMU_gx,3);Serial.print(" ");Serial.print(IMU_gy,3);Serial.print(" ");Serial.print(IMU_gz,3);Serial.print(" ");
  Serial.print(IMU_mx,3);Serial.print(" ");Serial.print(IMU_my,3);Serial.print(" ");Serial.print(IMU_mz,3);Serial.print(" ");
  Serial.print(IMU_tempr,3);Serial.print(" ");
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
  display.print("A:");display.print(IMU_ax);display.print(" ");display.print(IMU_ax);display.print(" ");display.println(IMU_az);
  display.print("G:");display.print(IMU_gx);display.print(" ");display.print(IMU_gy);display.print(" ");display.println(IMU_gz);
  display.print("M:");display.print(IMU_mx);display.print(" ");display.print(IMU_my);display.print(" ");display.println(IMU_mz);
  display.display();
}

// functions ==================================================================================
int move_actuator(int cmd)
{      if (cmd == 'u') {digitalWrite(ts_s1,LOW); digitalWrite(ts_s2,LOW);digitalWrite(ts_5v,LOW);digitalWrite(ts_gnd,LOW);} //go up
  else if (cmd == 'd') {digitalWrite(ts_s1,HIGH);digitalWrite(ts_s2,HIGH);digitalWrite(ts_5v,LOW);digitalWrite(ts_gnd,LOW);} //go down
  else if (cmd == 's') {digitalWrite(ts_5v,HIGH);digitalWrite(ts_gnd,HIGH);} //stop
  else {return 0;}}
