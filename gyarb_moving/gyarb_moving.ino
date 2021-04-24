//Include all necessary libraries
#include "esp_deep_sleep.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <WiFi.h>    // Built-in
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"
#include <Adafruit_INA219.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41); // Change I2C adress for servo driver
#define SERVOMIN  590 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  2800 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 300 //Servo frequency for digital servos
// Define variables
int pulseleng;
uint8_t servoUpDown = 0;
uint8_t servoSide = 1;
int azipulse;
int elepulse;
const char* ssid     = "";
const char* password = "";
float Lon = 12.84 * DEG_TO_RAD,
      Lat = 55.39 * DEG_TO_RAD,
      elevation,
      azimuth;
int sun_azimuth;
int sun_elevation;
String time_str, current_hour, current_minute, current_day, current_month, current_year;

//Establish connection to Adafruit IO feeds
AdafruitIO_Feed *Current = io.feed("Current");
AdafruitIO_Feed *Voltage = io.feed("Voltage");
AdafruitIO_Feed *Current_after = io.feed("Current_after");
AdafruitIO_Feed *Voltage_after = io.feed("Voltage_after");
AdafruitIO_Feed *Current_difference = io.feed("Current_difference");
AdafruitIO_Feed *Voltage_difference = io.feed("Voltage_difference");


Adafruit_INA219 ina219;


//#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex

static RTC_DATA_ATTR struct timeval sleep_enter_time;
RTC_DATA_ATTR int blinkAcc;
RTC_DATA_ATTR long  time2sleep;

const long wakeup_time_usec = 60 * 15 * 1000000; //15 minutes sleep time
const long DSoffset = 148 * 1000; //sligth extra time adjusted for DS

void setup(void){
  if (time2sleep == 0) time2sleep = wakeup_time_usec; //first time its not set

  Serial.begin(115200);

  struct timeval now;
  gettimeofday(&now, NULL);
  //Below: calc the time it was asleep before woken up
  long sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;
  Serial.print("sleep_time_ms: ");
  Serial.println(sleep_time_ms);
  Serial.print("time2sleep: us");
  Serial.println(time2sleep);

  esp_deep_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_deep_sleep_get_wakeup_cause();

  //Connect to INA219 featherwing
  uint32_t currentFrequency;
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  //Connect to Adafruit IO
  io.connect();
  while(io.status() < AIO_CONNECTED) {
    Serial.println(io.statusText());
    delay(500);
  }
  Serial.println();
  Serial.println(io.statusText());
  
  io.run();
  float shuntvoltage = 0;
  float busvoltage[999];
  float current_mA[999];
  float loadvoltage = 0;
  float power_mW = 0;
  
  float current_sum = 0;
  float busvoltage_sum = 0;
  
  
  float current_avg = 0;
  float busvoltage_avg = 0;
  
  //Measure current 1000 times
  for (int i=0; i < 1000; i++){
    current_mA[i] = ina219.getCurrent_mA();
    current_sum = current_sum + current_mA[i];//Calculate the total sum
  }
  current_avg = current_sum / 1000; //Calculate the current avarage
  //Measure voltage 1000 times
  for (int i=0; i < 1000; i++){
    busvoltage[i] = ina219.getBusVoltage_V();
    busvoltage_sum = busvoltage_sum + busvoltage[i];//Calculate the total sum
  }
  busvoltage_avg = busvoltage_sum / 1000;//Calculate the voltage avarage
  
  shuntvoltage = ina219.getShuntVoltage_mV();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage_avg + (shuntvoltage / 1000);
  Serial.println("Current: ");
  Serial.println(current_avg);
  Serial.println("Bus Voltage: ");
  Serial.println(busvoltage_avg);
  delay(1000);

//_______________END OF INA 219 GET CURRENT_______________
//_______________SERVO ROTATION___________________________
  //Start servos
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  //StartWiFi(ssid, password);
  StartTime();// 
  UpdateLocalTime();
  Serial.print("Update time:");
  Serial.println(Update_DateTime());
  

  if(current_hour.toInt()>20){//// Dont rotate servo if its too late
    time2sleep = 60 * 30 * 1000000; //reset sleep time for 30 minutes
    esp_deep_sleep_enable_timer_wakeup(time2sleep);//30 minutes between send readings wake ups
    gettimeofday(&sleep_enter_time, NULL);//store on RTC at what time you enter sleep
    Serial.println("Going to sleep...");
    esp_deep_sleep_start();
  }
  if(current_hour.toInt()<6){//// Dont rotate servo if its too early
    time2sleep = 60 * 30 * 1000000; //reset sleep time for 30 minutes
    esp_deep_sleep_enable_timer_wakeup(time2sleep);//30 minutes between send readings wake ups
    gettimeofday(&sleep_enter_time, NULL);//store on RTC at what time you enter sleep
    Serial.println("Going to sleep...");
    esp_deep_sleep_start();
  }
  else{
    time2sleep = wakeup_time_usec; //15 minutes sleep time
  }
  
  Calculate_Sun_Position(current_hour.toInt(), current_minute.toInt(), 0, current_day.toInt(), current_month.toInt(), current_year.toInt()); // parameters are HH:MM:SS DD:MM:YY start from midnight and work out all 24 hour positions.

  azipulse=map(sun_azimuth, 90, 270, 180, 0);//Map the sun position to degrees
  pulseleng=map(azipulse, 0, 180, SERVOMIN, SERVOMAX);// Map the degrees to PWM
  pwm.setPWM(servoSide, 0, pulseleng);//Move the servo to correct position
  delay(1000);
  pwm.setPWM(servoSide, 0, 4096);//servo "sleep"
  if (sun_elevation < 0) {
    sun_elevation = 0; // Point at horizon if less than horizon
  }
  sun_elevation = 145 - sun_elevation;
  sun_elevation = map(sun_elevation, 0, 180, SERVOMIN, SERVOMAX);// Map the degrees to PWM
  pwm.setPWM(servoUpDown, 0, sun_elevation);  // Move the servo to correct position
  delay(1000);
  pwm.setPWM(servoUpDown, 0, 4096);//servo "sleep"
 
//______________END OF SERVO ROTATION_________________________

//_________________INA219 GETCURRENT AFTER____________________
  
  float shuntvoltage_after = 0;
  float busvoltage_after[999];
  float current_mA_after[999];

  float current_after_sum = 0;
  float busvoltage_after_sum = 0;

  float current_after_avg = 0;
  float busvoltage_after_avg = 0;
  //Measure current 1000 times
  for (int i=0; i < 1000; i++){
    current_mA_after[i] = ina219.getCurrent_mA();
    current_after_sum = current_after_sum + current_mA_after[i];//Calculate the total sum
  }
  current_after_avg = current_after_sum / 1000;//Calculate current avarage
  //Measure voltage 1000 times
  for (int i=0; i < 1000; i++){
    busvoltage_after[i] = ina219.getBusVoltage_V();
    busvoltage_after_sum = busvoltage_after_sum + busvoltage_after[i];//Calculate the total sum
  }
  busvoltage_after_avg = busvoltage_after_sum / 1000;//Calculate voltage avarage
  float loadvoltage_after = 0;
  float power_mW_after = 0;
  float current_difference = 0;
  float voltage_difference = 0;

  shuntvoltage_after = ina219.getShuntVoltage_mV();
  power_mW_after = ina219.getPower_mW();
  loadvoltage_after = busvoltage_after_avg + (shuntvoltage_after / 1000);
  current_difference=current_avg-current_after_avg;
  voltage_difference=busvoltage_avg-busvoltage_after_avg;
  Serial.println("Current after: ");
  Serial.println(current_after_avg);
  Serial.println("Bus Voltage after: ");
  Serial.println(busvoltage_after_avg);  
  //Send values to Adafruit IO feeds
  Current->save(current_avg);
  Voltage->save(busvoltage_avg);
  Current_after->save(current_after_avg);
  Voltage_after->save(busvoltage_after_avg);
  Current_difference->save(current_difference);
  Voltage_difference->save(voltage_difference);
  delay(1000);
//_______________END OF INA 219 GET CURRENT AFTER_______________


  // Serial.setDebugOutput(true);
  time2sleep = wakeup_time_usec; //reset sleep time
  esp_deep_sleep_enable_timer_wakeup(time2sleep);//10 minutes between send readings wake ups
  gettimeofday(&sleep_enter_time, NULL);//store on RTC at what time you enter sleep
  //WiFi.disconnect();
  Serial.println("Going to sleep...");
  esp_deep_sleep_start();//Go to sleep
}
void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
  float T, JD_frac, L0, M, e, C, L_true, f, R, GrHrAngle, Obl, RA, Decl, HrAngle;
  long JD, JDx;
  int   zone = 0;  //Unused variable but retained for continuity 
  JD      = JulianDate(year, month, day);
  JD_frac = (hour + minute / 60. + second / 3600.0) / 24.0 - 0.5;
  T          = JD - 2451545; T = (T + JD_frac) / 36525.0;
  L0         = DEG_TO_RAD * fmod(280.46645 + 36000.76983 * T, 360);
  M          = DEG_TO_RAD * fmod(357.5291 + 35999.0503 * T, 360);
  e          = 0.016708617 - 0.000042037 * T;
  C          = DEG_TO_RAD * ((1.9146 - 0.004847 * T) * sin(M) + (0.019993 - 0.000101 * T) * sin(2 * M) + 0.00029 * sin(3 * M));
  f          = M + C;
  Obl        = DEG_TO_RAD * (23 + 26 / 60.0 + 21.448 / 3600. - 46.815 / 3600 * T);
  JDx        = JD - 2451545;
  GrHrAngle  = 280.46061837 + (360 * JDx) % 360 + 0.98564736629 * JDx + 360.98564736629 * JD_frac;
  GrHrAngle  = fmod(GrHrAngle, 360.0);
  L_true     = fmod(C + L0, 2 * PI);
  R          = 1.000001018 * (1 - e * e) / (1 + e * cos(f));
  RA         = atan2(sin(L_true) * cos(Obl), cos(L_true));
  Decl       = asin(sin(Obl) * sin(L_true));
  HrAngle    = DEG_TO_RAD * GrHrAngle + Lon - RA;
  elevation  = asin(sin(Lat) * sin(Decl) + cos(Lat) * (cos(Decl) * cos(HrAngle)));
  azimuth    = PI + atan2(sin(HrAngle), cos(HrAngle) * sin(Lat) - tan(Decl) * cos(Lat)); // Azimuth measured east from north, so 0 degrees is North
  sun_azimuth   = azimuth   / DEG_TO_RAD;
  sun_elevation = elevation / DEG_TO_RAD;
  Serial.println("Longitude and latitude " + String(Lon / DEG_TO_RAD, 3) + " " + String(Lat / DEG_TO_RAD, 3));
  Serial.println("Year\tMonth\tDay\tHour\tMinute\tSecond\tElevation\tAzimuth");
  Serial.print(String(year) + "\t" + String(month) + "\t" + String(day) + "\t" + String(hour - zone) + "\t" + String(minute) + "\t" + String(second) + "\t");
  Serial.println(String(elevation / DEG_TO_RAD, 0) + "\t\t" + String(azimuth / DEG_TO_RAD, 0));
}

long JulianDate(int year, int month, int day) {
  long JD;
  int A, B;
  if (month <= 2) {year--; month += 12;}
  A = year / 100; B = 2 - A + A / 4;
  JD = (long)(365.25 * (year + 4716)) + (int)(30.6001 * (month + 1)) + day + B - 1524;
  return JD;
}
 
////////////// WiFi, Time and Date Functions /////////////////
int StartWiFi(const char* ssid, const char* password) {
  int connAttempts = 0;
  Serial.print(F("\r\nConnecting to: ")); Serial.println(String(ssid));
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED ) {
    delay(500); Serial.print(".");
    if (connAttempts > 20) {
      Serial.println("\nFailed to connect to a Wi-Fi network");
      return false;
    }
    connAttempts++;
  }
  Serial.print(F("WiFi connected at: "));
  Serial.println(WiFi.localIP());
  return true;
}

void StartTime() {
  configTime(0, 0, "0.uk.pool.ntp.org", "time.nist.gov");
  setenv("TZ", "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00", 1); // Change for your location
  UpdateLocalTime();
}

void UpdateLocalTime() {
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
  }
  //See http://www.cplusplus.com/reference/ctime/strftime/
  Serial.println(&timeinfo, "%a %b %d %Y   %H:%M:%S"); // Displays: Saturday, June 24 2017 14:05:49
  char output[50];
  strftime(output, 50, "%a %d-%b-%y  (%H:%M:%S)", &timeinfo);
  time_str = output;
}

String GetTime() {
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time - trying again");
  }
  //See http://www.cplusplus.com/reference/ctime/strftime/
  //Serial.println(&timeinfo, "%a %b %d %Y %H:%M:%S"); // Displays: Saturday, June 24 2017 14:05:49
  char output[50];
  strftime(output, 50, "%d/%m/%y %H:%M:%S", &timeinfo); //Use %m/%d/%y for USA format
  time_str = output;
  Serial.println(time_str);
  return time_str; // returns date-time formatted like this "11/12/17 22:01:00"
}

String Update_DateTime() {
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time - trying again");
  }
  //See http://www.cplusplus.com/reference/ctime/strftime/
  char output[50];
  strftime(output, 50, "%H", &timeinfo);
  current_hour   = output;
  strftime(output, 50, "%M", &timeinfo);
  current_minute = output;
  strftime(output, 50, "%d", &timeinfo);
  current_day    = output;
  strftime(output, 50, "%m", &timeinfo);
  current_month  = output;
  strftime(output, 50, "%Y", &timeinfo);
  current_year   = output;
  Serial.println(time_str);
  return time_str; // returns date-time formatted like this "11/12/17 22:01:00"
}

void test_azimuth() {
  pwm.setPWM(servoSide,0,map(90, 0, 180, SERVOMIN, SERVOMAX));  // Centre position
  delay(500);
  pwm.setPWM(servoSide,0,map(60, 0, 180, SERVOMIN, SERVOMAX));  // Centre position
  delay(500);
  pwm.setPWM(servoSide,0,map(120, 0, 180, SERVOMIN, SERVOMAX)); // Centre position
  delay(500);
  pwm.setPWM(servoSide,0,map(90, 0, 180, SERVOMIN, SERVOMAX));  // Centre position
  delay(500);
}

void test_elevation() {
  for (int a = 5; a < 145; a = a + 2) {
    pwm.setPWM(servoUpDown, 0, map(a, 0, 180, SERVOMIN, SERVOMAX));// Centre position
    delay(30);
  }
  pwm.setPWM(servoUpDown, 0, map(145, 0, 180, SERVOMIN, SERVOMAX)); // Centre position
  delay(1000);
}
/*INA219
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina219.begin();
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();
  for (int i = 0; i < filterlen; i++) {
    shuntvoltage += ina219.getShuntVoltage_mV();
    busvoltage += ina219.getBusVoltage_V();
    current_mA += ina219.getCurrent_mA();

    battLevel += (analogRead(A13) / 4096.0) * 2 * 3.3 * 1.1;
  }
  shuntvoltage /= filterlen;
  busvoltage /= filterlen;
  current_mA /= filterlen;
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  battLevel /= filterlen;

}
*/
void loop(){
  gettimeofday(&sleep_enter_time, NULL);//store on RTC at what time you enter sleep

  esp_deep_sleep_start();
  //if published

  //delay(2000); // to avoid throttle on MQTT
}
