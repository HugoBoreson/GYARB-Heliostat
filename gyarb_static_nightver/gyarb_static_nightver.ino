//Include all necessary libraries
#include "esp_deep_sleep.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "config.h"
#include <Adafruit_INA219.h>
//Establish connection to Adafruit IO feeds
AdafruitIO_Feed *Current_static = io.feed("Current_static");
AdafruitIO_Feed *Voltage_static = io.feed("Voltage_static");
String time_str, current_hour, current_minute, current_day, current_month, current_year;
Adafruit_INA219 ina219;
//Save to RTC
static RTC_DATA_ATTR struct timeval sleep_enter_time;
RTC_DATA_ATTR int blinkAcc;
RTC_DATA_ATTR long  time2sleep;
const long wakeup_time_usec = 60 * 15 * 1000000; //200 seconds sleep time
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
  StartTime();
  UpdateLocalTime();
  Serial.print("Update time:");
  Serial.println(Update_DateTime());
  if(current_hour.toInt()>20){//// Dont upload values if its too late
    time2sleep = 60 * 30 * 1000000; //reset sleep time
    esp_deep_sleep_enable_timer_wakeup(time2sleep);//30 minutes between send readings wake ups
    gettimeofday(&sleep_enter_time, NULL);//store on RTC at what time you enter sleep
    Serial.println("Going to sleep...");
    esp_deep_sleep_start();
  }
  if(current_hour.toInt()<6){//// Dont upload values if its too early
    time2sleep = 60 * 30 * 1000000; //reset sleep time
    esp_deep_sleep_enable_timer_wakeup(time2sleep);//30 minutes between send readings wake ups
    gettimeofday(&sleep_enter_time, NULL);//store on RTC at what time you enter sleep
    Serial.println("Going to sleep...");
    esp_deep_sleep_start();
  }
  else{
    time2sleep = wakeup_time_usec; //15 minutes sleep time
  }
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
  //define variables
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
  current_avg = current_sum / 1000;//Calculate the current avarage
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
  //Send values to Adafruit IO feeds
  Current_static->save(current_avg);
  Voltage_static->save(busvoltage_avg);
  delay(1000);
  // Serial.setDebugOutput(true);
  time2sleep = wakeup_time_usec; //reset sleep time
  esp_deep_sleep_enable_timer_wakeup(time2sleep);//10 minutes between send readings wake ups
  gettimeofday(&sleep_enter_time, NULL);//store on RTC at what time you enter sleep
  //WiFi.disconnect();
  Serial.println("Going to sleep...");
  esp_deep_sleep_start();
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
void StartTime() {
  configTime(0, 0, "0.uk.pool.ntp.org", "time.nist.gov");
  setenv("TZ", "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00", 1); // Change for your location
  UpdateLocalTime();
}
void loop(){
  gettimeofday(&sleep_enter_time, NULL);//store on RTC at what time you enter sleep

  esp_deep_sleep_start();
  //if published

  //delay(2000); // to avoid throttle on MQTT
}
