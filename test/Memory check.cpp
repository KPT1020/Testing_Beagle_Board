#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include "SHTSensor.h"
#include <Wire.h>
#include <SPI.h>
#include <PID_v1.h>
#include <Spiffs.h>
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
SHTSensor sht;

#define colPin_1            26
#define colPin_2            13
#define pumpPin_1           27
#define pumpPin_2           32
 
#define buz                 13
#define btn_rst             39
#define btn_1               36
#define BOOT                0
#define sensor_heater       25
#define battery_EN          12
#define battery_read        34

#define Offset_channel  	0 //raw adc
#define Sensor_channel  	1 //raw adc
#define Heater_channel  	2 //raw adc
#define NTCC_channel      3 //raw adc

#define pumpChannel_1 		0
#define pumpChannel_2 		1
#define colChannel_1 		2
#define colChannel_2 		3

const int freq = 20000;
const int resolution = 8;
double PID_Setpoint = 6000;
double Input, Output;
double consKp=64, consKi= 2, consKd= 8;
PID myPID(&Input, &Output, &PID_Setpoint, consKp, consKi, consKd, REVERSE);

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Wire.begin();
  delay(1000); // let serial console settle
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
    if (sht.init()) {
      Serial.print("init(): success\n");
  } else {
      Serial.print("init(): failed\n");
  }
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x
  //clean SPIFFS file
  SPIFFS.format();
}  

int8_t counter = 1;
uint16_t data[2200] = {0};

void loop(){
  for(int i=0;i<2200;i++){
    //write random number to data
    data[i] = random(0,4096);
  }
  //Store the data into a Spiff file
  String filename = "/data.txt";
  filename.concat(String(counter));
  File file = SPIFFS.open(filename.c_str(), FILE_WRITE);
  Serial.print("Storing into: ");Serial.println(filename);
  if(!file){
    Serial.println("There was an error opening the file for writing");
    return;
  }
  for(int i=0;i<2200;i++){
    file.print(data[i]);file.print(',');file.write('\n');
  }
  //print the file size,used spiffs space and total spiffs space
  Serial.print("file size: ");Serial.println(file.size());
  Serial.print("used size: ");Serial.println(SPIFFS.usedBytes());
  Serial.print("total size: ");Serial.println(SPIFFS.totalBytes());
  file.close();
  counter ++;
  delay(1000);
   
}