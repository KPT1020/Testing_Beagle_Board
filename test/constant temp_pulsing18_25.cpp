#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include "SHTSensor.h"
#include <Wire.h>
#include <SPI.h>
#include <PID_v1.h>
#include <Spiffs.h>
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
SHTSensor sht;

#define colPin_1            27
#define pumpPin_1           26
#define solenoidPin         23

#define buz                 13
#define btn_rst             39
#define btn_1               36
#define BOOT                0
#define sensor_heater       14
#define sensor_heater_pulsing   12
#define SDA_pin             32
#define SCL_pin             33

#define Offset_channel  	2 //raw adc
#define Sensor_channel  	0 //raw adc
#define Heater_channel  	1 //raw adc
#define NTCC_channel      3 //raw adc

#define pumpChannel_1 		0
#define pumpChannel_2 		1
#define colChannel_1 		2
#define colChannel_2 		3

const int freq = 20000;
const int resolution = 8;
double PID_Setpoint = 6000;
double Input, Output;
//metal tube tuning
double consKp=8, consKi= 0.08, consKd= 2;
// double consKp=64, consKi= 2, consKd= 8;
PID myPID(&Input, &Output, &PID_Setpoint, consKp, consKi, consKd, REVERSE);

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Wire.begin(SDA_pin,SCL_pin);
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
  pinMode(pumpPin_1,OUTPUT);
  // pinMode(pumpPin_2,OUTPUT);
  pinMode(colPin_1,OUTPUT);
  pinMode(solenoidPin,OUTPUT);
  // pinMode(colPin_2,OUTPUT);
  pinMode(sensor_heater,OUTPUT);
  pinMode(sensor_heater_pulsing,OUTPUT);
  // pinMode(battery_EN, OUTPUT);
  pinMode(btn_rst, INPUT);
  // pinMode(battery_read,INPUT);

  // digitalWrite(battery_EN,1);           //  enable battery monitor
  digitalWrite(sensor_heater,255);             //  enable senosr heater
  ledcSetup(colChannel_1, 5000, 8);
  ledcSetup(colChannel_2, 5000, 8);
  ledcSetup(pumpChannel_1, freq, resolution);
  ledcSetup(pumpChannel_2, freq, resolution);
  ledcAttachPin(pumpPin_1,pumpChannel_1);
  // ledcAttachPin(pumpPin_2,pumpChannel_2);
  ledcAttachPin(colPin_1,colChannel_1);
  // ledcAttachPin(colPin_2,colChannel_2);
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(consKp, consKi, consKd);

  //while NTCC larger than setpoint,turn on heater and compute PID
  while( ads.readADC_SingleEnded(NTCC_channel)> PID_Setpoint+10){
    Input = ads.readADC_SingleEnded(NTCC_channel);
    myPID.Compute();
    ledcWrite(colChannel_1,Output);
    Serial.print("Input: ");Serial.print(Input);Serial.print(",");
    Serial.print("NTCC: ");Serial.println(ads.readADC_SingleEnded(NTCC_channel));
    delay(10);
  }  
  //turn on the pump
  ledcWrite(pumpChannel_1,150);
  delay(200);
  ledcWrite(pumpChannel_1,100);
  delay(200);
  ledcWrite(pumpChannel_1, 80);

  // digitalWrite(solenoidPin,255);
}
double load_resistance = 47000;
      double input_voltage = 3.3;
      double sensor_resistance = 0;
      double sensor_voltage = 0;
unsigned long previous_time = millis();

void read_value(){
  Input = ads.readADC_SingleEnded(NTCC_channel);
  myPID.Compute();
  ledcWrite(colChannel_1,Output); 
  if(sht.readSample()){
    Serial.print(sht.getHumidity(), 2);Serial.print(",");
    Serial.print(sht.getTemperature(), 2);Serial.print(",");
    Serial.print(ads.readADC_SingleEnded(Sensor_channel));Serial.print(",");
    sensor_voltage = ads.computeVolts(ads.readADC_SingleEnded(Sensor_channel));
    sensor_resistance = ((load_resistance * input_voltage)/sensor_voltage) - load_resistance;
    Serial.print(sensor_resistance);Serial.print(",");
    Serial.print(ads.readADC_SingleEnded(Heater_channel));Serial.print(",");
    Serial.print(ads.readADC_SingleEnded(Offset_channel));Serial.print(",");
    Serial.print(Output);Serial.print(",");
    Serial.println(ads.readADC_SingleEnded(NTCC_channel));
  }
}

void loop(){
//continuously read channel from ads and sht40, swtich on heater channel for 3 minutes and switch it off. Then switch on heater pusling channel for 10 seconds, repeat the cycle
  digitalWrite(sensor_heater,255);
  digitalWrite(sensor_heater_pulsing,0);
  Serial.println("heating sensor");
  previous_time = millis();
  while(millis()-previous_time < 120000){
    read_value();
  }
  digitalWrite(solenoidPin,255);
  Serial.println("open");
  previous_time = millis();
  while(millis()-previous_time < 3000){
    read_value();
  }
  digitalWrite(solenoidPin,0);
  Serial.println("close");
  previous_time = millis();
  while(millis()-previous_time < 180000){
    read_value();
  }
  digitalWrite(sensor_heater,0);
  digitalWrite(sensor_heater_pulsing,255);
  Serial.println("Pulsimg");
  previous_time = millis();
  while(millis() - previous_time < 10000){
    read_value();
  }
  digitalWrite(sensor_heater,0);
  digitalWrite(sensor_heater_pulsing,0);
  Serial.println("waiting");
  previous_time = millis();
  while(millis() - previous_time < 60000){
    read_value();
  }
}