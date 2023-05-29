#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include "SHTSensor.h"
#include <Wire.h>
#include <SPI.h>
#include <PID_v1.h>
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
  pinMode(pumpPin_2,OUTPUT);
  pinMode(colPin_1,OUTPUT);
  pinMode(colPin_2,OUTPUT);
  pinMode(sensor_heater,OUTPUT);
  pinMode(battery_EN, OUTPUT);
  pinMode(btn_rst, INPUT);
  pinMode(battery_read,INPUT);

  digitalWrite(battery_EN,1);           //  enable battery monitor
  dacWrite(sensor_heater,0);             //  enable senosr heater
  ledcSetup(colChannel_1, 5000, 8);
  ledcSetup(colChannel_2, 5000, 8);
  ledcSetup(pumpChannel_1, freq, resolution);
  ledcSetup(pumpChannel_2, freq, resolution);
  ledcAttachPin(pumpPin_1,pumpChannel_1);
  ledcAttachPin(pumpPin_2,pumpChannel_2);
  ledcAttachPin(colPin_1,colChannel_1);
  ledcAttachPin(colPin_2,colChannel_2);
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(consKp, consKi, consKd);

  while(ads.readADC_SingleEnded(NTCC_channel)>PID_Setpoint){
    Input = ads.readADC_SingleEnded(NTCC_channel);
    myPID.Compute();
    ledcWrite(colChannel_1,Output); //220
    Serial.println(ads.readADC_SingleEnded(NTCC_channel));
  }
  dacWrite(sensor_heater,0);//turn off sensor
  ledcWrite(pumpChannel_1,150);
  delay(200);
  ledcWrite(pumpChannel_1,100);
  delay(200);
  ledcWrite(pumpChannel_1, 120);
}

unsigned long previousmillis = millis();
double load_resistance = 47000;
      double input_voltage = 3.3;
      double sensor_resistance = 0;
      double sensor_voltage = 0;
void loop(){
    dacWrite(sensor_heater,255);Serial.println("heater on");
    for(int i=0;i<2000;i++){
        Input = ads.readADC_SingleEnded(NTCC_channel);
        myPID.Compute();
        ledcWrite(colChannel_1,Output); //220
        Serial.println(ads.readADC_SingleEnded(Sensor_channel));
        delay(10);
    }
    for (int i = 0; i < 3; i++)
    {
        ledcWrite(pumpChannel_1, 255);
        delay(500);
        ledcWrite(pumpChannel_1, 0);
        delay(500);
    }
    ledcWrite(pumpChannel_1,150);
    delay(200);
    ledcWrite(pumpChannel_1,100);
    delay(200);
    ledcWrite(pumpChannel_1, 120);

    for(int i=0;i<2000;i++){
        Input = ads.readADC_SingleEnded(NTCC_channel);
        myPID.Compute();
        ledcWrite(colChannel_1,Output); //220
        Serial.println(ads.readADC_SingleEnded(Sensor_channel));
        delay(10);
    }
    
    dacWrite(sensor_heater,0);Serial.println("heater off");
    for(int i=0;i<2000;i++){
        Input = ads.readADC_SingleEnded(NTCC_channel);
        myPID.Compute();
        ledcWrite(colChannel_1,Output); //220
        Serial.println(ads.readADC_SingleEnded(Sensor_channel));
        delay(10);
    }
}