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
double PID_Setpoint = 8000;
double Input, Output;
double consKp=64, consKi= 2, consKd= 8;
double size =1000.00;
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
}

unsigned long previousmillis = millis();
unsigned long previousstagechange = 0;
bool sensor_heater_state = false;

void loop(){
    if (sht.readSample()) {
        // Serial.print(ads.readADC_SingleEnded(NTCC_channel));Serial.print(",");
        Serial.print(sht.getTemperature());Serial.print(",");
        Serial.println(ads.readADC_SingleEnded(Sensor_channel));
    }
    if(millis()- previousmillis > 120000){   
        previousmillis = millis();      //pulse for 10 seconds
        if(sensor_heater_state == false){
            sensor_heater_state = true;
            dacWrite(sensor_heater,255);
            Serial.println("heater on");
        }
        else{
            // unsigned long previousmillis_1 = millis();
            // while(millis()- previousmillis_1 < 10000){
            //     if(millis()-previousstagechange > 10){          //switch on the sensor for 10ms and off for 10ms
            //         previousstagechange = millis();
            //         if (sensor_heater_state == 0){
            //             sensor_heater_state = 1;
            //             dacWrite(sensor_heater,255);
            //             Serial.println("pulse heater on");
            //         }
            //         else{
            //             sensor_heater_state = 0;
            //             dacWrite(sensor_heater,0);
            //             Serial.println("pulse heater off");
            //         }
            //     }
            //     if (sht.readSample()) {
            //         // Serial.print(ads.readADC_SingleEnded(NTCC_channel));Serial.print(",");
            //         Serial.print(sht.getTemperature());Serial.print(",");
            //         Serial.println(ads.readADC_SingleEnded(Sensor_channel));
            //     }
            // }
            // previousmillis_1 = millis();
            // while(millis()- previousmillis_1 < 110000){
            //     if (sht.readSample()) {
            //         // Serial.print(ads.readADC_SingleEnded(NTCC_channel));Serial.print(",");
            //         Serial.print(sht.getTemperature());Serial.print(",");
            //         Serial.println(ads.readADC_SingleEnded(Sensor_channel));
            //     }
            // }
            sensor_heater_state = false;
            dacWrite(sensor_heater,0);
            Serial.println("heater off");
        }
    }
}
        
        
        
        
        
        // Input = ads.readADC_SingleEnded(3);
        // myPID.Compute();
        // ledcWrite(colChannel_1,Output); //220
        // if (sht.readSample()) {
        //     Serial.print(ads.readADC_SingleEnded(NTCC_channel));Serial.print(",");
        //     Serial.print(sht.getTemperature());Serial.print(",");
        //     Serial.println("0");
        //     if(millis()-previousstagechange > 10){          //switch on the sensor for 10ms and off for 10ms
        //         previousstagechange = millis();
        //         if (sensor_heater_state == 0){
        //             sensor_heater_state = 1;
        //             dacWrite(sensor_heater,255);
        //         }
        //         else{
        //             sensor_heater_state = 0;
        //             dacWrite(sensor_heater,0);
        //         }
        //     }
        // }