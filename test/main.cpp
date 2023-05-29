#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include "SHTSensor.h"
#include <Wire.h>
#include <SPI.h>

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
SHTSensor sht;

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
  pinMode(25,OUTPUT);
  dacWrite(25,255);   
}
void loop() {
  // put your main code here, to run repeatedly:
  int16_t adc0, adc1, adc2, adc3; 
  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);


  if (sht.readSample()) {
      Serial.print(sht.getHumidity(), 2);Serial.print(",");
      Serial.print(sht.getTemperature(), 2);Serial.print(",");
      Serial.print(adc0);Serial.print(",");
      Serial.print(adc1);Serial.print(",");
      Serial.print(adc2);Serial.print(",");
      Serial.println(adc3);
  } 
}