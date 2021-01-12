/*
 * Marlowe Modequillo
 * GMIT
 * BENG Software and Electronic Engineering Year 4(Honours)
 * Fitness Tracker Project 
 */
/*
 * References of the Codes
 * 
 * /*****Heart Rate Monitor**********
 * https://github.com/martincornu/pulse-oximeter-arduino
   https://github.com/aromring/MAX30102_by_RF
 * https://docs.rs-online.com/8c0c/A700000006519161.pdf
 * https://www.instructables.com/Pulse-Oximeter-With-Much-Improved-Precision/
 * https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
 * /*********************************
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "algorithm_by_RF.h"
#include "max30102.h"
#include "heartRate.h"

#define DEBUG // Uncomment for debug output to the Serial stream

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

#define HR_MAX 200
#define HR_MIN 40



// Interrupt pin
const byte oxiInt = 4; // pin connected to MAX30102 INT

uint32_t elapsedTime,timeStart;

uint32_t aun_ir_buffer[BUFFER_SIZE]; //infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE];  //red LED sensor data
float old_n_spo2;  // Previous SPO2 value
uint8_t uch_dummy,k;


void setup() {
  uint8_t uch_maxinit_ret = 0;
  pinMode(oxiInt, INPUT);  //pin D4 of the ESP32 connects to the interrupt output pin of the MAX30102

  Wire.begin();

#if defined(DEBUG)
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
#endif




  maxim_max30102_reset(); //resets the MAX30102
  delay(1000);
  maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register
  uch_maxinit_ret = (uint8_t) maxim_max30102_init();  //initialize the MAX30102
 
  old_n_spo2=0.0;

#ifdef DEBUG
  delay(1000);
  uch_dummy=Serial.read();
#endif // DEBUG
  
  timeStart=millis();
}


void loop() {
  HeartBeat();
}

void HeartBeat(){
   float n_spo2,ratio,correl;  //SPO2 value
  int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
  int32_t n_heart_rate; //heart rate value
  int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid
  int32_t i;
     
  //buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
  //read BUFFER_SIZE samples, and determine the signal range
  for(i=0;i<BUFFER_SIZE;i++)
  {
    while(digitalRead(oxiInt)==1);  //wait until the interrupt pin asserts
    maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
  }

  //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
  rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl); 



   long irValue = aun_ir_buffer[i];


   if(checkForBeat(irValue) == true){
    
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 /(delta / 1000.0);

    if(beatsPerMinute < HR_MAX && beatsPerMinute > HR_MIN){
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot &= RATE_SIZE;

      beatAvg = 0;
      for(byte x = 0; x < BUFFER_SIZE ; x++)
      beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
   }
   

   Serial.print("BPM = "); 
   Serial.print("\t\n");
   Serial.print(beatsPerMinute);
   Serial.print("\t\n");
   Serial.print(" Avg BPM = ");
   Serial.print("\t\n"); 
   Serial.print(beatAvg);
   Serial.print("\t\n");
}

//void stepper(){
//  
//}
