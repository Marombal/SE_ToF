#include <Arduino.h>
#include <WiFi.h>
#include "pico/cyw43_arch.h"
#include <Wire.h>
#include <VL53L0X.h>

#include "FSM.h"
#include "defines.h"



#define ADDRESS1 0x31
#define ADDRESS2 0x36
#define VDown 15
#define LED 10

#define ADC_Pin 28



VL53L0X ToF1;
VL53L0X ToF2;

float distance, prev_distance;

float distance2, prev_distance2;

int LED_state;
unsigned long interval;
unsigned long currentMicros, previousMicros;
int loop_count;

fsm_t Dimming;
fsm_t Clap;
fsm_t Emergency;

void setup() 
{

  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  pinMode(LED, OUTPUT);
  
  pinMode(ADC_Pin, INPUT);

  delay(1000);
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  digitalWrite(VDown, HIGH);

  interval = 40 * 1000;

  Serial.begin(115200);

  Wire.setSDA(12);
  Wire.setSCL(13);  

  Wire.begin();

  digitalWrite(XSHUT1, HIGH); 

  ToF1.setTimeout(500);
  while (!ToF1.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X (ToF1)!"));
    delay(100);
  }  
  Serial.println(F("ToF1. Success initialize VL53L0X!"));
  Serial.println(F("Changing address to 0x31"));

  ToF1.setAddress(ADDRESS1);

  delay(500);

  digitalWrite(XSHUT2, HIGH);


  ToF2.setTimeout(500);
  while (!ToF2.init()){
    Serial.println(F("Failed to detect and initialize VL53L0X! (ToF2)!"));
    delay(100);
  }

  ToF2.setAddress(ADDRESS2);


  // Reduce timing budget to 20 ms (default is about 33 ms)
  //tof.setMeasurementTimingBudget(20000);

  // Start new distance measure
  ToF1.startReadRangeMillimeters();  
  

  ToF2.startReadRangeMillimeters();


  set_state(Dimming, IDLE);
  set_state(Clap, CLAP_ON);
  set_state(Emergency, EM_ON);
  Brightness = 255;
  LED_STATE = 1;
}

#define CYW43_WL_GPIO_LED_PIN 0

void loop() 
{
  currentMicros = micros();
  

  // THE Control Loop (40ms Between loops)
  if (currentMicros - previousMicros >= interval) { 
    previousMicros = currentMicros;
    
    Serial.println(analogRead(ADC_Pin));
    
    // Update timers
    unsigned long cur_time = millis();
    Dimming.tis = cur_time - Dimming.tes;
    Serial.println(Dimming.tis);
   
    // Read Sensors (ToF)
    if (ToF1.readRangeAvailable()) {
      prev_distance = distance;
      distance = ToF1.readRangeMillimeters() * 1e-3;
    }
    
    if (ToF2.readRangeAvailable()){
      prev_distance2 = distance2;
      distance2 = ToF2.readRangeMillimeters() * 1e-3;
    }



    // Start new distance measure
    ToF1.startReadRangeMillimeters(); 
    
    
    ToF2.startReadRangeMillimeters();

    /* calculate next state */
    Dimming_calc_next_state(Dimming, Emergency, Clap, distance, distance2); // distance -> distance_down; distance2 -> distance_up

    /* update state */
    set_state(Dimming, Dimming.state_new);

    /* Actions set by the current state */
    Brightness = Dimming_calc_outputs(Dimming, Brightness);

    /* Outputs */
    outputs();


    /* ToFs*/
    Serial.print(" Dist ToF1: ");    Serial.println(distance*100, 3);
    Serial.print(" Dist ToF2 : ");   Serial.println(distance2*100, 3);
    Serial.print("State: ");  Serial.println(Dimming.state);
    Serial.print("Brightness: ");  Serial.println(Brightness);   
    
    if(LED_STATE == 1){
      analogWrite(LED, Brightness); //14-22
    }
    else{
      analogWrite(LED, 0);
    }
    
  }
}

