#ifndef FSM
#define FSM

#include <Arduino.h>
#include <WiFi.h>
#include "pico/cyw43_arch.h"
#include <Wire.h>
#include <VL53L0X.h>

typedef struct{
  int state, state_new;
  unsigned long tes, tis;
} fsm_t;

/* Geral propose (used for all State Machines) */ 
void set_state(fsm_t &fsm, int state_new);
void outputs();

/* Diming State Machine */ 
void Dimming_calc_next_state(fsm_t &fsm, fsm_t &EM, fsm_t &CLAP, float distance_down, float distance_up);
int Dimming_calc_outputs(fsm_t& fsm, int brightness);

/* Clap State Machine */ 
void Clap_calc_next_state(fsm_t &fsm, fsm_t &EM, int clap);
int Clap_calc_outputs(fsm_t& fsm);

/* Emergency State Machine */ 
void Emergency_calc_next_state(fsm_t &fsm, int EmergencySwitch);
int Emergency_calc_outputs(fsm_t& fsm);


#endif