#include "FSM.h"
#include "defines.h"

void set_state(fsm_t &fsm, int state_new){
  if(fsm.state != state_new){
    fsm.state = state_new;
    fsm.tis = 0;                  /* tis = time in state    */
    fsm.tes = millis();           /* tes = time enter state */
  } 
}

void Dimming_calc_next_state(fsm_t &fsm, fsm_t& EM, fsm_t& CLAP, float distance_down, float distance_up){
    
    boolean s_down = ((distance_down * 100) < 15);
    boolean s_up = ((distance_up * 100) < 15);


    // Serial.println(s_down);
    // Serial.println(s_up);

    int Brightness = 0;

    if(EM.state == EM_OFF || CLAP.state == OFF){
        fsm.state == IDLE;
    }
    
    else if((fsm.state == IDLE) && (s_down) && (!s_up)){
        fsm.state_new = S_DOWN;
    }
    else if((fsm.state == IDLE) && (s_up) && (!s_down)){
        fsm.state_new = S_UP;
    }
    else if((fsm.state == S_DOWN) && fsm.tis > 500){
        fsm.state_new = DOWN;
    }
    else if((fsm.state == S_DOWN) && ((!s_down) || (s_up))){
        fsm.state_new = IDLE;
    }
    else if((fsm.state == DOWN) && ((!s_down) || (Brightness))){
        fsm.state_new = IDLE;
    }
    else if((fsm.state == S_UP) && fsm.tis > 500){
        fsm.state_new = UP;
    }
    else if((fsm.state == S_UP) && ((!s_up) || (s_down))){
        fsm.state_new = IDLE;
    }
    else if((fsm.state == UP) && ((!s_up) || (Brightness == 255))){
        fsm.state_new = IDLE;
    }
}

int Dimming_calc_outputs(fsm_t& fsm, int brightness){
    int FADE_TIME = 2500;

    if(fsm.state == DOWN){
        brightness -= 4;
    }
    else if(fsm.state == UP){
        brightness += 4;
    }

    /* "Anti Windup" */
    if(brightness < 0){
        brightness = 0;
    }
    else if(brightness > 255){
        brightness = 255;
    }

    return brightness;
}


void Clap_calc_next_state(fsm_t &fsm, fsm_t &EM, int clap){
    if(EM.state == EM_OFF){
        fsm.state_new = OFF;
    }


    else if((fsm.state == OFF) && (clap)){
        fsm.state_new = CLAP_ON;
    }

    else if((fsm.state == CLAP_ON) && (fsm.tis > 300)){
        fsm.state_new = OFF;
    }
    else if((fsm.state == CLAP_ON) && (clap)){
        fsm.state_new = CLAP_ON2;
    }

    else if((fsm.state == CLAP_ON2) && (clap)){
        fsm.state_new = OFF;
    }
    else if((fsm.state == CLAP_ON2) && (fsm.tis > 300)){
        fsm.state_new = ON;
    }

    else if((fsm.state == ON) && (clap)){
        fsm.state_new = CLAP_OFF;
    }

    else if((fsm.state == CLAP_OFF) && (clap)){
        fsm.state_new = CLAP_OFF2;
    }
    else if((fsm.state == CLAP_OFF) && (fsm.tis > 300)){
        fsm.state_new = ON;
    }

    else if((fsm.state == CLAP_OFF2) && (fsm.tis > 300)){
        fsm.state_new = OFF;
    }    
    else if((fsm.state == CLAP_OFF2) && (clap)){
        fsm.state_new = ON;
    }
}


int Clap_calc_outputs(fsm_t& fsm){
    if(fsm.state == ON){
        LED_STATE = 1;
    }
    else if(fsm.state == OFF){
        LED_STATE = 0;
    }

    return 0;
}

void Emergency_calc_next_state(fsm_t &fsm, int EmergencySwitch){
    if((fsm.state == EM_ON) && (EmergencySwitch)){
        fsm.state_new = EM_OFF;
    }
    else if((fsm.state == EM_OFF) && (EmergencySwitch == 0)){
        fsm.state_new = EM_ON;
    }
}

int Emergency_calc_outputs(fsm_t& fsm){
    if(fsm.state == EM_ON){
        LED_STATE = 1;
        return 1;
    }
    else if(fsm.state == EM_OFF){
        LED_STATE = 0;
        return 0;
    }
    return -1;
}

void PWM_calc_next_state(fsm_t &fsm, fsm_t &EM, fsm_t &CLAP, int brightness, int signal){

    int time = 10 * brightness / 255;

    if((fsm.state == 0) && (CLAP.state == EM_ON) && (EM.state == ON) && (signal)){
        fsm.state_new = 1;
    }
    else if((fsm.state == 1) && (CLAP.state != EM_ON) && (EM.state != ON) && (fsm.tis > time)){
        fsm.state_new = 0;
    }
}

int PWM_calc_outputs(fsm_t &fsm){
    if(fsm.state == 0){
        return 0;
    }
    else if(fsm.state == 1){
        return 1;
    }
    return -1;
}


void outputs(){

}