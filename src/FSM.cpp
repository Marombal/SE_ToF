#include "FSM.h"
#include "defines.h"

void set_state(fsm_t &fsm, int state_new){
  if(fsm.state != state_new){
    fsm.state = state_new;
    fsm.tis = 0;                  /* tis = time in state    */
    fsm.tes = millis();           /* tes = time enter state */
  } 
}

void Dimming_calc_next_state(fsm_t &fsm, float distance_down, float distance_up){
    
    boolean s_down = ((distance_down * 100) < 15);
    boolean s_up = ((distance_up * 100) < 15);


    Serial.println(s_down);
    Serial.println(s_up);

    int Brightness = 0;
    
    if((fsm.state == IDLE) && (s_down) && (!s_up)){
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



void outputs(){

}