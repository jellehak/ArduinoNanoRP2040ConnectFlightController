/*========================================================================================================================
This file contains all necessary functions and code used for radio communication to avoid cluttering the main code

Each USE_RCIN_xxx section in this file defines:
rcin_Setup() -> init
rcin_GetPWM(int *pwm) -> fills pwm[0..RCIN_NUM_CHANNELS-1] received PWM values, returns true if new data was received

Uses: rc, HW_PIN_RCIN_RX, RCIN_NUM_CHANNELS
========================================================================================================================*/
// #define USE_RCIN_CRSF
#define USE_RCIN_SBUS
#define RCIN_NUM_CHANNELS  7 //number of receiver channels (minimal 6)

HardwareSerial *rc = &Serial1;

#ifdef USE_RCIN_CRSF

//========================================================================================================================
//CRSF Receiver 
//========================================================================================================================
#include "rcin/crsf/crsf.h"
CRSF crsf;

// int rcin_pwm[RCIN_NUM_CHANNELS]; //filtered raw PWM values

uint16_t *pwm;

unsigned long getRadioPWM(int ch) {
  return pwm[ch];
}

void radioSetup() {
  Serial.println("USE_RCIN_CRSF");
  pwm = crsf.channel;
  rc->begin(CRSF_BAUD);
}

bool rcUpdate() {
    bool rv = false;
    while(rc->available()) {
        int c = rc->read();
        //print received data
        //if(c == CRSF_ADDRESS_FLIGHT_CONTROLLER) Serial.printf("\nreceived: "); Serial.printf("%02x ",c);
        if(crsf.update(c)) {
            //print decoded rc data
            //Serial.print(" decoded RC: "); for(int i=0;i<16;i++) Serial.printf("%d:%d ",i,crsf.channel[i]); Serial.println();
            rv = true;
        }
    }
    
    return rv;
}
#endif

// ======
// 

#ifdef USE_RCIN_PWM

//variables for reading PWM from the radio receiver
unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6;
unsigned long channel_1_raw = 0;
unsigned long channel_2_raw = 0;
unsigned long channel_3_raw = 0;
unsigned long channel_4_raw = 0;
unsigned long channel_5_raw = 0;

int ppm_counter = 0;
unsigned long time_ms = 0;

void radioSetup() {
  //PWM Receiver
  //Declare interrupt pins
  pinMode(throttlePin, INPUT_PULLUP);
  pinMode(rollPin, INPUT_PULLUP);
  pinMode(upDownPin, INPUT_PULLUP);
  pinMode(ruddPin, INPUT_PULLUP);
  pinMode(throttleCutSwitchPin, INPUT_PULLUP);

  delay(20);
  //Attach interrupt and point to corresponding ISR functions
  attachInterrupt(digitalPinToInterrupt(throttlePin), getCh1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rollPin), getCh2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(upDownPin), getCh3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ruddPin), getCh4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(throttleCutSwitchPin), getCh5, CHANGE);

  delay(20);
}

bool rcUpdate() {
    // Nothing, uses interrupts
}

unsigned long getRadioPWM(int ch_num) {
  //DESCRIPTION: Get current radio commands from interrupt routines
  unsigned long returnPWM = 0;

  if (ch_num == 1) {
    returnPWM = channel_1_raw;
  } else if (ch_num == 2) {
    returnPWM = channel_2_raw;
  } else if (ch_num == 3) {
    returnPWM = channel_3_raw;
  } else if (ch_num == 4) {
    returnPWM = channel_4_raw;
  } else if (ch_num == 5) {
    returnPWM = channel_5_raw;
  }
  return returnPWM;
}

//========================================================================================================================//

//INTERRUPT SERVICE ROUTINES (for reading PWM and PPM)

void getCh1() {
  int trigger = digitalRead(throttlePin);
  if (trigger == 1) {
    rising_edge_start_1 = micros();
  } else if (trigger == 0) {
    channel_1_raw = micros() - rising_edge_start_1;
  }
}

void getCh2() {
  int trigger = digitalRead(rollPin);
  if (trigger == 1) {
    rising_edge_start_2 = micros();
  } else if (trigger == 0) {
    channel_2_raw = micros() - rising_edge_start_2;
  }
}

void getCh3() {
  int trigger = digitalRead(upDownPin);
  if (trigger == 1) {
    rising_edge_start_3 = micros();
  } else if (trigger == 0) {
    channel_3_raw = micros() - rising_edge_start_3;
  }
}

void getCh4() {
  int trigger = digitalRead(ruddPin);
  if (trigger == 1) {
    rising_edge_start_4 = micros();
  } else if (trigger == 0) {
    channel_4_raw = micros() - rising_edge_start_4;
  }
}

void getCh5() {
  int trigger = digitalRead(throttleCutSwitchPin);
  if (trigger == 1) {
    rising_edge_start_5 = micros();
  } else if (trigger == 0) {
    channel_5_raw = micros() - rising_edge_start_5;
  }
}

#endif

//========================================================================================================================
//SBUS Receiver 
//========================================================================================================================
#if defined USE_RCIN_SBUS
#warning "USE_RX_SBUS not ported/tested - see src/RCIN/RCIN.h" //TODO

#include "rcin/sbus/SBUS.cpp" //sBus interface

SBUS sbus(*rc);
uint16_t sbusChannels[16];
bool sbusFailSafe;
bool sbusLostFrame;

uint16_t *pwm;

unsigned long getRadioPWM(int ch) {
  return pwm[ch];
}

void radioSetup() {
  Serial.println("USE_RX_SBUS");
  sbus.begin();
}

bool rcUpdate() {
    if (sbus.read(sbusChannels, &sbusFailSafe, &sbusLostFrame))
    {
        //sBus scaling below is for Taranis-Plus and X4R-SB
        float scale = 0.615;
        float bias  = 895.0;
        for(int i=0;i<RCIN_NUM_CHANNELS;i++) {
            pwm[i] = sbusChannels[i] * scale + bias;
        }
        return true;
    }
    return false;
}

#endif