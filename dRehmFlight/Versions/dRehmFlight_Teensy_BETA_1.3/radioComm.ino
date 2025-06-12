//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 7/29/2022
//Version: Beta 1.3

//========================================================================================================================//

//This file contains all necessary functions and code used for radio communication to avoid cluttering the main code

unsigned long rising_edge_start[6];
unsigned long channel_raw[6];
int currentPin = 0;
int ppm_counter = 0;
unsigned long time_ms = 0;

void radioSetup() {
  //PPM Receiver 
  #if defined USE_PPM_RX
    //Declare interrupt pin
    pinMode(PPM_Pin, INPUT_PULLUP);
    delay(20);
    //Attach interrupt and point to corresponding ISR function
    attachInterrupt(digitalPinToInterrupt(PPM_Pin), getPPM, CHANGE);

  // PWM Receiver
  #elif defined USE_PWM_RX
    // Declare interrupt pins 
    for (int i = 0; i < 6; i++) {
      pinMode(channelPins[i], INPUT_PULLUP);
    }
    delay(20);

    // Attach interrupt and point to corresponding ISR functions
    for (int i = 0; i < 6; i++) {
      currentPin = i;
      attachInterrupt(digitalPinToInterrupt(channelPins[i]), getCh, CHANGE);
    }
    delay(20);

  //SBUS Recevier 
  #elif defined USE_SBUS_RX
    sbus.begin();

  //DSM receiver
  #elif defined USE_DSM_RX
    Serial3.begin(115000);
  #else
    #error No RX type defined...
  #endif
}

unsigned long getRadioPWM(int ch_num) 
{
  // DESCRIPTION: Get current radio commands from interrupt routines 
  unsigned long returnPWM = 0;
  returnPWM = channel_raw[ch_num];
  
  return returnPWM;
}

// For DSM type receivers
void serialEvent3(void)
{
  #if defined USE_DSM_RX
    while (Serial3.available()) {
        DSM.handleSerialEvent(Serial3.read(), micros());
    }
  #endif
}

// ========================================================================================================================//

// INTERRUPT SERVICE ROUTINES (for reading PWM and PPM)
void getPPM() 
{
  unsigned long dt_ppm;
  int trig = digitalRead(PPM_Pin);
  if (trig == 1) { //Only care about rising edge
    dt_ppm = micros() - time_ms;
    time_ms = micros();
    
    // Waiting for long pulse to indicate a new pulse train has arrived
    if (dt_ppm > 5000) {
      ppm_counter = 0;
    }

    // Check pulse for each channel
    for (int i = 0; i < 6; i++) {
      if (ppm_counter = i) {
        channel_raw[i] = dt_ppm;
      }
    }

    ppm_counter = ppm_counter + 1;
  }
}

void getCh()
{
  // Check to see the status of a channel pin
  int trigger = digitalRead(channelPins[currentPin]); 
  if (trigger) {
    rising_edge_start[currentPin] = micros();
  } else if (!trigger) {
    channel_raw[currentPin] = micros() - rising_edge_start[currentPin];
  }
}
