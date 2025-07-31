//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 7/29/2022
//Version: Beta 1.3

//==========================================================================//

// This file contains all necessary functions and code used for radio
// communication to avoid cluttering the main code

unsigned long rising_edge_start[6];
unsigned long channel_raw[6];
int currentPin = 0;
int ppm_counter = 0;
unsigned long time_ms = 0;

void radioSetup() {
  //PPM Receiver
  #if defined USE_PPM_RX
    // Declare interrupt pin
    pinMode(PPM_Pin, INPUT_PULLUP);
    delay(20);
    // Attach interrupt and point to corresponding ISR function
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

  // CRSF Receiver - Prefers 115,200 or 400,000 Baud Rate
  #elif defined USE_CRSF_RX
    Serial5.begin(400000);
    crsf = new CRSFforArduino(&Serial3);
    if (!crsf->begin())
    {
      crsf->end();
      delete crsf;
      crsf = nullptr;
      Serial.println("CRSF Instance Failed");
      // Repeat Until We Find Connection, this works as a callback
      while (1)
        delay(10);
    }

    // CRSF Library is based on callbacks that are called within "update()"
    crsf->setRcChannelsCallback(prepareChannelsCallback);
    Serial.println("CRSF Instance Ready");

  // SBUS Recevier
  #elif defined USE_SBUS_RX
    sbus.begin();

  // DSM receiver
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
    while (Serial5.available()) {
        DSM.handleSerialEvent(Serial5.read(), micros());
    }
  #endif
}

// ==========================================================================//

// INTERRUPT SERVICE ROUTINES (for reading PWM and PPM)
void getPPM()
{
  unsigned long dt_ppm;
  int trigger = digitalRead(PPM_Pin);
  if (trigger) {  // Only care about rising edge
    dt_ppm = micros() - time_ms;
    time_ms = micros();

    // Waiting for long pulse to indicate a new pulse train has arrived
    if (dt_ppm > 5000) {
      ppm_counter = 0;
    }

    // Check pulse for each channel
    for (int i = 0; i < 6; i++) {
      if (ppm_counter == i) {
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

// CRSF Functions
void prepareChannelsCallback(serialReceiverLayer::rcChannels_t * rcChannels)
{
  // If we don't have the active failsafe values, we can proceed with printing
  if (rcChannels->failsafe == false)
  {
    unsigned long currentTime = millis();
    static unsigned long lastTime = millis();
    if (currentTime < lastTime)
    {
      lastTime = currentTime;
    }
    if (currentTime - lastTime >= 100)
    {
      lastTime = currentTime;
      // RC Channels are on 1-based index - Print the Values
      for (int i = 1; i <= crsfChannels; i++)
      {
        channel_pwm[i - 1] = crsf->getChannel(i);
        // TODO: Fix Arm Channel
        if (i == 5){
          channel_pwm[i - 1] += 809;
        }
        
        /*
        Serial.print(" Ch");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(channel_pwm[i - 1]);
        */
      }
      //Serial.println("");
    }
  }
}
