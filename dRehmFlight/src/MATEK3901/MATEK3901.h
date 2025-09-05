#pragma once
#include <Arduino.h>
#include "MSPv2.h"

/**
 * Matek 3901-L0X adapter:
 *  - Watches incoming MSPv2 frames
 *  - Extracts flow_x (int32), flow_y (int32), range_mm (uint32) when the function ID matches
 *  - Provides simple low-pass filtering and a "freshness" flag
 */
class Matek3901 {
public:
  // Basic Constructor
  Matek3901();

  // Need to set up a the Serial port that the UART is connected to
  void begin(HardwareSerial & port, uint32_t baud);
  // Check to see if there's a change in action and "recall" the onframe function
  void poll();

  // ------------- Getter Functions -------------- //
  // Latest values (raw, unscaled)
  bool getFresh() const;
  int32_t getFlowX() const;
  int32_t getFlowY() const;
  uint32_t getRangeMM() const;
  uint32_t getLastUpdateMs() const;

  // Low Pass Filter Getters
  float getLPFFlowX() const;
  float getLPFFlowY() const;
  float getLPFRangeM() const;
  
  // ------------- Setter Functions -------------- //
  void setLPFAlpha(float a);

  // For debugging/bring-up
  void enableLogging(Stream * s);

private:
  HardwareSerial * serial = nullptr;
  MSPv2Parser parser{[&](const MSPv2Frame& f){  }};
  Stream * log = nullptr;
  
  // Neccessary IDs for the 2 chips
  const uint16_t RANGEFINDER_FUNC_ID = 0x1F01;
  const uint16_t OPFLOW_FUNC_ID = 0x1F02;

  // Raw values to extract
  int32_t  flow_x = 0;
  int32_t  flow_y = 0;
  uint32_t range_mm = 0;

  // Filter values to use
  float alpha = 0.5f;
  float lpf_flow_x = 0.0f;
  float lpf_flow_y = 0.0f;
  float lpf_range_mm = 0.0f;

  uint32_t last_ms = 0;
  bool fresh = false;

  void onFrame(const MSPv2Frame & f);

  // Parsing Functions to Break up the Stream
  void parseFlow(const  MSPv2Frame & f);
  void parseRange(const MSPv2Frame & f);

  // Filter the values for usable data
  void commitRange(uint32_t range_mm, uint8_t quality);
  void commitFlow(int32_t fx, int32_t fy, uint8_t quality);

  void printStream(const MSPv2Frame & f);
};  // End of Matek 3901 Class
