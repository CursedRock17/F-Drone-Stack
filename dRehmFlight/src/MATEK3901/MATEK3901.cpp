#include "MATEK3901.h"

// Basic Constructor
Matek3901::Matek3901()
{}

// Need to set up a the Serial port that the UART is connected to
void Matek3901::begin(HardwareSerial & port, uint32_t baud = 115200)
{
  serial = &port;
  serial->begin(baud);
  parser.setHandler([this](const MSPv2Frame & f) {
    this->onFrame(f);
  });
}

// Check to see if there's a change in action and "recall" the onframe function
void Matek3901::poll()
{
  while (serial && serial->available() > 0) {
    parser.feed(static_cast<uint8_t>(serial->read()));
  }
}

// ------------- Getter Functions -------------- //
bool     Matek3901::getFresh() const { return fresh; }
int32_t  Matek3901::getFlowX() const { return flow_x; }     // typical units: counts or rad/s * scale
int32_t  Matek3901::getFlowY() const { return flow_y; }
uint32_t Matek3901::getRangeMM() const { return range_mm; }
uint32_t Matek3901::getLastUpdateMs() const { return last_ms; }

// LPF Getter Functions
float Matek3901::getLPFFlowX() const { return lpf_flow_x; }
float Matek3901::getLPFFlowY() const { return lpf_flow_y; }
float Matek3901::getLPFRangeM() const { return lpf_range_mm * 0.001f; }

// ------------- Setter Functions -------------- //
// Simple one-pole LPF (alpha in [0..1], 0 = no update, 1 = no filtering)
void Matek3901::setLPFAlpha(float a)
{ 
  alpha = constrain(a, 0.0f, 1.0f);
}

// For debugging/bring-up
void Matek3901::enableLogging(Stream * s)
{
  log = s;
}

void Matek3901::onFrame(const MSPv2Frame & f)
{
  // We have the function IDs for each of the two sensors on board
  // So match it to a corresponding parse function
  if (f.function == OPFLOW_FUNC_ID) {
    parseFlow(f);
  } else if (f.function == RANGEFINDER_FUNC_ID) {
    parseRange(f);
  } else {
    printStream(f); 
  }
}

void Matek3901::parseFlow(const MSPv2Frame & f)
{
  // Range has size of 9 anything more is the wrong type
  if (f.size > 9) {
    if (log) {
      log->print(F("Wrong Optical Flow Payload Size: ")); log->println(f.size);
    }
    return;
  }
  // Optical Flow Stream is built as [1 Byte][4 Byte][4 Byte]
  //                              quality | motion_x | motion_y
  
  // Transform Hexadecimal to int32_t skipping quality for X
  int32_t fx = int32_t(uint32_t(f.payload[1]) | (uint32_t(f.payload[2]) << 8) |
                     (uint32_t(f.payload[3]) << 16) | (uint32_t(f.payload[4]) << 24));
  // Transform Hexadecimal to int32_t skipping quality & motion_x for Y
  int32_t fy = int32_t(uint32_t(f.payload[5]) | (uint32_t(f.payload[6]) << 8) |
                     (uint32_t(f.payload[7]) << 16) | (uint32_t(f.payload[8]) << 24));

  commitFlow(fx, fy, uint8_t(f.payload[0]));
  if (log) {
    log->print(F("[MSP] (auto) parsed flowX= ")); log->print(fx);
    log->print(F(" flowY= ")); log->println(fy);
  }
}

void Matek3901::parseRange(const MSPv2Frame & f)
{
  // Rangefinder has a stream size of 5, anything more is the wrong type
  if (f.size > 5) {
    if (log) {
      log->print(F("Wrong Rangefinder Payload Size: ")); 
      log->println(f.size);
    }
    return;
  }
  // Range is built as [1 Byte][4 Byte]
  //                  quality | distance_mm
  // Transform Hexadecimal to Real number int32_t skipping quality
  int32_t r = int32_t(uint32_t(f.payload[1]) | (uint32_t(f.payload[2]) << 8) |
                     (uint32_t(f.payload[3]) << 16) | (uint32_t(f.payload[4]) << 24));

  // Basic sanity: range within a plausible 0.08m -> 2m window
  if (r >= 80 && r <= 2000) {
    commitRange(r, uint8_t(f.payload[0]));
    if (log) {
      log->print(F("[MSP] (auto) parsed range_mm: ")); 
      log->println(r);
    }
    return;
  }
}

void Matek3901::commitRange(uint32_t range_mm_, uint8_t quality) 
{
  // Setup up Raw Value
  range_mm = range_mm_;
  // Low Pass Filter 
  lpf_range_mm = alpha * float(range_mm) + (1.0f - alpha) * lpf_range_mm;
  last_ms = millis();
  if (quality > 0) {
    fresh = true;
  }
}

void Matek3901::commitFlow(int32_t fx, int32_t fy, uint8_t quality)
{
  // Setup up Raw Values
  flow_x = fx; 
  flow_y = fy;
  // Low Pass Filter 
  lpf_flow_x = alpha * float(fx) + (1.0f - alpha) * lpf_flow_x;
  lpf_flow_y = alpha * float(fy) + (1.0f - alpha) * lpf_flow_y;
  last_ms = millis();
  if (quality > 0) {
    fresh = true;
  }
}

void Matek3901::printStream(const MSPv2Frame & f)
{
  if (log) {
    log->print(F("[MSP] dir=")); log->print(f.dir);
    log->print(F(" flags=")); log->print(f.flags, HEX);
    log->print(F(" func=0x")); log->print(f.function, HEX);
    log->print(F(" size=")); log->print(f.size);
    log->print(F(" payload="));
    for (auto b : f.payload) { 
      if (b < 16) 
        log->print('0'); 
      log->print(b, HEX); 
      log->print(' '); 
    }
    log->println();
  }
}
