#pragma once
#include <Arduino.h>
#include <functional>
#include <stdint.h>
#include <vector>

/**
 * Minimal MSP v2 frame parser (byte-by-byte FSM).
 * Frame layout (over the wire):
 *   '$' 'X' dir  flags  function(LE16)  size(LE16)  payload[size]  crc8_dvb_s2
 * CRC covers: flags, function(2), size(2), payload
 * Direction 'dir' is informational ('<' request, '>' response, '!' error).
 * We accept any dir since some devices use '>' for push telemetry.
 */

struct MSPv2Frame {
  char  dir = '>';
  uint8_t flags = 0;
  uint16_t function = 0;
  uint16_t size = 0;
  std::vector<uint8_t> payload;
  uint8_t crc = 0;
};

class MSPv2Parser {
public:
  // Callback when a full frame is parsed
  using Handler = std::function<void(const MSPv2Frame&)>;

  explicit MSPv2Parser(Handler h) : handler(h) {}

  void setHandler(Handler h) { handler = h; }

  // Feed one byte at a time (e.g., inside serial read loop)
  void feed(uint8_t b) {
    switch (state) {
      case State::WAIT_DOLLAR:
        if (b == '$') state = State::WAIT_X; else reset();
        break;
      case State::WAIT_X:
        if (b == 'X') state = State::READ_DIR; else reset();
        break;
      case State::READ_DIR:
        frame.dir = char(b);
        crc = 0;                      // CRC starts AFTER dir
        state = State::READ_FLAGS;
        break;
      case State::READ_FLAGS:
        frame.flags = b; crc = crc8(crc, b);
        state = State::READ_FUNC_L;
        break;
      case State::READ_FUNC_L:
        func_lo = b; crc = crc8(crc, b);
        state = State::READ_FUNC_H;
        break;
      case State::READ_FUNC_H:
        frame.function = uint16_t(func_lo | (uint16_t(b) << 8));
        crc = crc8(crc, b);
        state = State::READ_SIZE_L;
        break;
      case State::READ_SIZE_L:
        size_lo = b; crc = crc8(crc, b);
        state = State::READ_SIZE_H;
        break;
      case State::READ_SIZE_H:
        frame.size = uint16_t(size_lo | (uint16_t(b) << 8));
        crc = crc8(crc, b);
        frame.payload.clear();
        if (frame.size == 0) {
          state = State::READ_CRC;
        } else if (frame.size <= MAX_PAYLOAD) {
          frame.payload.reserve(frame.size);
          state = State::READ_PAYLOAD;
        } else {
          reset(); // unreasonable size
        }
        break;
      case State::READ_PAYLOAD:
        frame.payload.push_back(b);
        crc = crc8(crc, b);
        if (frame.payload.size() >= frame.size) {
          state = State::READ_CRC;
        }
        break;
      case State::READ_CRC:
        frame.crc = b;
        if (crc == frame.crc) {
          if (handler) {
            handler(frame);
          }
        }
        // Regardless of CRC validity, reset and hunt for next frame
        reset();
        break;
    }
  }

private:
  const uint16_t MAX_PAYLOAD = 128;

  enum class State {
    WAIT_DOLLAR, WAIT_X, READ_DIR, READ_FLAGS,
    READ_FUNC_L, READ_FUNC_H, READ_SIZE_L, READ_SIZE_H,
    READ_PAYLOAD, READ_CRC
  } state = State::WAIT_DOLLAR;

  MSPv2Frame frame;
  uint8_t func_lo = 0, size_lo = 0;
  uint8_t crc = 0;
  Handler handler;

  void reset() 
  {
    state = State::WAIT_DOLLAR;
    frame = MSPv2Frame{};
    func_lo = size_lo = 0;
    crc = 0;
  }

  // CRC-8 DVB-S2 (poly 0xD5), initial 0x00
  static uint8_t crc8(uint8_t crc, uint8_t a)
  {
    crc ^= a;
    for (int i = 0; i < 8; ++i) {
      crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
    }
    return crc;
  }
};
