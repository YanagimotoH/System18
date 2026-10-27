#pragma once
#include <cmath>
#include <cstdint>

float hostHeadingDeg();

class LSM303 {
 public:
  struct Vector { int16_t x = 0, y = 0, z = 0; };
  Vector m;
  void init() {}
  void enableDefault() {}
  void writeReg(uint8_t, uint8_t) {}
  void read() {
    const float angle = hostHeadingDeg() * 3.14159265358979323846f / 180.0f;
    m.x = static_cast<int16_t>(std::cos(angle) * 1000.0f);
    m.y = static_cast<int16_t>(std::sin(angle) * 1000.0f);
  }
  static constexpr uint8_t CRB_REG_M = 0x01;
  static constexpr uint8_t CRA_REG_M = 0x00;
};
