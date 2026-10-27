#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

using std::max;
using std::min;

template <typename T>
T constrain(T value, T low, T high) {
  return min(max(value, low), high);
}

class String {
 public:
  String() = default;
  String(const char* value) : value_(value) {}
  bool operator==(const char* value) const { return value_ == value; }
  String& operator+=(char value) { value_ += value; return *this; }
  void trim() {
    const auto first = value_.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) value_.clear();
    else value_ = value_.substr(first, value_.find_last_not_of(" \t\r\n") - first + 1);
  }

 private:
  std::string value_;
};

class HostSerial {
 public:
  void begin(unsigned long) {}
  int available() const { return static_cast<int>(input_.size()); }
  char read() { const char value = input_.front(); input_.erase(input_.begin()); return value; }
  void inject(const std::string& value) { input_ += value; }
  template <typename T> void print(const T&) {}
  template <typename T> void println(const T&) {}
  void println() {}

 private:
  std::string input_;
};

extern HostSerial Serial;
extern HostSerial Serial1;
extern unsigned long hostMillis;
extern int hostDistanceCm;
void hostAdvanceTime(unsigned long value);

inline unsigned long millis() { return hostMillis; }
inline void delay(unsigned long value) { hostAdvanceTime(value); }
inline void delayMicroseconds(unsigned int) {}
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline void tone(int, unsigned int, unsigned long = 0) {}
inline unsigned long pulseIn(int, int, unsigned long) {
  if (hostDistanceCm < 0) return 0;
  return static_cast<unsigned long>(std::lround(std::max(hostDistanceCm, 1) * 20000.0 / 346.75));
}

constexpr int HIGH = 1;
constexpr int LOW = 0;
constexpr int INPUT = 0;
constexpr int OUTPUT = 1;
