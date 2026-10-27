#pragma once
constexpr int ZUMO_BUTTON = 0;
class Pushbutton {
 public:
  explicit Pushbutton(int) {}
  bool isPressed() const { return false; }
};
