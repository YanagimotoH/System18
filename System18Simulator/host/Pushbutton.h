#pragma once
constexpr int ZUMO_BUTTON = 0;
extern bool hostButtonPressed;
class Pushbutton {
 public:
  explicit Pushbutton(int) {}
  bool isPressed() const { return hostButtonPressed; }
};
