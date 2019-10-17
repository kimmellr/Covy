/*

*/


#include "esphome.h"
#include "pins.h"
#include "class_motor_control.h"

int stallguard = 1;
long travel = 2;
long velocity = 3;
long current_open = 1;
long current_close = 2;

MotorControl mc;

class MyCustomCover : public Component, public Cover {
 public:

  void setup() override {
    // This will be called by App.setup()
    pinMode(5, INPUT);
    pinMode(btn1,INPUT_PULLUP);
    pinMode(btn2,INPUT_PULLUP);

    mc.setup_pins(chipCS, CLOCKOUT, ENABLE_PIN);
    mc.Begin(stallguard, travel, velocity, current_open, current_close);
    mc.setup_motors();
  }
  CoverTraits get_traits() override {
    auto traits = CoverTraits();
    traits.set_is_assumed_state(false);
    traits.set_supports_position(true);
    traits.set_supports_tilt(false);
    return traits;
  }
  void control(const CoverCall &call) override {
    // This will be called every time the user requests a state change.
    if (call.get_position().has_value()) {
      float pos = *call.get_position();
      // Write pos (range 0-1) to cover
      if (pos == 0) {
          mc.move_open();
      }

      if (pos == 1) {
          mc.move_close();
      }

      // Publish new state
      this->position = pos;
      this->publish_state();
    }
    if (call.get_stop()) {
      // User requested cover stop
      mc.moce_stop();
    }
  }
  void loop() override {
      mc.check_stall();
      // If the stepper location can be pulled, it can be updated in real time:
      // this->position = mc.get_location();
      // this->publish_state()
  }
};