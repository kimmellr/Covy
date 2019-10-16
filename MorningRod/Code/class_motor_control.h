#ifndef Motor_control
#define Motor_control
// #include "Arduino.h"
class Movement
{
  unsigned long _current;
  long _velocity;
  long _travel;
  long _stallguard;
  
  public :
    Movement(unsigned long current, long velocity, long travel, long stallguard);
    void move();
};

class MotorControl
{
    int _stallguard;
    long _travel;
    long _velocity;
    bool _motor_running;
    long _current_open;
    long _current_close;
    int _chipCS;
    float _absolute_position;   // absolute position 0-1
    float _requested_position;  // requested travel position
    bool _finding_home;         // Used to set absolute position using move_home
    bool _full_open;            // am i running all the way open
    bool _full_close;           // am i running all the way close

  public:
  // Motor_control(int stallguard, long travel, long velocity);
    MotorControl();
    void Begin(int stallguard, long travel, long velocity, long current_open, long current_close);
    void setup_pins(int chipCS, int CLOCKOUT, int ENABLE_PIN);
    void move_open();
    void move_close();
    void move_position(float position);
    void move_stop();
    void move_home();
    int get_position():
    void check_stall();
    void setup_motors();
    unsigned long sendData(nsigned long address, unsigned long datagram);


};

#endif