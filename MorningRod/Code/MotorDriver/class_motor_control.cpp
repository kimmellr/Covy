#include "Arduino.h"
//#include "esphome.h"
#include "SPI.h"
#include "class_motor_control.h"
//#include "pins.h"

MotorControl::MotorControl()
{
  _motor_running = false;
  _absolute_position = 0;
  _Vfs = 0.325;
  _Rsense = 0.075;
}

void MotorControl::Begin(int stallguard, long travel, long velocity, long current_open, long current_close)
{
  _stallguard = stallguard;
  _velocity = velocity;
  _travel = travel;
  _current_open = current_open;
  _current_close = current_close;
}
void MotorControl::setup_pins(int chipCS)
{
  _chipCS = chipCS;
}
void MotorControl::move_position(float position)
{
  long steps = to_steps( position * _travel);
  _requested_position = steps;
  long new_travel = (position - _absolute_position) / _travel;
  auto m = Movement(_current_open, _velocity, -new_travel, _stallguard, _chipCS);
  m.move();
  _motor_running = true;
}
void MotorControl::move_open()
{
  _requested_position = 0.0
  auto m = Movement(_current_open, _velocity, -_absolute_position, _stallguard, _chipCS);
  digitalWrite(ENABLE_PIN, LOW); // enable the TMC5130
  m.move();
  _full_close = false;
  _full_open = true;
  _motor_running = true;
}

void MotorControl::move_close()
{
  _requested_position = to_steps(_travel);
  long steps = _requested_position - _absolute_position;
  auto m = Movement(_current_close, _velocity, steps, _stallguard, _chipCS);
  digitalWrite(ENABLE_PIN, LOW); // enable the TMC5130
  m.move();
  _full_close = true;
  _full_open = false;
  _motor_running = true;
}

int MotorControl::get_position()
{
  unsigned long xactual;
  xactual = sendData(0x21, 0); // Get the current Position
  return xactual;
}

void MotorControl::move_stop()
{

  sendData(0x23 + 0x80, 0);      // set VMAX and VSTART to zero, then enable positioning mode
  sendData(0x27 + 0x80, 0);      //  > doing this stops the motor
  sendData(0x20 + 0x80, 0);      //
  while (sendData(0x22, 0) != 0) // wait for the motor to stop (VACTUAL != 0 until stopped)
    delayMicroseconds(10);
  sendData(0x21 + 0x80, 0);         // target=xactual=0 to keep motor stopped
  sendData(0x2D + 0x80, 0);         //
  sendData(0x23 + 0x80, 0x180);     // fix VMAX and VSTART to previous values so motor can run again
  sendData(0x27 + 0x80, _velocity); //
  digitalWrite(ENABLE_PIN, HIGH);   // disable the motor driver if possible
  _motor_running = false;
}
void MotorControl::check_stall()
{
  if ((sendData(0x35, 0) & 0x200) == 1)
  {
    // End of Motion
    move_stop();
  }
  else if ((sendData(0x35, 0) & 0x40) == 1)
  {
    //Stalled
    if (abs(get_position() - _requested_position) < 100) //Within 100 steps
    { // Stalled close to the end

      if (_full_close)
      {
        _absolute_position = get_position() + _absolute_position;  // Absolute position was the starting point, plus the current position
        _travel = to_inches(_absolute_position); //Update the travel position.  This is probably the truth
        move_stop();
      }
      else if (_full_open)
      {
        _absolute_position = 0;
        // Likely lost some steps somewhere
        move_stop();
      }
    } 
    else
    {
      move_stop(); // True stall
      _absolute_position = get_position() + _absolute_position;  // Best guess on where we are
    }
    
  }
  else
  {
    //Still moing along
  }
}
void MotorControl::setup_motors()
{
  pinMode(_chipCS, OUTPUT);
  pinMode(CLOCKOUT, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(_chipCS, HIGH);
  digitalWrite(ENABLE_PIN, LOW);

  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setDataMode(SPI_MODE3);
  SPI.begin(SCLK, MISO, MOSI, _chipCS); // Edit 'pins.h' to change pins

  sendData(0x00 + 0x80, 0x0);        // General settings / en_pwm_mode OFF
  sendData(0x6C + 0x80, 0x000101D5); // CHOPCONF
  sendData(0x10 + 0x80, 0x00010D00); // IHOLD_IRUN // 0x00011900 = 25 = 2 Amps // 0x00010D00 = 13 = 1 Amp
  sendData(0x20 + 0x80, 0x00000000); //RAMPMODE=0

  sendData(0x60 + 0x80, 0xAAAAB554); // writing value 0xAAAAB554 = 0 = 0.0 to address 25 = 0x60(MSLUT[0])
  sendData(0x61 + 0x80, 0x4A9554AA); // writing value 0x4A9554AA = 1251300522 = 0.0 to address 26 = 0x61(MSLUT[1])
  sendData(0x62 + 0x80, 0x24492929); // writing value 0x24492929 = 608774441 = 0.0 to address 27 = 0x62(MSLUT[2])
  sendData(0x63 + 0x80, 0x10104222); // writing value 0x10104222 = 269500962 = 0.0 to address 28 = 0x63(MSLUT[3])
  sendData(0x64 + 0x80, 0xFBFFFFFF); // writing value 0xFBFFFFFF = 0 = 0.0 to address 29 = 0x64(MSLUT[4])
  sendData(0x65 + 0x80, 0xB5BB777D); // writing value 0xB5BB777D = 0 = 0.0 to address 30 = 0x65(MSLUT[5])
  sendData(0x66 + 0x80, 0x49295556); // writing value 0x49295556 = 1227445590 = 0.0 to address 31 = 0x66(MSLUT[6])
  sendData(0x67 + 0x80, 0x00404222); // writing value 0x00404222 = 4211234 = 0.0 to address 32 = 0x67(MSLUT[7])
  sendData(0x68 + 0x80, 0xFFFF8056); // writing value 0xFFFF8056 = 0 = 0.0 to address 33 = 0x68(MSLUTSEL)
  sendData(0x69 + 0x80, 0x00F70000); // writing value 0x00F70000 = 16187392 = 0.0 to address 34 = 0x69(MSLUTSTART)
  sendData(0x70 + 0x80, 0x00000000); // PWMCONF

  //Standard values for speed and acceleration
  int q = _stallguard;
  //   DEBUG_STREAM.print("Stall value: ");
  //   DEBUG_STREAM.println(q);
  q &= 0x7F;
  q = q << 16;
  sendData(0x6D + 0x80, q); // STALLGUARD

  this->move_stop();
}
long MotorControl::to_steps(float inches)
{
  long steps = long(inches * STEPS_PER_INCH);
  return steps;
}
float MotorControl::to_inches(long steps)
{
  float inches = float(steps / STEPS_PER_INCH);
  return inches;
}
unsigned long MotorControl::sendData(unsigned long address, unsigned long datagram)
{
  //TMC5130 takes 40 bits of data: 8 address and 32 data
  delay(10);
  uint8_t stat;
  unsigned long i_datagram = 0;
  digitalWrite(_chipCS, LOW);
  delayMicroseconds(10);
  stat = SPI.transfer(address);
  i_datagram |= SPI.transfer((datagram >> 24) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram >> 16) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram >> 8) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram)&0xff);
  digitalWrite(_chipCS, HIGH);
  return i_datagram;
}

int MotorControl::current_to_command(float current)
{

    // Calculate the CS Value
    int cs = -1 * ((-1600 * current * _Rsense - 32 * current + 25 * sqrt(2) * _Vfs) /
              (25 * sqrt(2) * _Vfs));

    // Build the command string in binary
    String part1 = "0001000";
    String part2 = "";
    for (int k = 0; k <= 4; k++)
    {
        int b = (cs & (1 << k)) >> k; //get k-th bit of cs
        part2 = String(b) + part2;
    };
    String part3 = "00000000";
    String command_string = part1 + part2 + part3;

    // Convert the binary to integer
    int value = 0;
    int len = command_string.length();
    for (int i = 0; i < len; i++) // for every character in the string
    {
        value *= 2; // double the result so far
        if (command_string[i] == '1')
            value++; //add 1 if needed
    }
    return value;
}

void MotorControl::move()
{
  //Move Code  assumes driver is enabled
  //SPI Commands using _current, _velocity, _travel
  //default _current = 0x00010A00
  sendData(0x10 + 0x80, current_to_command(_current)); // 13 = 1.06A current for open function
  sendData(0xA0, 0x00000000);      //RAMPMODE=0

  sendData(0x14 + 0x80, _velocity - 1); // VCOOLTHRS: This value disable stallGuard below a certain velocity to prevent premature stall

  //Stallguard_open will need to change due to higher current than stallguard_close
  int q = _stallguard;
  q &= 0x6F;
  q = q << 16;
  sendData(0xED, q); // STALLGUARD_OPEN

  sendData(0x24 + 0x80, 1000);      //A1
  sendData(0x26 + 0x80, 4000);      //AMAX
  sendData(0x28 + 0x80, 800);       // DMAX
  sendData(0x2A + 0x80, 3000);      //D1
  sendData(0x23 + 0x80, 0);         // VSTART
  sendData(0x2B + 0x80, 10);        //VSTOP
  sendData(0x25 + 0x80, 40000);     //V1
  sendData(0x27 + 0x80, _velocity); //VMAX

  delay(5);

  // clear flags
  sendData(0x35, 0);
  sendData(0x34 + 0x80, 0x400); // Enable stallguard

  sendData(0x2D+  0x80, -_travel); //XTARGET: Positive makes it move right
  sendData(0xA1, 0);        // set XACTUAL to zero
}