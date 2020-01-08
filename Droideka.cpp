#include "Droideka.h"

Droideka::Droideka()
{
}

void Droideka::initialize(int l_m_p_1, int l_m_p_2, int l_m_p_pwm, int rec_rx, int rec_tx, int rec_state)
{
  longitudinal_mot_pin_1 = l_m_p_1;
  longitudinal_mot_pin_2 = l_m_p_2;
  longitudinal_mot_pin_pwm = l_m_p_pwm;

  pinMode(longitudinal_mot_pin_1, OUTPUT);
  pinMode(longitudinal_mot_pin_2, OUTPUT);
  pinMode(longitudinal_mot_pin_pwm, OUTPUT);

  rec.start(rec_rx, rec_tx, rec_state);
}

bool Droideka::receive_data()
{
  if (rec.isBtConnected()) // if bluetooth is connected
  {
    if (rec.receivedDataFromController()) // if new data has been collected
    {
      throttle_x = rec.throttle_x;
      throttle_y = rec.throttle_y;
      button1 = rec.but1;
      button2 = rec.but2;
      button3 = rec.but3;
      return true;
    }
  }
  return false;
}

ErrorCode Droideka::move(char motor = 'l', int speed = 0)
{
  int pin_1;
  int pin_2;
  int pin_pwm;

  // Choose motor pins
  if (motor == 'l')
  {
    pin_1 = longitudinal_mot_pin_1;
    pin_2 = longitudinal_mot_pin_2;
    pin_pwm = longitudinal_mot_pin_pwm;
  }
  else
  {
    return WRONG_MOTOR_SPECIFIED;
  }

  // Choose forward or backward motion
  if (speed > 0)
  {
    digitalWrite(pin_1, 1);
    digitalWrite(pin_2, 0);
  }
  else if (speed < 0)
  {
    digitalWrite(pin_1, 0);
    digitalWrite(pin_2, 1);
  }
  else
  {
    digitalWrite(pin_1, 0);
    digitalWrite(pin_2, 0);
  }

  int mapped_speed = abs(speed);
  mapped_speed = map(mapped_speed, 0, 100, 0, 127);
  // Choose speed
  if (mapped_speed >= 0 && mapped_speed < 128)
  {
    analogWrite(pin_pwm, mapped_speed);
  }
  else
  {
    return OUT_OF_BOUNDS_SPEED_SPECIFIED;
  }

  return NO_ERROR;
}