#include "Droideka.h"

Droideka::Droideka()
{
  Serial2.begin(baud_rate_bt);
  ET_in.begin(details(rx_data), &Serial2);
  ET_out.begin(details(tx_data), &Serial2);
}

void Droideka::initialize(int l_m_p_1 = 0, int l_m_p_2 = 0, int l_m_p_pwm = 0)
{
  longitudinal_mot_pin_1 = l_m_p_1;
  longitudinal_mot_pin_2 = l_m_p_2;
  longitudinal_mot_pin_pwm = l_m_p_pwm;

  pinMode(longitudinal_mot_pin_1, OUTPUT);
  pinMode(longitudinal_mot_pin_2, OUTPUT);
  pinMode(longitudinal_mot_pin_pwm, OUTPUT);
}

bool Droideka::receive_data()
{
  if (ET_in.receiveData())
  {
    joy_x = rx_data.joy_x;
    joy_y = rx_data.joy_y;
    mode = rx_data.mode;
    trig_left = rx_data.trig_left;
    trig_right = rx_data.trig_right;
    return true;
  }
  return false;
}

ErrorCode Droideka::move(char motor = 'l', int speed = 0, char f_or_b = 'f')
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
  if (f_or_b == 'f')
  {
    digitalWrite(pin_1, 1);
    digitalWrite(pin_2, 0);
  }
  else if (f_or_b == 'b')
  {
    digitalWrite(pin_1, 0);
    digitalWrite(pin_2, 1);
  }
  else
  {
    return WRONG_MOTOR_DIRECTION_SPECIFIED;
  }

  // Choose speed
  if (speed >= 0 && speed < 256)
  {
    analogWrite(pin_pwm, speed);
  }
  else
  {
    return NEGATIVE_SPEED_SPECIFIED;
  }

  return NO_ERROR;
}