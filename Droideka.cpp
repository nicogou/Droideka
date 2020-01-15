#include "Droideka.h"

Droideka::Droideka(Stream *debugBoardStream)
{
  this->servoBus_ = new ServoBus(debugBoardStream, servo_bus_write_pin);
  this->servoBus_->setEventHandler(REPLY_POSITION, this->receive_debug_board_position);
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

State lastState_;
unsigned long timestampLastHandledMessage_ = 0;

void Droideka::receive_debug_board_position(uint8_t id, uint8_t command, uint16_t param1, uint16_t param2)
{
  (void)command;
  (void)param2;
  lastState_.positions[id] = param1;
  lastState_.is_position_updated[id] = true;
  //Check for incoherences in motor readings.
  if (param1 <= 10 || 1030 < param1)
  {
    lastState_.correct_motor_reading = false;
  }
  // TODO find index of array according to id, now we suppose the index and id are the same
}

State *Droideka::read_debug_board_positions()
{
  memset(lastState_.is_position_updated, 0, sizeof(bool) * MOTOR_NB);
  for (uint8_t i = 0; i < MOTOR_NB; i++)
  {
    this->servoBus_->requestPosition(this->motor_ids[i]);
  }
  return &lastState_;
}

void Droideka::act(Action *action)
{
  for (uint8_t i = 0; i < MOTOR_NB; i++)
  {
    if (action->commands[i][2] > 0)
    {
      this->servoBus_->MoveTime(this->motor_ids[i], action->commands[i][0], action->commands[i][1]);
    }
    else
    {
      this->servoBus_->SetUnload(this->motor_ids[i]);
    }
  }
}

ErrorCode Droideka::in_position(Droideka_Position pos, Action *pos_act)
{
  float knee_angle_sign;

  for (int ii = 0; ii < LEG_NB; ii++)
  {
    shoulder_angle_deg[ii] = pos.legs[ii][0];

    knee_angle_rad[ii] = acos((pos.legs[ii][1] * pos.legs[ii][1] + pos.legs[ii][2] * pos.legs[ii][2] - hip_length * hip_length - tibia_length * tibia_length) / (2 * hip_length * tibia_length));

    // The knee angle can be either positive or negative. When the Droideka is walking, we want the knee angle to be negative, but when the Droideka is in parking position, we want the knee angle to be positive.
    if (pos.legs[ii][2] > 0)
    {
      knee_angle_sign = 1;
    }
    else
    {
      knee_angle_sign = -1;
    }
    knee_angle_rad[ii] = knee_angle_sign * knee_angle_rad[ii];

    hip_angle_rad[ii] = atan(pos.legs[ii][2] / pos.legs[ii][1]) - atan(tibia_length * sin(knee_angle_rad[ii]) / (hip_length + tibia_length * cos(knee_angle_rad[ii])));

    hip_angle_deg[ii] = hip_angle_rad[ii] * 180 / 3.141592;
    knee_angle_deg[ii] = knee_angle_rad[ii] * 180 / 3.141592;

    if (shoulder_angle_deg[ii] < min_shoulder_angle || shoulder_angle_deg[ii] > max_shoulder_angle)
    {
      return OUT_OF_BOUNDS_SHOULDER_ANGLE;
    }

    if (hip_angle_deg[ii] < min_hip_angle || hip_angle_deg[ii] > max_hip_angle)
    {
      return OUT_OF_BOUNDS_HIP_ANGLE;
    }

    if (knee_angle_deg[ii] < min_knee_angle || knee_angle_deg[ii] > max_knee_angle)
    {
      return OUT_OF_BOUNDS_KNEE_ANGLE;
    }

    shoulder_angle_encoder[ii] = shoulder_angle_deg[ii] * servo_deg_ratio;
    hip_angle_encoder[ii] = hip_angle_deg[ii] * servo_deg_ratio;
    knee_angle_encoder[ii] = knee_angle_deg[ii] * servo_deg_ratio;

    pos_act->commands[3 * ii][0] = (uint16_t)shoulder_angle_encoder[ii];
    pos_act->commands[3 * ii + 2][0] = (uint16_t)hip_angle_encoder[ii];
    pos_act->commands[3 * ii + 1][0] = (uint16_t)knee_angle_encoder[ii];
  }
  for (int jj = 0; jj < MOTOR_NB; jj++)
  {
    // pos_act->commands[jj][1] = 100;
  }
  return NO_ERROR;
}

void Droideka::set_parking_position(Droideka_Position *park)
{
  parking = park;
  parking_updated = true;
}

void Droideka::set_parking_position(float park[LEG_NB][3])
{
  parking = new Droideka_Position(park);
  parking_updated = true;
}

ErrorCode Droideka::park(bool actually_move = true)
{
  if (parking_updated)
  {
    Droideka_Position temp_parking(parking->legs);
    State *current_state = read_debug_board_positions();
    float temp_shoulder_angle_deg;
    Action *temp_action;

    for (int ii = 0; ii < LEG_NB; ii++)
    {
      temp_shoulder_angle_deg = current_state->positions[ii] * servo_deg_ratio;
      temp_parking.legs[ii][0] = temp_shoulder_angle_deg;
    }

    if (in_position(temp_parking, temp_action) == NO_ERROR)
    {
      if (actually_move)
      {
        act(temp_action);
      }
      if (in_position(*parking, temp_action) == NO_ERROR)
      {
        if (actually_move)
        {
          act(temp_action);
        }
      }
      else
      {
        return PARKING_POSITION_IMPOSSIBLE;
      }
    }
    else
    {
      return PREPARKING_POSITION_IMPOSSIBLE;
    }

    return NO_ERROR;
  }
  else
  {
    return PARKING_POSITION_NOT_UPDATED;
  }
}