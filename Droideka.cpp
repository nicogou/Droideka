#include "Droideka.h"

Droideka::Droideka(Stream *debugBoardStream_front, Stream *debugBoardStream_back)
{
  servoBus_front = new ServoBus(debugBoardStream_front, servo_bus_write_pin);
  servoBus_back = new ServoBus(debugBoardStream_back, servo_bus_write_pin);
  servoBus_front->setEventHandler(REPLY_POSITION, this->receive_debug_board_position);
  servoBus_back->setEventHandler(REPLY_POSITION, this->receive_debug_board_position);
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

bool Droideka::forward()
{
  if (throttle_x > 0)
  {
    return true;
  }
  return false;
}

bool Droideka::backward()
{
  if (throttle_x < 0)
  {
    return true;
  }
  return false;
}

bool Droideka::leftward()
{
  if (throttle_y > 0)
  {
    return true;
  }
  return false;
}

bool Droideka::rightward()
{
  if (throttle_y < 0)
  {
    return true;
  }
  return false;
}

bool Droideka::button1_pushed()
{
  return rec.but1Pushed();
}

bool Droideka::button1_clicked()
{
  return rec.but1Clicked();
}

bool Droideka::button1_released()
{
  return rec.but1Released();
}

bool Droideka::button2_pushed()
{
  return rec.but2Pushed();
}

bool Droideka::button2_clicked()
{
  return rec.but2Clicked();
}

bool Droideka::button2_released()
{
  return rec.but2Released();
}

bool Droideka::button3_pushed()
{
  return rec.but3Pushed();
}

bool Droideka::button3_clicked()
{
  return rec.but3Clicked();
}

bool Droideka::button3_released()
{
  return rec.but3Released();
}

ErrorCode Droideka::roll(int speed = 0)
{
  int pin_1 = longitudinal_mot_pin_1;
  int pin_2 = longitudinal_mot_pin_2;
  int pin_pwm = longitudinal_mot_pin_pwm;

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
  mapped_speed = map(mapped_speed, 0, 100, 0, 255);
  // Choose speed
  if (mapped_speed >= 0 && mapped_speed < 256)
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
    this->servoBus_front->requestPosition(this->motor_ids[i]);
    this->servoBus_back->requestPosition(this->motor_ids[i]);
  }
  return &lastState_;
}

void Droideka::act(Action *action)
{
  for (uint8_t i = 0; i < MOTOR_NB / 2; i++)
  {
    if (action->commands[i][2] > 0)
    {
      this->servoBus_front->MoveTime(this->motor_ids[i], action->commands[i][0], action->commands[i][1]);
      this->servoBus_back->MoveTime(this->motor_ids[i + 6], action->commands[i + 6][0], action->commands[i + 6][1]);
    }
    else
    {
      this->servoBus_front->SetUnload(this->motor_ids[i]);
      this->servoBus_back->SetUnload(this->motor_ids[i + 6]);
    }
  }
}

ErrorCode Droideka::encode_leg_angles(int leg_id)
{
  shoulder_angle_encoder[leg_id] = deg_to_encoder(3 * leg_id + 0, shoulder_angle_deg[leg_id]);
  hip_angle_encoder[leg_id] = deg_to_encoder(3 * leg_id + 1, hip_angle_deg[leg_id]);
  knee_angle_encoder[leg_id] = deg_to_encoder(3 * leg_id + 2, knee_angle_deg[leg_id]);

  if (shoulder_angle_encoder[leg_id] < extreme_values_motor[3 * leg_id + 0][0] || shoulder_angle_encoder[leg_id] > extreme_values_motor[3 * leg_id + 0][2])
  {
    return OUT_OF_BOUNDS_SHOULDER_ANGLE;
  }
  if (hip_angle_encoder[leg_id] < extreme_values_motor[3 * leg_id + 1][0] || hip_angle_encoder[leg_id] > extreme_values_motor[3 * leg_id + 1][2])
  {
    return OUT_OF_BOUNDS_HIP_ANGLE;
  }
  if (knee_angle_encoder[leg_id] < extreme_values_motor[3 * leg_id + 2][0] || knee_angle_encoder[leg_id] > extreme_values_motor[3 * leg_id + 2][2])
  {
    return OUT_OF_BOUNDS_KNEE_ANGLE;
  }

  return NO_ERROR;
}

int Droideka::deg_to_encoder(int motor_id, float deg_angle)
{
  int encoder_angle;
  encoder_angle = extreme_values_motor[motor_id][1] + extreme_values_motor[motor_id][3] * (int)(deg_angle / servo_deg_ratio);

  return encoder_angle;
}

float Droideka::encoder_to_deg(int motor_id, int encoder_angle)
{
  float deg_angle;
  deg_angle = (encoder_angle - extreme_values_motor[motor_id][1]) * servo_deg_ratio / extreme_values_motor[motor_id][3];

  return deg_angle;
}

ErrorCode Droideka::move(int throttle)
{
  if (get_mode() == WALKING)
  {
    twist(1000, 10);
  }
  else if (get_mode() == ROLLING)
  {
    roll(throttle);
  }
}

DroidekaMode Droideka::get_mode()
{
  if (current_position == -1)
  {
    return ROLLING;
  }
  else
  {
    return WALKING;
  }
}

ErrorCode Droideka::change_mode()
{
  if (get_mode() == WALKING)
  {
    park(1000, 0);
  }
  else if (get_mode() == ROLLING)
  {
    unpark(1000, 0);
  }
}

ErrorCode Droideka::in_position(Droideka_Position pos, Action &pos_act, int time)
{
  float knee_angle_sign;

  if (!pos.valid_position)
  {
    return POSITION_UNREACHABLE;
  }
  else
  {
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

      ErrorCode error = encode_leg_angles(ii);
      if (error != NO_ERROR)
      {
        return error;
      }

      pos_act.commands[3 * ii + 0][0] = shoulder_angle_encoder[ii];
      pos_act.commands[3 * ii + 1][0] = hip_angle_encoder[ii];
      pos_act.commands[3 * ii + 2][0] = knee_angle_encoder[ii];
    }
    pos_act.set_time(time);

    return NO_ERROR;
  }
}

void Droideka::set_parking_position(Droideka_Position *park)
{
  if (park->valid_position)
  {
    parking_position = park;
    parking_updated = true;
  }
}

void Droideka::set_parking_position(float park[LEG_NB][3])
{
  Droideka_Position *temp = new Droideka_Position(park);
  if (temp->valid_position)
  {
    parking_position = temp;
    parking_updated = true;
  }
}

ErrorCode Droideka::park(int time = 500, int offset_time = 500)
{
  if (current_position == -1)
  {
    return ROBOT_ALREADY_PARKED;
  }
  else
  {
    if (parking_updated)
    {
      Action temp_action;
      ErrorCode result;
      Droideka_Position temp_transition_pos = *parking_transition_position;

      for (int jj = 0; jj < LEG_NB; jj++)
      {
        for (int kk = 0; kk < 2; kk++)
        {
          temp_transition_pos.legs[jj][kk] = walking_sequence[current_position][jj][kk];
        }
        temp_transition_pos.legs[jj][2] = Y_NOT_TOUCHING;
      }

      result = in_position(temp_transition_pos, temp_action, time);
      if (result == NO_ERROR)
      {
        temp_action.set_active();
        // temp_action.shoulders_active(false);
        act(&temp_action);
        last_action_millis = millis();         // Not sure if this is necessary due to the delay below, but I do it anyway, just to be safe
        time_last_action = time;               // Not sure if this is necessary due to the delay below, but I do it anyway, just to be safe
        offset_time_last_action = offset_time; // Not sure if this is necessary due to the delay below, but I do it anyway, just to be safe
        delay(time + offset_time);
      }
      else
      {
        return PARKING_TRANSITION_POSITION_IMPOSSIBLE;
      }

      result = in_position(*parking_position, temp_action, time);
      if (result == NO_ERROR)
      {
        temp_action.set_active();
        act(&temp_action);
        last_action_millis = millis();         // Not sure if this is necessary due to the delay below, but I do it anyway, just to be safe
        time_last_action = time;               // Not sure if this is necessary due to the delay below, but I do it anyway, just to be safe
        offset_time_last_action = offset_time; // Not sure if this is necessary due to the delay below, but I do it anyway, just to be safe
        delay(time + offset_time);
      }
      else
      {
        return PARKING_POSITION_IMPOSSIBLE;
      }
      current_position = -1;
      rec.flushSerialPort(); // Data is received from the receiver during the delays in this function, so we need to flush this unwanted data in order to receive the real-time data.
      return NO_ERROR;
    }
    else
    {
      return PARKING_POSITION_NOT_UPDATED;
    }
  }
}

ErrorCode Droideka::unpark(int time = 500, int offset_time = 500)
{
  if (current_position != -1)
  {
    return ROBOT_ALREADY_UNPARKED;
  }
  else
  {
    Action unparking;
    ErrorCode result;
    result = in_position(*parking_transition_position, unparking, time);
    if (result == NO_ERROR)
    {
      unparking.set_active();
      act(&unparking);
      last_action_millis = millis();         // Not sure if this is necessary due to the delay below, but I do it anyway, just to be safe
      time_last_action = time;               // Not sure if this is necessary due to the delay below, but I do it anyway, just to be safe
      offset_time_last_action = offset_time; // Not sure if this is necessary due to the delay below, but I do it anyway, just to be safe
      delay(time + offset_time);
    }
    else
    {
      unparking.set_active(false);
      return PARKING_TRANSITION_POSITION_IMPOSSIBLE;
    }

    result = in_position(*starting_position_walking, unparking, time);
    if (result == NO_ERROR)
    {
      unparking.set_active();
      act(&unparking);
      last_action_millis = millis();         // Not sure if this is necessary due to the delay below, but I do it anyway, just to be safe
      time_last_action = time;               // Not sure if this is necessary due to the delay below, but I do it anyway, just to be safe
      offset_time_last_action = offset_time; // Not sure if this is necessary due to the delay below, but I do it anyway, just to be safe
      delay(time + offset_time);
    }
    else
    {
      unparking.set_active(false);
      return STARTING_WALKING_POSITION_IMPOSSIBLE;
    }

    current_position = nb_walking_sequence - 1;
    rec.flushSerialPort(); // Data is received from the receiver during the delays in this function, so we need to flush this unwanted data in order to receive the real-time data.
    return NO_ERROR;
  }
}

ErrorCode Droideka::walk(int time = 500, int offset_time = 500)
{
  if (current_position == -1)
  {
    return ROBOT_PARKED_WHEN_ASKED_TO_WALK;
  }
  else
  {
    int temp_current_pos;
    int forward_or_backward = 0;
    if (forward())
    {
      forward_or_backward = 1;
    }
    else if (backward())
    {
      forward_or_backward = -1;
    }

    Action walking;
    Droideka_Position next_pos = *starting_position_walking;
    ErrorCode result;
    double current_action_millis;

    temp_current_pos = max(current_position + forward_or_backward, current_position + forward_or_backward + nb_walking_sequence);
    temp_current_pos = temp_current_pos % nb_walking_sequence;
    for (int jj = 0; jj < LEG_NB; jj++)
    {
      for (int kk = 0; kk < 3; kk++)
      {
        next_pos.legs[jj][kk] = walking_sequence[temp_current_pos][jj][kk];
      }
    }
    result = in_position(next_pos, walking, time);
    if (result == NO_ERROR)
    {
      walking.set_active();

      current_action_millis = millis();
      if (current_action_millis - last_action_millis > time_last_action + offset_time_last_action)
      {
        act(&walking);
        last_action_millis = current_action_millis;
        time_last_action = time;
        offset_time_last_action = offset_time;
      }
      else
      {
        return WAITING;
      }
    }
    else
    {
      walking.set_active(false);
      return result;
    }

    current_position = temp_current_pos;
    return NO_ERROR;
  }
}

ErrorCode Droideka::slide(int time = 500, int offset_time = 500)
{
  if (current_position == -1)
  {
    return ROBOT_NOT_IN_POSITION_TO_SLIDE;
  }
  else
  {
    int temp_current_pos;
    int forward_or_backward = 0;
    if (forward()) // TODO change to joystick sideways
    {
      forward_or_backward = 1;
    }
    else if (backward()) // TODO change to joystick sideways
    {
      forward_or_backward = -1;
    }

    Action walking;
    Droideka_Position next_pos = *starting_position_walking;
    ErrorCode result;
    double current_action_millis;

    temp_current_pos = max(current_position + forward_or_backward, current_position + forward_or_backward + nb_sliding_sequence);
    temp_current_pos = temp_current_pos % nb_sliding_sequence;
    for (int jj = 0; jj < LEG_NB; jj++)
    {
      for (int kk = 0; kk < 3; kk++)
      {
        next_pos.legs[jj][kk] = sliding_sequence[temp_current_pos][jj][kk];
      }
    }
    result = in_position(next_pos, walking, time);
    if (result == NO_ERROR)
    {
      walking.set_active();

      current_action_millis = millis();
      if (current_action_millis - last_action_millis > time_last_action + offset_time_last_action)
      {
        act(&walking);
        last_action_millis = current_action_millis;
        time_last_action = time;
        offset_time_last_action = offset_time;
      }
      else
      {
        return WAITING;
      }
    }
    else
    {
      walking.set_active(false);
      return result;
    }

    current_position = temp_current_pos;
    return NO_ERROR;
  }
}

ErrorCode Droideka::twist(int time = 500, int offset_time = 500)
{
  if (current_position == -1)
  {
    return ROBOT_NOT_IN_POSITION_TO_SLIDE;
  }
  else
  {
    int temp_current_pos;
    int forward_or_backward = 0;
    if (forward()) // TODO change to joystick sideways
    {
      forward_or_backward = 1;
    }
    else if (backward()) // TODO change to joystick sideways
    {
      forward_or_backward = -1;
    }

    Action walking;
    Droideka_Position next_pos = *starting_position_walking;
    ErrorCode result;
    double current_action_millis;

    temp_current_pos = max(current_position + forward_or_backward, current_position + forward_or_backward + nb_twisting_sequence);
    temp_current_pos = temp_current_pos % nb_twisting_sequence;
    for (int jj = 0; jj < LEG_NB; jj++)
    {
      for (int kk = 0; kk < 3; kk++)
      {
        next_pos.legs[jj][kk] = twisting_sequence[temp_current_pos][jj][kk];
      }
    }
    result = in_position(next_pos, walking, time);
    if (result == NO_ERROR)
    {
      walking.set_active();

      current_action_millis = millis();
      if (current_action_millis - last_action_millis > time_last_action + offset_time_last_action)
      {
        act(&walking);
        last_action_millis = current_action_millis;
        time_last_action = time;
        offset_time_last_action = offset_time;
      }
      else
      {
        return WAITING;
      }
    }
    else
    {
      walking.set_active(false);
      return result;
    }

    current_position = temp_current_pos;
    return NO_ERROR;
  }
}