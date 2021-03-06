#include "Droideka.h"

Droideka::Droideka(Stream *debugBoardStream_front, Stream *debugBoardStream_back)
{
  servoBus_front = new ServoBus(debugBoardStream_front, servo_bus_write_pin_front);
  servoBus_back = new ServoBus(debugBoardStream_back, servo_bus_write_pin_back);
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
  if (throttle_x > 0 && abs(throttle_x) > abs(throttle_y))
  {
    return true;
  }
  return false;
}

bool Droideka::backward()
{
  if (throttle_x < 0 && abs(throttle_x) > abs(throttle_y))
  {
    return true;
  }
  return false;
}

bool Droideka::rightward()
{
  if (throttle_y > 0 && abs(throttle_y) > abs(throttle_x))
  {
    return true;
  }
  return false;
}

bool Droideka::leftward()
{
  if (throttle_y < 0 && abs(throttle_y) > abs(throttle_x))
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
    /* TODO : find out what variable to change with the throttle : the easy one is the speed of the moves, the harder one is the
     *        the easy one is the speed of the moves
     *        the harder one is the reach of the move, i.e. how far the legs go (requires more computation)
     */
    if (forward() || backward())
    {
      walk(1000, 10);
    }
    else if (leftward() || rightward())
    {
      turn_left(1000, 10);
    }
    else
    {
      // IDEA: add a function to go back to unparked position from any intermediate position ? This seems complicated.
    }
  }
  else if (get_mode() == ROLLING)
  {
    roll(throttle);
  }
}

DroidekaMode Droideka::get_mode()
{
  if (current_position == END_PARKING_SEQUENCE)
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

ErrorCode Droideka::park(int time = 500, int offset_time = 500)
{
  if (current_position == END_PARKING_SEQUENCE)
  {
    return ROBOT_ALREADY_PARKED;
  }
  else
  {
    for (int jj = 0; jj < LEG_NB; jj++)
    {
      for (int kk = 0; kk < 2; kk++)
      {
        sequences[START_PARKING_SEQUENCE][jj][kk] = sequences[current_position][jj][kk];
      }
    }
    int forward_or_backward = 1;
    int previous_position = current_position;
    current_position = START_PARKING_SEQUENCE - 1;
    while (current_position != END_PARKING_SEQUENCE)
    {
      ErrorCode result = execute_sequence(forward_or_backward, START_PARKING_SEQUENCE, LENGTH_PARKING_SEQUENCE, time, offset_time);
      // In some cases, the robot cannot go from an intermediate position to parking (some steps of the turning sequences for instance).
      // In this case, we stop the parking routine, and go back to previous state.
      if (result != NO_ERROR && result != WAITING)
      {
        current_position = previous_position;
        break;
      }
    }
    rec.flushSerialPort();
  }
}

ErrorCode Droideka::unpark(int time = 500, int offset_time = 500)
{
  if (current_position != END_PARKING_SEQUENCE)
  {
    return ROBOT_ALREADY_UNPARKED;
  }
  else
  {
    while (current_position != END_UNPARKING_SEQUENCE)
    {
      ErrorCode result = execute_sequence(1, START_UNPARKING_SEQUENCE, LENGTH_UNPARKING_SEQUENCE, time, offset_time);
      if (result != NO_ERROR && result != WAITING)
      {
        break;
      }
    }
    rec.flushSerialPort();
  }
}

ErrorCode Droideka::walk(int time = 500, int offset_time = 500)
{
  if (current_position == END_PARKING_SEQUENCE)
  {
    return ROBOT_PARKED_WHEN_ASKED_TO_MOVE;
  }
  else
  {
    if (current_position == END_UNPARKING_SEQUENCE)
    {
      current_position = END_WALKING_SEQUENCE;
    }
    int forward_or_backward = 0;
    if (forward())
    {
      forward_or_backward = 1;
    }
    else if (backward())
    {
      forward_or_backward = -1;
    }

    execute_sequence(forward_or_backward, START_WALKING_SEQUENCE, LENGTH_WALKING_SEQUENCE, time, offset_time);
  }
}

ErrorCode Droideka::turn_left(int time = 500, int offset_time = 500)
{
  if (current_position == END_PARKING_SEQUENCE)
  {
    return ROBOT_PARKED_WHEN_ASKED_TO_MOVE;
  }
  else
  {
    if (current_position == END_UNPARKING_SEQUENCE)
    {
      current_position = END_TURN_LEFT_SEQUENCE;
    }
    int forward_or_backward = 0;
    if (leftward()) // TODO change to joystick sideways
    {
      forward_or_backward = 1;
    }
    else if (rightward()) // TODO change to joystick sideways
    {
      forward_or_backward = -1;
    }
    if (forward_or_backward != 0)
    {
      execute_sequence(forward_or_backward, START_TURN_LEFT_SEQUENCE, LENGTH_TURN_LEFT_SEQUENCE, time, offset_time);
    }
  }
}

ErrorCode Droideka::execute_sequence(int f_or_b, int start_sequence, int length_sequence, int time, int offset_time)
{
  int temp_current_pos;

  Action walking;
  Droideka_Position next_pos(sequences[START_UNPARKING_SEQUENCE + LENGTH_UNPARKING_SEQUENCE - 1]);
  ErrorCode result;
  double current_action_millis;

  temp_current_pos = max(current_position - start_sequence + f_or_b, current_position - start_sequence + f_or_b + length_sequence);
  temp_current_pos = temp_current_pos % length_sequence + start_sequence;
  for (int jj = 0; jj < LEG_NB; jj++)
  {
    for (int kk = 0; kk < 3; kk++)
    {
      next_pos.legs[jj][kk] = sequences[temp_current_pos][jj][kk];
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