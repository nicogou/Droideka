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
    this->servoBus_front->requestPosition(this->motor_ids[i]);
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
  deg_angle = (encoder_angle - extreme_values_motor[motor_id][1]) * servo_deg_ratio * extreme_values_motor[motor_id][3];

  return deg_angle;
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
  if (parking_updated)
  {
    Action temp_action;
    ErrorCode result;

    result = in_position(*parking_transition_position, temp_action, time);
    if (result == NO_ERROR)
    {
      temp_action.set_active();
      act(&temp_action);
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
      delay(time + offset_time);
    }
    else
    {
      return PARKING_POSITION_IMPOSSIBLE;
    }

    return NO_ERROR;
  }
  else
  {
    return PARKING_POSITION_NOT_UPDATED;
  }
}

ErrorCode Droideka::unpark(int time = 500, int offset_time = 500)
{
  Action unparking;

  ErrorCode result;
  result = in_position(*parking_transition_position, unparking, time);
  if (result == NO_ERROR)
  {
    unparking.set_active();
    act(&unparking);
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
    delay(time + offset_time);
  }
  else
  {
    unparking.set_active(false);
    return STARTING_WALKING_POSITION_IMPOSSIBLE;
  }

  return NO_ERROR;
}

ErrorCode Droideka::walk(int repetitions = 1, int time = 500, int offset_time = 500)
{
  Action walking;
  Droideka_Position next_pos = *starting_position_walking;
  ErrorCode result;
  result = in_position(next_pos, walking, time);
  if (result == NO_ERROR)
  {
    walking.set_active();
    act(&walking);
    delay(time + offset_time);
  }
  else
  {
    walking.set_active(false);
    return result;
  }
  delay(2000);
  for (int rep = 0; rep < repetitions; rep++)
  {
    for (int ii = 0; ii < nb_sequence; ii++)
    {
      for (int jj = 0; jj < LEG_NB; jj++)
      {
        for (int kk = 0; kk < 3; kk++)
        {
          next_pos.legs[jj][kk] = sequence[ii][jj][kk];
        }
      }
      result = in_position(next_pos, walking, time);
      if (result == NO_ERROR)
      {
        walking.set_active();
        act(&walking);
        delay(time + offset_time);
      }
      else
      {
        walking.set_active(false);
        return result;
      }
    }
  }
  return NO_ERROR;
}