#include "Droideka.h"

Droideka::Droideka(Stream *debugBoardStream_front)
{
  servoBus_front = new ServoBus(debugBoardStream_front, servo_bus_write_pin_front);
  servoBus_front->setEventHandler(REPLY_POSITION, this->receive_debug_board_position);
}

void Droideka::initialize(int l_m_p_1, int l_m_p_2, int l_m_p_pwm, int rec_rx, int rec_tx, int rec_state)
{
  longitudinal_mot_pin_1 = l_m_p_1;
  longitudinal_mot_pin_2 = l_m_p_2;
  longitudinal_mot_pin_pwm = l_m_p_pwm;

  pinMode(longitudinal_mot_pin_1, OUTPUT);
  pinMode(longitudinal_mot_pin_2, OUTPUT);
  pinMode(longitudinal_mot_pin_pwm, OUTPUT);

  rec.setHardwareSerial(&Serial3);
  rec.setSoftwareSerial(rec_rx, rec_tx);
  rec.start(rec_state);
}

bool Droideka::receive_data()
{
  if (rec.isBtConnected()) // if bluetooth is connected
  {
    if (rec.receivedDataFromController()) // if new data has been collected
    {
      throttle_x = rec.bt_throttle_x;
      throttle_y = rec.bt_throttle_y;
      button1 = rec.bt_but1;
      button2 = rec.bt_but2;
      button3 = rec.bt_but3;
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
  return rec.bt_but1Pushed();
}

bool Droideka::button1_clicked()
{
  return rec.bt_but1Clicked();
}

bool Droideka::button1_released()
{
  return rec.bt_but1Released();
}

bool Droideka::button2_pushed()
{
  return rec.bt_but2Pushed();
}

bool Droideka::button2_clicked()
{
  return rec.bt_but2Clicked();
}

bool Droideka::button2_released()
{
  return rec.bt_but2Released();
}

bool Droideka::button3_pushed()
{
  return rec.bt_but3Pushed();
}

bool Droideka::button3_clicked()
{
  return rec.bt_but3Clicked();
}

bool Droideka::button3_released()
{
  return rec.bt_but3Released();
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
  }
  return &lastState_;
}

void Droideka::act(Action *action)
{
  for (uint8_t i = 0; i < MOTOR_NB; i++)
  {
    if (action->commands[i][2] > 0)
    {
      this->servoBus_front->MoveTime(this->motor_ids[i], action->commands[i][0], action->commands[i][1]);
    }
    else
    {
      this->servoBus_front->SetUnload(this->motor_ids[i]);
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
  DroidekaMode mode = get_mode();
  if (mode == WALKING)
  {
    /* TODO : find out what variable to change with the throttle : the easy one is the speed of the moves, the harder one is the
     *        the easy one is the speed of the moves
     *        the harder one is the reach of the move, i.e. how far the legs go (requires more computation)
     */

    walk(throttle_x, throttle_y);
  }
  else if (mode == ROLLING)
  {
    roll(throttle);
  }
}

DroidekaMode Droideka::get_mode()
{
  Droideka_Position current_position(get_current_position().legs);
  bool test = 1;
  for (int ii = 0; ii < LEG_NB; ii++)
  {
    test = test * (current_position.legs[ii][2] >= Y_NOT_TOUCHING);
  }
  if (test)
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
  DroidekaMode mode = get_mode();
  if (mode == WALKING)
  {
    park();
  }
  else if (mode == ROLLING)
  {
    unpark();
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

ErrorCode Droideka::move_into_position(Droideka_Position pos, int time = 0)
{
  Action action;
  ErrorCode result = in_position(pos, action, time);
  if (result == NO_ERROR)
  {
    action.set_active();
    act(&action);
  }

  return result;
}

ErrorCode Droideka::park(int time = 1000)
{
  current_position = get_current_position();
  for (int ii = 0; ii < LEG_NB; ii++)
  {
    current_position.legs[ii][2] = Y_NOT_TOUCHING;
  }

  ErrorCode result = move_into_position(current_position, time);
  delay(time);

  result = move_into_position(parked, time);
  delay(time);

  return result;
}

ErrorCode Droideka::unpark(int time = 1000)
{
  Droideka_Position unparking_(unparking);
  ErrorCode result = move_into_position(unparking_, time);
  delay(time);

  Droideka_Position unparked_(unparked);
  result = move_into_position(unparked_, time);
  delay(time);

  return result;
}

ErrorCode Droideka::walk(int throttle_x, int throttle_y, unsigned long time = 8000000)
{
  ErrorCode result;

  // Step 1 : Compute the leg moves over the whole movement.
  if (walk_compute_state == 0)
  {
    current_position = get_current_position(); // Gets position of each motor at the start of the movement
    Serial.println("Current Position");
    for (int jj = 0; jj < LEG_NB; jj++)
    {
      for (int kk = 0; kk < 3; kk++)
      {
        Serial.print(current_position.legs[jj][kk]);
        Serial.print("\t");
      }
      Serial.println();
    }
    Serial.print("Valid : ");
    Serial.println(current_position.valid_position);
    Serial.println();
    Serial.println();

    movement = Movement(current_position, throttle_x, throttle_y); // Computes the movement of the CoG and the movement of the legs.
    if (movement.valid_movement)
    {
      walk_compute_state = 1;
      result = NO_ERROR;
    }
    else
    {
      result = INVALID_MOVEMENT;
    }
  }

  // Step 2 : move legs with the right timing.
  if (walk_compute_state == 1)
  {
    //float temp[LEG_NB][3];
    unsigned long time_elapsed = (micros() - movement.start_walk_time) * TIME_SAMPLE / (time); // Watch out for overflow of micros!!!
    if (time_elapsed > TIME_SAMPLE)
    {
      // If the time measured is bigger than the needed time to make a step, the step is finished so walk_compute_state goes back to 1.
      // Furthermore, we cap time_elapsed to TIME_SAMPLE for calculation purposes.
      time_elapsed = TIME_SAMPLE;
      walk_compute_state = 0;
    }

    result = move_into_position(movement.positions[time_elapsed]);
  }
  return result;
}

Droideka_Position Droideka::get_current_position()
{
  // Il faut récupérer le lastState_ des servos, transformer cela en radians, puis degrés, puis en Droideka_Position.
  // State *lastState = read_debug_board_positions();
  State *lastState;
  lastState = read_debug_board_positions();
  // int ii_t = 0;  // Permet de compter le nombre d'itérations nécessaires à une bonne lecture de la position des moteurs.
  while (lastState->correct_motor_reading == false)
  {
    // ii_t++;
    lastState = read_debug_board_positions();
    bool temp = 1;
    for (int ii = 0; ii < MOTOR_NB; ii++)
    {
      temp *= lastState->is_position_updated[ii];
    }
    lastState->correct_motor_reading = temp;
  }
  // Serial.print("Nb de boucles: ");
  // Serial.println(ii_t);

  int angle_deg[MOTOR_NB];
  float temp[LEG_NB][3];

  for (int ii = 0; ii < LEG_NB; ii++)
  {
    shoulder_angle_encoder[ii] = lastState->positions[3 * ii + 0];
    shoulder_angle_deg[ii] = encoder_to_deg(3 * ii + 0, shoulder_angle_encoder[ii]);
    shoulder_angle_rad[ii] = shoulder_angle_deg[ii] * 3.141592 / 180;
    hip_angle_encoder[ii] = lastState->positions[3 * ii + 1];
    hip_angle_deg[ii] = encoder_to_deg(3 * ii + 1, hip_angle_encoder[ii]);
    hip_angle_rad[ii] = hip_angle_deg[ii] * 3.141592 / 180;
    knee_angle_encoder[ii] = lastState->positions[3 * ii + 2];
    knee_angle_deg[ii] = encoder_to_deg(3 * ii + 2, knee_angle_encoder[ii]);
    knee_angle_rad[ii] = knee_angle_deg[ii] * 3.141592 / 180;
    temp[ii][0] = shoulder_angle_deg[ii];
    temp[ii][1] = HIP_LENGTH * cos(hip_angle_rad[ii]) + TIBIA_LENGTH * cos(hip_angle_rad[ii] + knee_angle_rad[ii]);
    temp[ii][2] = HIP_LENGTH * sin(hip_angle_rad[ii]) + TIBIA_LENGTH * sin(hip_angle_rad[ii] + knee_angle_rad[ii]);
  }
  Droideka_Position result(temp);
  return result;
}
