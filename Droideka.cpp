#include "Droideka.h"

Droideka::Droideka(Stream *debugBoardStream_front, Stream *debugBoardStream_back)
{
  servoBus_front = new ServoBus(debugBoardStream_front, servo_bus_write_pin_front);
  servoBus_back = new ServoBus(debugBoardStream_back, servo_bus_write_pin_back);
  servoBus_front->setEventHandler(REPLY_POSITION, this->receive_debug_board_position);
  servoBus_back->setEventHandler(REPLY_POSITION, this->receive_debug_board_position);

  for (int ii = 0; ii < TIME_SAMPLE; ii++)
  {
    t[ii] = ii;
  }
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

    walk(throttle_x, throttle_y);
  }
  else if (get_mode() == ROLLING)
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
  if (get_mode() == WALKING)
  {
    park();
  }
  else if (get_mode() == ROLLING)
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
  Droideka_Position current_position(get_current_position().legs);
  for (int ii = 0; ii < LEG_NB; ii++)
  {
    current_position.legs[ii][2] = Y_PARKING;
  }

  ErrorCode result = move_into_position(current_position);

  result = move_into_position(parked);
  return result;
}

ErrorCode Droideka::unpark(int time = 1000)
{
  Droideka_Position unparking(unparking);
  ErrorCode result = move_into_position(unparking);

  Droideka_Position unparked(unparked);
  result = move_into_position(unparked);
  return result;
}

ErrorCode Droideka::walk(int throttle_x, int throttle_y, unsigned long time = 8000000)
{
  ErrorCode result;

  // Step 1 : determiner tx(t), ty(t) et alpha(t) correspondant au centre de gravité du robot, l'ordre de déplacement des pattes, et prévoir le déplacement de chaque patte.
  if (walk_compute_state == 0)
  {
    establish_cog_movement();
    current_position = get_current_position();        // Gets position of each motor at the start of the movement
    final_pos = get_final_position(current_position); // Calculates position of the legs if the CoG moves the expected amount (but the legs themselves don't move).
    establish_legs_movement();
    walk_compute_state = 1;
    result = NO_ERROR;
    start_walk_time = micros();
  }

  // Step 2 : déterminer le déplacement de chaque patte et effectuer le déplacement.
  if (walk_compute_state == 1)
  {
    //float temp[LEG_NB][3];
    unsigned long time_elapsed = (micros() - start_walk_time) * TIME_SAMPLE / (time);
    if (time_elapsed > TIME_SAMPLE)
    {
      // If the time measured is bigger than the needed time to make a step, the step is finished so walk_compute_state goes back to 1.
      // Furthermore, we cap time_elapsed to TIME_SAMPLE for calculation purposes.
      time_elapsed = TIME_SAMPLE;
      walk_compute_state = 0;
    }

    result = move_into_position(movement[time_elapsed]);
  }
  return result;
}

ErrorCode Droideka::establish_cog_movement()
{
  if (forward())
  {
    for (int ii = 0; ii < TIME_SAMPLE; ii++)
    {
      tx[ii] = MAX_LONGITUDINAL_COG_MOVE * ii / TIME_SAMPLE;
      ty[ii] = 0;
      alpha[ii] = 0;
    }
    leg_order[3] = 1;
    leg_order[1] = 2;
    leg_order[2] = 3;
    leg_order[0] = 4;

    for (int ii = 0; ii < LEG_NB; ii++)
    {
      leg_lifted[ii] = false;
    }
    moving_leg_nb = 4;
    delta_time = TIME_SAMPLE / (moving_leg_nb * 4);
  }

  for (int ii = 0; ii < TIME_SAMPLE; ii++)
  {
    reverse_tx[ii] = tx[ii] * -1;
    reverse_ty[ii] = ty[ii] * -1;
    reverse_alpha[ii] = alpha[ii] * -1;
  }

  return NO_ERROR;
}

Droideka_Position Droideka::get_current_position()
{
  // Il faut récupérer le lastState_ des servos, transformer cela en radians, puis degrés, puis en Droideka_Position.
  State *lastState = read_debug_board_positions();
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

Droideka_Position Droideka::get_future_position(Droideka_Position start_pos, float trans_x[TIME_SAMPLE], float trans_y[TIME_SAMPLE], float angle[TIME_SAMPLE], unsigned long time_elapsed, int one_leg = -1)
{
  if (time_elapsed < 0 || time_elapsed > TIME_SAMPLE)
  {
    return start_pos; // Not sure this is right if time_elapsed > TIME_SAMPLE.
  }

  float temp[LEG_NB][2];           // x and y final coordinates of each feet
  float temp_final_pos[LEG_NB][3]; // used to build the Droideka_Position object by calculating rho, theta and z thanks to x and y stored in temp.

  for (int ii = 0; ii < LEG_NB; ii++)
  {
    if (one_leg != -1)
    {
      ii = one_leg;
    }

    temp[ii][0] = shoulder_pos[ii][0] * (cos(angle[time_elapsed]) - 1) + shoulder_pos[ii][1] * sin(angle[time_elapsed]) + shoulder_mult[ii][0] * sqrt((start_pos.legs[ii][1] * cos(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][0] * trans_x[time_elapsed]) * (start_pos.legs[ii][1] * cos(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][0] * trans_x[time_elapsed]) + (start_pos.legs[ii][1] * sin(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][1] * trans_y[time_elapsed]) * (start_pos.legs[ii][1] * sin(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][1] * trans_y[time_elapsed])) * cos(shoulder_mult[ii][0] * shoulder_mult[ii][1] * atan((start_pos.legs[ii][1] * cos(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][0] * trans_x[time_elapsed]) / (start_pos.legs[ii][1] * sin(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][1] * trans_y[time_elapsed])) - angle[time_elapsed]);
    temp[ii][1] = shoulder_pos[ii][1] * (cos(angle[time_elapsed]) - 1) + shoulder_pos[ii][0] * sin(angle[time_elapsed]) + shoulder_mult[ii][0] * sqrt((start_pos.legs[ii][1] * cos(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][0] * trans_x[time_elapsed]) * (start_pos.legs[ii][1] * cos(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][0] * trans_x[time_elapsed]) + (start_pos.legs[ii][1] * sin(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][1] * trans_y[time_elapsed]) * (start_pos.legs[ii][1] * sin(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][1] * trans_y[time_elapsed])) * sin(shoulder_mult[ii][0] * shoulder_mult[ii][1] * atan((start_pos.legs[ii][1] * cos(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][0] * trans_x[time_elapsed]) / (start_pos.legs[ii][1] * sin(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][1] * trans_y[time_elapsed])) - angle[time_elapsed]);
    temp_final_pos[ii][2] = start_pos.legs[ii][2];
    temp_final_pos[ii][1] = sqrt(temp[ii][0] * temp[ii][0] + temp[ii][1] * temp[ii][1]);
    if (temp_final_pos[ii][1] == 0) // Si x et y sont nuls
    {
      temp_final_pos[ii][0] = 0; // TODO : réfléchir à cette valeur. Est-il possible de déterminer theta si x et y sont nuls?
    }
    else if (temp[ii][0] == 0) // Si rho est non nul, alors si x est nul, y est non nul et on peut diviser par y.
    {
      temp_final_pos[ii][0] = temp[ii][1] / abs(temp[ii][1]) * 90; // Si x est nul, rho vaut + ou - 90°, determiné par le signe de y.
    }
    else
    {
      temp_final_pos[ii][0] = atan(temp[ii][1] / temp[ii][0]); // Dans le cas général, tan(theta) = y/x.
    }

    if (one_leg != -1)
    {
      ii = LEG_NB;
    }
  }
  Droideka_Position final_pos(temp_final_pos);
  return final_pos;
}

Droideka_Position Droideka::get_final_position(Droideka_Position start_pos)
{
  return get_future_position(start_pos, tx, ty, alpha, TIME_SAMPLE);
}

Droideka_Position Droideka::get_lifted_position(int leg, Droideka_Position start_pos, Droideka_Position end_pos, unsigned long time_)
{
  unsigned long debut_time = (leg_order[leg] - 1) * TIME_SAMPLE / moving_leg_nb + delta_time;
  unsigned long fin_time = leg_order[leg] * TIME_SAMPLE / moving_leg_nb;
  unsigned long mid_time = (debut_time + fin_time) / 2; // Not used.
  unsigned long interval_time = fin_time - debut_time;
  unsigned long time_from_lifting = time_ - debut_time;

  Droideka_Position debut_pos(get_future_position(start_pos, tx, ty, alpha, debut_time, leg).legs);
  Droideka_Position fin_pos(get_future_position(end_pos, reverse_tx, reverse_ty, reverse_alpha, TIME_SAMPLE - fin_time, leg).legs);

  // Between the lifting and putting back of the leg, theta and X are linear, wheras Y follows a quadratic curve (arbitrarily defined)

  float temp[LEG_NB][3];
  for (int ii = 0; ii < 2; ii++)
  {
    temp[leg][ii] = (fin_pos.legs[leg][ii] - debut_pos.legs[leg][ii]) / interval_time * time_from_lifting + debut_pos.legs[leg][ii];
  }
  temp[leg][2] = Y_NOT_TOUCHING - (Y_NOT_TOUCHING - Y_TOUCHING) * ((time_from_lifting - interval_time / 2) / (interval_time / 2)) * ((time_from_lifting - interval_time / 2) / (interval_time / 2));

  Droideka_Position result(temp);
  return result;
}

ErrorCode Droideka::establish_legs_movement()
{
  float temp[LEG_NB][3];
  unsigned long time_leg_starts_lifting;
  unsigned long time_leg_touches_ground_again;

  for (int ii = 0; ii < TIME_SAMPLE; ii++)
  {
    Droideka_Position temp_current_pos = get_future_position(current_position, tx, ty, alpha, ii);                               // Calculates the position of the legs before the leg is lifted.
    Droideka_Position temp_future_pos = get_future_position(final_pos, reverse_tx, reverse_ty, reverse_alpha, TIME_SAMPLE - ii); // Calculates the position of the legs after the leg has been lifted and put back on the ground.

    for (int jj = 0; jj < LEG_NB; jj++)
    {
      time_leg_starts_lifting = (leg_order[jj] - 1) * TIME_SAMPLE / moving_leg_nb + delta_time;
      time_leg_touches_ground_again = (leg_order[jj]) * TIME_SAMPLE / moving_leg_nb;

      if (ii <= time_leg_starts_lifting)
      {
        for (int kk = 0; kk < 3; kk++)
        {
          temp[jj][kk] = temp_current_pos.legs[jj][kk];
        }
      }
      else if (ii > time_leg_starts_lifting && ii <= time_leg_touches_ground_again)
      {
        for (int kk = 0; kk < 3; kk++)
        {
          temp[jj][kk] = get_lifted_position(jj, current_position, final_pos, ii).legs[jj][kk];
        }
      }
      else if (ii > time_leg_touches_ground_again && ii <= TIME_SAMPLE)
      {
        for (int kk = 0; kk < 3; kk++)
        {
          temp[jj][kk] = temp_future_pos.legs[jj][kk];
        }
      }
    }
    movement[ii] = Droideka_Position(temp);
  }
  return NO_ERROR;
}
