#include "Droideka.h"

Droideka::Droideka(HardwareSerial *serial_servos, int tXpin_servos, int rx, int tx, int16_t thresh[NB_MAX_DATA], String btHardware, int l_m_p_1, int l_m_p_2, int l_m_p_pwm)
{
  initialize(serial_servos, tXpin_servos, l_m_p_1, l_m_p_2, l_m_p_pwm);
  droideka_rec = new Universal_Receiver(rx, tx, btHardware);
}

Droideka::Droideka(HardwareSerial *serial_servos, int tXpin_servos, HardwareSerial *serial_receiver, int16_t thresh[NB_MAX_DATA], String btHardware, int l_m_p_1, int l_m_p_2, int l_m_p_pwm)
{
  initialize(serial_servos, tXpin_servos, l_m_p_1, l_m_p_2, l_m_p_pwm);
  droideka_rec = new Universal_Receiver(serial_receiver, btHardware);
}

void Droideka::initialize(HardwareSerial *serial_servos, int tXpin_servos, int l_m_p_1, int l_m_p_2, int l_m_p_pwm)
{
  servoBus.begin(serial_servos, tXpin_servos);
  servoBus.debug(false);
  servoBus.retry = 0;

  for (int ii = 0; ii < MOTOR_NB; ii++)
  {
    servos[ii] = new LX16AServo(&servoBus, ii);
  }

  longitudinal_mot_pin_1 = l_m_p_1;
  longitudinal_mot_pin_2 = l_m_p_2;
  longitudinal_mot_pin_pwm = l_m_p_pwm;

  pinMode(longitudinal_mot_pin_1, OUTPUT);
  pinMode(longitudinal_mot_pin_2, OUTPUT);
  pinMode(longitudinal_mot_pin_pwm, OUTPUT);
}

bool Droideka::receive_data()
{
  if (droideka_rec->state())
  {
    if (droideka_rec->receivedData())
    {
      if (droideka_rec->isUpdated.bluetooth()) // If we received new bluetooth data.
      {
        Serial.print("\t\tBluetooth Inputs: ");
        for (int ii = 0; ii < droideka_rec->rxdata.digitalNb; ii++)
        {
          Serial.print(ii + 1);
          Serial.print(":");
          Serial.print(droideka_rec->digital[ii]);
          Serial.print(", ");
          Serial.print(droideka_rec->lastDigital[ii]);
          Serial.print("\t");
        }
        for (int ii = 0; ii < droideka_rec->rxdata.analogNb; ii++)
        {
          Serial.print(ii + 1);
          Serial.print(":");
          Serial.print(droideka_rec->analog[ii]);
          Serial.print(", ");
          Serial.print(droideka_rec->lastAnalog[ii]);
          Serial.print("\t");
        }
      }
      if (droideka_rec->isUpdated.hardware()) // If we received new hardware data.
      {
        Serial.print("\t\tHardware Inputs: ");
        for (int ii = 0; ii < droideka_rec->digitalNb_hw; ii++)
        {
          Serial.print(ii + 1);
          Serial.print(":");
          Serial.print(droideka_rec->digital[ii + NB_MAX_DATA]);
          Serial.print(", ");
          Serial.print(droideka_rec->lastDigital[ii + NB_MAX_DATA]);
          Serial.print("\t");
        }
        for (int ii = 0; ii < droideka_rec->analogNb_hw; ii++)
        {
          Serial.print(ii + 1);
          Serial.print(":");
          Serial.print(droideka_rec->analog[ii + NB_MAX_DATA]);
          Serial.print(", ");
          Serial.print(droideka_rec->lastAnalog[ii + NB_MAX_DATA]);
          Serial.print("\t");
        }
      }

      // If we printed something, we go to the next line for the next batch of data.
      if (droideka_rec->isUpdated.bluetooth() || droideka_rec->isUpdated.hardware())
      {
        Serial.println();
        return true;
      }
    }
    return false;
  }
  return false;
}

// ErrorCode Droideka::roll(int speed = 0)
// {
//   int pin_1 = longitudinal_mot_pin_1;
//   int pin_2 = longitudinal_mot_pin_2;
//   int pin_pwm = longitudinal_mot_pin_pwm;

//   // Choose forward or backward motion
//   if (speed > 0)
//   {
//     digitalWrite(pin_1, 1);
//     digitalWrite(pin_2, 0);
//   }
//   else if (speed < 0)
//   {
//     digitalWrite(pin_1, 0);
//     digitalWrite(pin_2, 1);
//   }
//   else
//   {
//     digitalWrite(pin_1, 0);
//     digitalWrite(pin_2, 0);
//   }

//   int mapped_speed = abs(speed);
//   mapped_speed = map(mapped_speed, 0, 100, 0, 255);
//   // Choose speed
//   if (mapped_speed >= 0 && mapped_speed < 256)
//   {
//     analogWrite(pin_pwm, mapped_speed);
//   }
//   else
//   {
//     return OUT_OF_BOUNDS_SPEED_SPECIFIED;
//   }

//   return NO_ERROR;
// }

State Droideka::read_servos_positions()
{
  lastServoState.timestamp = millis();
  for (int ii = 0; ii < MOTOR_NB; ii++)
  {
    lastServoState.positions[ii] = servos[ii]->pos_read();
    lastServoState.is_position_updated[ii] = true;
  }

  // A mettre à jour
  lastServoState.correct_motor_reading = true;

  return lastServoState;
}

void Droideka::act(Action *action)
{
  for (int ii = 0; ii < MOTOR_NB; ii++)
  {
    if (action->activate[ii] > 0)
    {
      servos[ii]->move_time(action->angle[ii], action->span[ii]);
    }
    else
    {
      servos[ii]->disable();
    }
  }
}

ErrorCode Droideka::encode_leg_angles(int leg_id)
{

  for (int jj = 0; jj < 3; jj++)
  {
    motors_angle_encoder[leg_id][jj] = deg_to_encoder(3 * leg_id + jj, motors_angle_deg[leg_id][jj]);
  }

  if (motors_angle_encoder[leg_id][0] < extreme_values_motor[3 * leg_id + 0][0] || motors_angle_encoder[leg_id][0] > extreme_values_motor[3 * leg_id + 0][2])
  {
    return OUT_OF_BOUNDS_SHOULDER_ANGLE;
  }
  if (motors_angle_encoder[leg_id][1] < extreme_values_motor[3 * leg_id + 1][0] || motors_angle_encoder[leg_id][1] > extreme_values_motor[3 * leg_id + 1][2])
  {
    return OUT_OF_BOUNDS_HIP_ANGLE;
  }
  if (motors_angle_encoder[leg_id][2] < extreme_values_motor[3 * leg_id + 2][0] || motors_angle_encoder[leg_id][2] > extreme_values_motor[3 * leg_id + 2][2])
  {
    return OUT_OF_BOUNDS_KNEE_ANGLE;
  }

  return NO_ERROR;
}

int32_t Droideka::deg_to_encoder(int motor_id, float deg_angle)
{
  int32_t encoder_angle;
  encoder_angle = extreme_values_motor[motor_id][1] + extreme_values_motor[motor_id][3] * (int32_t)(deg_angle / servo_deg_ratio);

  return encoder_angle;
}

float Droideka::encoder_to_deg(int motor_id, int32_t encoder_angle)
{
  float deg_angle;
  deg_angle = (float)(encoder_angle - extreme_values_motor[motor_id][1]) * servo_deg_ratio / (float)extreme_values_motor[motor_id][3];

  return deg_angle;
}

// ErrorCode Droideka::move(int throttle)
// {
//   DroidekaMode mode = get_mode();
//   if (mode == WALKING)
//   {
//     /* TODO : find out what variable to change with the throttle : the easy one is the speed of the moves, the harder one is the
//      *        the easy one is the speed of the moves
//      *        the harder one is the reach of the move, i.e. how far the legs go (requires more computation)
//      */

//     walk(throttle_x, throttle_y);
//   }
//   else if (mode == ROLLING)
//   {
//     roll(throttle);
//   }
//   return NO_ERROR;
// }

DroidekaMode Droideka::get_mode()
{
  Droideka_Position current_position = get_current_position();
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
  return NO_ERROR;
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
      motors_angle_deg[ii][0] = pos.legs[ii][0];

      motors_angle_rad[ii][2] = acos((pos.legs[ii][1] * pos.legs[ii][1] + pos.legs[ii][2] * pos.legs[ii][2] - hip_length * hip_length - tibia_length * tibia_length) / (2 * hip_length * tibia_length));

      // The knee angle can be either positive or negative. When the Droideka is walking, we want the knee angle to be negative, but when the Droideka is in parking position, we want the knee angle to be positive.
      if (pos.legs[ii][2] > 0)
      {
        knee_angle_sign = 1;
      }
      else
      {
        knee_angle_sign = -1;
      }
      motors_angle_rad[ii][2] = knee_angle_sign * motors_angle_rad[ii][2];

      motors_angle_rad[ii][1] = atan(pos.legs[ii][2] / pos.legs[ii][1]) - atan(tibia_length * sin(motors_angle_rad[ii][2]) / (hip_length + tibia_length * cos(motors_angle_rad[ii][2])));

      motors_angle_deg[ii][1] = motors_angle_rad[ii][1] * 180 / 3.141592;
      motors_angle_deg[ii][2] = motors_angle_rad[ii][2] * 180 / 3.141592;

      ErrorCode error = encode_leg_angles(ii);
      if (error != NO_ERROR)
      {
        return error;
      }

      pos_act.angle[3 * ii + 0] = motors_angle_encoder[ii][0];
      pos_act.angle[3 * ii + 1] = motors_angle_encoder[ii][1];
      pos_act.angle[3 * ii + 2] = motors_angle_encoder[ii][2];
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
  if (result != NO_ERROR)
  {
    return result;
  }
  delay(time);

  result = move_into_position(parked, time);
  delay(time);

  return result;
}

ErrorCode Droideka::unpark(int time = 1000)
{
  Droideka_Position unparking_(unparking);
  ErrorCode result = move_into_position(unparking_, time);
  if (result != NO_ERROR)
  {
    return result;
  }
  delay(time);

  Droideka_Position unparked_(unparked);
  result = move_into_position(unparked_, time);
  delay(time);

  return result;
}

Droideka_Position Droideka::get_current_position()
{
  // Il faut récupérer le lastState_ des servos, transformer cela en radians, puis degrés, puis en Droideka_Position.
  State lastState = read_servos_positions();

  while (lastState.correct_motor_reading == false)
  {
    lastState = read_servos_positions();
    bool temp = 1;
    for (int ii = 0; ii < MOTOR_NB; ii++)
    {
      temp *= lastState.is_position_updated[ii];
    }
    lastState.correct_motor_reading = temp;
  }

  float temp[LEG_NB][3];

  for (int ii = 0; ii < LEG_NB; ii++)
  {
    for (int jj = 0; jj < 3; jj++)
    {
      motors_angle_encoder[ii][jj] = lastState.positions[3 * ii + jj];
      motors_angle_deg[ii][jj] = encoder_to_deg(3 * ii + jj, motors_angle_encoder[ii][jj]);
      motors_angle_rad[ii][jj] = motors_angle_deg[ii][jj] * 3.141592 / 180;
    }
    temp[ii][0] = motors_angle_deg[ii][0];
    temp[ii][1] = HIP_LENGTH * cos(motors_angle_rad[ii][1]) + TIBIA_LENGTH * cos(motors_angle_rad[ii][1] + motors_angle_rad[ii][2]);
    temp[ii][2] = HIP_LENGTH * sin(motors_angle_rad[ii][1]) + TIBIA_LENGTH * sin(motors_angle_rad[ii][1] + motors_angle_rad[ii][2]);
  }
  Droideka_Position result(temp);
  return result;
}
