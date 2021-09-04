#include "Droideka.h"

Droideka::Droideka(HardwareSerial *serial_servos, int8_t tXpin_servos, int8_t rx, int8_t tx, int16_t thresh[NB_MAX_DATA], String btHardware, int8_t l_m_p_1, int8_t l_m_p_pwm, int8_t imu_int_pin)
{
  initialize(serial_servos, tXpin_servos, l_m_p_1, l_m_p_pwm);
  droideka_rec = new Universal_Receiver(rx, tx, btHardware);
  initialize_imu(imu_int_pin);
  initialize_pid();
}

Droideka::Droideka(HardwareSerial *serial_servos, int8_t tXpin_servos, HardwareSerial *serial_receiver, int16_t thresh[NB_MAX_DATA], String btHardware, int8_t l_m_p_1, int8_t l_m_p_pwm, int8_t imu_int_pin)
{
  initialize(serial_servos, tXpin_servos, l_m_p_1, l_m_p_pwm);
  droideka_rec = new Universal_Receiver(serial_receiver, btHardware);
  initialize_imu(imu_int_pin);
  initialize_pid();
  digitalWrite(led[info_led], 0);
  digitalWrite(led[ok_led], 1);
  delay(2000);
  digitalWrite(led[ok_led], 0);
}

void Droideka::initialize(HardwareSerial *serial_servos, int8_t tXpin_servos, int8_t l_m_p_1, int8_t l_m_p_pwm)
{
  servoBus.begin(serial_servos, tXpin_servos);
  servoBus.debug(false);
  servoBus.retry = 0;

  for (int8_t ii = 0; ii < MOTOR_NB; ii++)
  {
    servos[ii] = new LX16AServo(&servoBus, ii);
  }

  for (int ii = 0; ii < LED_NB; ii++)
  {
    pinMode(led[ii], OUTPUT);
    digitalWrite(led[ii], 0);
  }

  while (check_voltage() == SERVOS_VOLTAGE_TOO_LOW)
  {
  }
  digitalWrite(led[info_led], 1);

  longitudinal_mot_pin_1 = l_m_p_1;
  longitudinal_mot_pin_pwm = l_m_p_pwm;

  pinMode(longitudinal_mot_pin_1, OUTPUT);
  pinMode(longitudinal_mot_pin_pwm, OUTPUT);

  movement.finished = true;
}

ErrorCode Droideka::check_voltage(bool overwriting = false)
{
  if (overwriting || sinceVoltageCheck >= voltage_check_timer)
  {
    sinceVoltageCheck = 0;
    uint32_t tmp = 0;
    min_voltage = 9000;
    for (int8_t ii = 0; ii < MOTOR_NB; ii++)
    {
      servo_voltage[ii] = servos[ii]->vin();
      tmp += servo_voltage[ii];
      if (servo_voltage[ii] > max_voltage)
      {
        max_voltage = servo_voltage[ii];
      }
      if (servo_voltage[ii] < min_voltage)
      {
        min_voltage = servo_voltage[ii];
      }
    }
    avg_voltage = tmp / MOTOR_NB;

    if (max_voltage < SERVOS_UNDER_VOLTAGE_LIMIT)
    {
      disable_enable_motors();
      ErrorCode result = SERVOS_VOLTAGE_TOO_LOW;
      Serial.println("Servo voltage too low");
      digitalWrite(led[problem_led], 1);
      voltage_check_timer = VOLTAGE_CHECK_TIMER_HIGH_FREQ;
      return result;
    }
    else
    {
      voltage_check_timer = VOLTAGE_CHECK_TIMER;
      digitalWrite(led[problem_led], 0);
      return NO_ERROR;
    }
  }
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
inline void dmp_Data_Ready()
{
  mpuInterrupt = true;
}

ErrorCode Droideka::initialize_imu(int8_t imu_interrupt_pin)
{
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(imu_interrupt_pin, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  if (mpu.testConnection())
  {
    Serial.println(F("MPU6050 connection successful"));
  }
  else
  {
    digitalWrite(led[info_led], 0);
    digitalWrite(led[problem_led], 1);
    Serial.println(F("MPU6050 connection failed"));
    return MPU_6050_CONNECTION_FAILED;
  }

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(X_GYRO_OFFSET);
  mpu.setYGyroOffset(Y_GYRO_OFFSET);
  mpu.setZGyroOffset(Z_GYRO_OFFSET);
  mpu.setXAccelOffset(X_ACCEL_OFFSET);
  mpu.setYAccelOffset(Y_ACCEL_OFFSET);
  mpu.setZAccelOffset(Z_ACCEL_OFFSET);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(imu_interrupt_pin));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(imu_interrupt_pin), dmp_Data_Ready, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    float avg_pitch = 0;
    int nb_measures = 5;
    for (int ii = 0; ii < nb_measures; ii++)
    {
      read_imu();
      avg_pitch += ypr[1] * 180 / M_PI;
      delay(100);
    }
    avg_pitch = avg_pitch / nb_measures;
    calibrated_pitch = avg_pitch;

    return NO_ERROR;
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));

    digitalWrite(led[info_led], 0);
    digitalWrite(led[problem_led], 1);
    return MPU_6050_DMP_INIT_FAILED;
  }
}

void Droideka::read_imu()
{
  // if programming failed, don't try to do anything
  if (!dmpReady)
    return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet
    // display Yaw, Pitch and Roll angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
}

void Droideka::initialize_pid()
{
  pinMode(int_1, INPUT_PULLUP);
  pinMode(int_2, INPUT_PULLUP);
  pinMode(int_3, INPUT_PULLUP);
  pinMode(pot_1, INPUT);
  pinMode(pot_2, INPUT);
  pinMode(pot_3, INPUT);
  long_pid->SetOutputLimits(LONG_MOTOR_DEAD_ZONE - 100, 100 - LONG_MOTOR_DEAD_ZONE);
  Kp = ((double)analogRead(pot_1) - 1023) * 10.0 / (-1023.0);
  Ki = ((double)analogRead(pot_2) - 1023) * 2.0 / (-1023.0);
  Kd = ((double)analogRead(pot_3) - 1023) * 6.0 / (-1023.0);
  Serial.println("Kp:" + String(Kp) + " Ki:" + String(Ki) + " Kd:" + String(Kd));
  // long_pid->SetTunings(Kp, Ki, Kd);
}

void Droideka::compute_pid()
{
  // if (digitalRead(int_1) == 0)
  // {
  // Serial.print("ypr\t");
  // Serial.print(ypr[0] * 180 / M_PI);
  // Serial.print("\t");
  // Serial.print(ypr[1] * 180 / M_PI);
  // Serial.print("\t");
  // Serial.print(ypr[2] * 180 / M_PI);
  // Serial.print("\t");
  // }
  if (pid_running == false)
  {
    // if (digitalRead(int_2) == 0)
    // {
    //   start_pid();
    // }
  }
  if (pid_running == true)
  {
    Input = (double)ypr[1] * 180 / M_PI - calibrated_pitch;
    // if (digitalRead(int_2) == 1)
    // {
    // stop_pid();
    // }
  }

  // Kp = ((double)analogRead(pot_1) - 1023.0) * 10.0 / (-1023.0);
  // Ki = ((double)analogRead(pot_2) - 1023.0) * 2.0 / (-1023.0);
  // Kd = ((double)analogRead(pot_3) - 1023.0) * 6.0 / (-1023.0);

  // if (digitalRead(int_1) == 0)
  // {
  //   Serial.print(Kp);
  //   Serial.print("\t");
  //   Serial.print(Ki);
  //   Serial.print("\t");
  //   Serial.print(Kd);
  //   Serial.print("\t");
  //   Serial.print(long_pid->GetKp());
  //   Serial.print("\t");
  //   Serial.print(long_pid->GetKi());
  //   Serial.print("\t");
  //   Serial.print(long_pid->GetKd());
  //   Serial.print("\t");
  // }

  // if (pid_tunings_updated == false)
  // {
  //   if (digitalRead(int_3) == 0)
  //   {
  //     long_pid->SetTunings(Kp, Ki, Kd);
  //     pid_tunings_updated = true;
  //   }
  // }
  // if (pid_tunings_updated == true)
  // {
  //   if (digitalRead(int_3) == 1)
  //   {
  //     pid_tunings_updated = false;
  //   }
  // }

  double command;
  if (pid_running == true)
  {
    long_pid->Compute();
    if (Output != 0)
    {
      command = Output + LONG_MOTOR_DEAD_ZONE * Output / abs(Output);
    }
    roll(command);
  }
  // if (digitalRead(int_1) == 0)
  // {
  //   Serial.print(Setpoint);
  //   Serial.print("\t");
  //   Serial.print(Input);
  //   Serial.print("\t");
  //   Serial.print(Output);
  //   Serial.print("\t");
  //   Serial.print(command);
  //   Serial.println();
  // }
}

void Droideka::start_pid()
{
  if (pid_running == false)
  {
    Serial.println("PID on!");
    Output = 0;
    long_pid->SetMode(AUTOMATIC);
    pid_running = true;
  }
}

void Droideka::stop_pid()
{
  if (pid_running == true)
  {
    Serial.println("PID off!");
    long_pid->SetMode(MANUAL);
    roll(0);
    pid_running = false;
  }
}

bool Droideka::receive_data()
{
  if (droideka_rec->state())
  {
    if (droideka_rec->receivedData())
    {
      /*
      if (droideka_rec->isUpdated.bluetooth()) // If we received new bluetooth data.
      {
        Serial.print("Bluetooth Inputs: ");
        for (int8_t ii = 0; ii < droideka_rec->rxdata.digitalNb; ii++)
        {
          Serial.print(ii + 1);
          Serial.print(":");
          Serial.print(droideka_rec->digital[ii]);
          Serial.print(", ");
          Serial.print(droideka_rec->lastDigital[ii]);
          Serial.print("\t");
        }
        for (int8_t ii = 0; ii < droideka_rec->rxdata.analogNb; ii++)
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
        for (int8_t ii = 0; ii < droideka_rec->digitalNb_hw; ii++)
        {
          Serial.print(ii + 1);
          Serial.print(":");
          Serial.print(droideka_rec->digital[ii + NB_MAX_DATA]);
          Serial.print(", ");
          Serial.print(droideka_rec->lastDigital[ii + NB_MAX_DATA]);
          Serial.print("\t");
        }
        for (int8_t ii = 0; ii < droideka_rec->analogNb_hw; ii++)
        {
          Serial.print(ii + 1);
          Serial.print(":");
          Serial.print(droideka_rec->analog[ii + NB_MAX_DATA]);
          Serial.print(", ");
          Serial.print(droideka_rec->lastAnalog[ii + NB_MAX_DATA]);
          Serial.print("\t");
        }
      }
    */
      // If we printed something, we go to the next line for the next batch of data.
      if (droideka_rec->isUpdated.bluetooth() || droideka_rec->isUpdated.hardware())
      {
        //Serial.println();
        return true;
      }
    }
    return false;
  }
  return false;
}

ErrorCode Droideka::roll(int speed = 0)
{
  // int pin_1 = longitudinal_mot_pin_1;
  // int pin_pwm = longitudinal_mot_pin_pwm;

  // // Choose forward or backward motion
  if (speed >= 0)
  {
    digitalWrite(longitudinal_mot_pin_1, 1);
  }
  else if (speed < 0)
  {
    digitalWrite(longitudinal_mot_pin_1, 0);
  }

  int mapped_speed = abs(speed);
  mapped_speed = map(mapped_speed, 0, 100, 0, 255);
  // Choose speed
  if (mapped_speed >= 0 && mapped_speed < 256)
  {
    analogWrite(longitudinal_mot_pin_pwm, mapped_speed);
  }
  else
  {
    return OUT_OF_BOUNDS_SPEED_SPECIFIED;
  }

  return NO_ERROR;
}

void Droideka::disable_enable_motors()
{
  for (int ii = 0; ii < MOTOR_NB; ii++)
  {
    servos[ii]->disable();
  }
}

State Droideka::read_servos_positions()
{
  lastServoState.timestamp = millis();
  for (int8_t ii = 0; ii < MOTOR_NB; ii++)
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
  for (int8_t ii = 0; ii < MOTOR_NB; ii++)
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

ErrorCode Droideka::encode_leg_angles(int8_t leg_id)
{

  for (int8_t jj = 0; jj < 3; jj++)
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

int32_t Droideka::deg_to_encoder(int8_t motor_id, float deg_angle)
{
  int32_t encoder_angle;
  encoder_angle = extreme_values_motor[motor_id][1] + extreme_values_motor[motor_id][3] * (int32_t)(deg_angle / servo_deg_ratio);

  return encoder_angle;
}

float Droideka::encoder_to_deg(int8_t motor_id, int32_t encoder_angle)
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
  if (current_position == Droideka_Position(maintenance_pos))
  {
    return MAINTENANCE;
  }
  Droideka_Position curr = get_current_position();
  bool test = 1;
  for (int8_t ii = 0; ii < LEG_NB; ii++)
  {
    test = test * (curr.legs[ii][2] >= Y_NOT_TOUCHING);
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
  if (mode == MAINTENANCE)
  {
    move_into_position(Droideka_Position(parked), 2000);
  }
  if (mode == WALKING)
  {
    bool over = false;
    if (movement.paused)
    {
      over = true;
    }
    park(1000, over);
  }
  else if (mode == ROLLING)
  {
    if (pid_running)
    {
      if (Setpoint != 0)
      {
        Setpoint = 0;
      }
      else
      {
        stop_pid();
      }
    }
    else
    {
      unpark();
    }
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
    for (int8_t ii = 0; ii < LEG_NB; ii++)
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

      motors_angle_deg[ii][1] = motors_angle_rad[ii][1] * 180 / PI;
      motors_angle_deg[ii][2] = motors_angle_rad[ii][2] * 180 / PI;

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
  if (current_position == Droideka_Position(unparked))
  {
    read_imu();
    calibrated_pitch = (double)ypr[1] * 180 / M_PI;
  }
  current_position = pos;
  Action action;
  ErrorCode result = in_position(pos, action, time);
  if (result == NO_ERROR)
  {
    action.set_active();
    act(&action);
  }

  return result;
}

ErrorCode Droideka::park(int time = 1000, bool overwriting = false)
{
  float temp[LEG_NB][3];
  Droideka_Position curr = get_current_position();
  for (int8_t ii = 0; ii < LEG_NB; ii++)
  {
    temp[ii][0] = curr.legs[ii][0];
    temp[ii][1] = curr.legs[ii][1];
    temp[ii][2] = Y_NOT_TOUCHING;
  }

  Droideka_Position temp_pos(temp);
  if (!temp_pos.valid_position)
  {
    return PARKING_TRANSITION_POSITION_IMPOSSIBLE;
  }

  ErrorCode result = set_movement(Droideka_Movement(current_position, Droideka_Position(temp), time), overwriting);
  if (result == MOVING_THUS_UNABLE_TO_SET_MOVEMENT)
  {
    return MOVING_THUS_UNABLE_TO_SET_MOVEMENT;
  }
  result = add_position(Droideka_Position(parked), time);
  if (result == MOVING_THUS_UNABLE_TO_ADD_POSITION)
  {
    return MOVING_THUS_UNABLE_TO_ADD_POSITION;
  }
  delayed_function(DISABLE_SERVOS, 3 * time); // disabling the servos after the parking position is reached. 2*time is needed in theory. 3*time to have a bit of wiggle room.
  return NO_ERROR;
}

ErrorCode Droideka::unpark(int time = 1000, bool overwriting = false)
{
  delayed_function(NOTHING, 0);
  ErrorCode result = set_movement(Droideka_Movement(Droideka_Position(parked), Droideka_Position(unparking), time), overwriting);
  if (result == MOVING_THUS_UNABLE_TO_SET_MOVEMENT)
  {
    return MOVING_THUS_UNABLE_TO_SET_MOVEMENT;
  }
  result = add_position(Droideka_Position(unparked), time);
  if (result == MOVING_THUS_UNABLE_TO_ADD_POSITION)
  {
    return MOVING_THUS_UNABLE_TO_ADD_POSITION;
  }

  return NO_ERROR;
}

void Droideka::delayed_function()
{
  if (func != NOTHING)
  {
    if (since_event > event_time_limit)
    {
      if (func == DISABLE_SERVOS)
      {
        disable_enable_motors();
        func = NOTHING;
      }
    }
  }
}

void Droideka::delayed_function(DelayedFunction f, int t)
{
  if (func == NOTHING || f == NOTHING) // If there are no functions already waiting, or if we need to remove the current delayed function.
  {
    func = f;
    since_event = 0;
    event_time_limit = t;
  }
  else
  {
    // If there is already a function running, we cannot erase it with a new one.
    // We have to be able to handle several delayed functions -> using a table of DelayeFunction.
    // Not needed for the moment but could be a future improvement.
  }
}

ErrorCode Droideka::go_to_maintenance()
{
  Droideka_Position maintenance_(maintenance_pos);
  ErrorCode result = set_movement(Droideka_Movement(current_position, maintenance_, 2000));
  delayed_function(DISABLE_SERVOS, 2500); // disabling the servos after the maintenance position is reached. 2*time is needed in theory. 3*time to have a bit of wiggle room.

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

  for (int8_t ii = 0; ii < LEG_NB; ii++)
  {
    for (int8_t jj = 0; jj < 3; jj++)
    {
      motors_angle_encoder[ii][jj] = lastState.positions[3 * ii + jj];
      motors_angle_deg[ii][jj] = encoder_to_deg(3 * ii + jj, motors_angle_encoder[ii][jj]);
      motors_angle_rad[ii][jj] = motors_angle_deg[ii][jj] * PI / 180;
    }
    temp[ii][0] = motors_angle_deg[ii][0];
    temp[ii][1] = HIP_LENGTH * cos(motors_angle_rad[ii][1]) + TIBIA_LENGTH * cos(motors_angle_rad[ii][1] + motors_angle_rad[ii][2]);
    temp[ii][2] = HIP_LENGTH * sin(motors_angle_rad[ii][1]) + TIBIA_LENGTH * sin(motors_angle_rad[ii][1] + motors_angle_rad[ii][2]);
  }
  Droideka_Position result(temp);
  return result;
}

ErrorCode Droideka::stop_movement()
{
  if (movement.started == true && movement.finished == false)
  {
    movement.finished = true;
    movement.seq = STARTING_SEQUENCE;
    movement.next_seq = STARTING_SEQUENCE;
    return NO_ERROR;
  }
}

ErrorCode Droideka::pause_movement(bool pause = true)
{
  if (movement.started == true && movement.finished == false)
  {
    unsigned long now = micros(); // Watch out for rollover!
    if (movement.paused == false)
    {
      movement.start = now - movement.start;
      movement.paused = true;
      Serial.println("Movement paused!");
    }
    else
    {
      movement.start = now - movement.start;
      movement.paused = false;
      Serial.println("Movement unpaused!");
    }
    return NO_ERROR;
  }
}

ErrorCode Droideka::next_movement()
{
  if (movement.started == false && movement.finished == false)
  {
    if (movement.type != CENTER_OF_GRAVITY_TRAJ)
    {
      ErrorCode volts = check_voltage(true);
      if (volts == SERVOS_VOLTAGE_TOO_LOW)
      {
        if (movement.seq == STARTING_SEQUENCE)
        {
          movement.finished = true;
        }
        else
        {
          movement.paused = true;
        }
        return volts;
      }
      else if (volts == NO_ERROR)
      {
        movement.paused = false;
      }
    }
    movement.next_position = movement.get_future_position(movement.start_position, 0);
    movement.start = micros();
    move_into_position(movement.next_position, movement.time_iter[0] / 1000);
    movement.started = true;
    movement.finished = false;
    movement.iter = 1;
  }
  if (movement.started == true && movement.finished == false)
  {
    if (movement.paused)
    {
      return NO_ERROR;
    }
    unsigned long now = micros();
    if (now - movement.start >= movement.time_span)
    {
      movement.finished = true;
      return NO_ERROR;
    }
    if (now - movement.start >= movement.time_iter[movement.iter] && now - movement.start < movement.time_span)
    {
      for (int8_t ii = movement.iter + 1; ii < movement.nb_iter; ii++)
      {
        if (now - movement.start >= movement.time_iter[ii - 1] && now - movement.start < movement.time_iter[ii])
        {
          movement.iter = ii;
          break;
        }
      }
    }
    if (now - movement.start < movement.time_iter[movement.iter - 1])
    {
      if (movement.next_pos_calc == false)
      {
        movement.next_position = movement.get_future_position(movement.start_position, movement.iter);
        movement.next_pos_calc = true;
      }
    }
    if (now - movement.start >= movement.time_iter[movement.iter - 1] && now - movement.start < movement.time_iter[movement.iter])
    {
      if (movement.next_pos_calc == false)
      {
        movement.next_position = movement.get_future_position(movement.start_position, movement.iter);
      }
      move_into_position(movement.next_position, (movement.start + movement.time_iter[movement.iter] - now) / 1000);
      // current_position.print_position("Current Position " + String(movement.iter));
      movement.next_pos_calc = false;
      movement.iter++;
    }
  }
}

ErrorCode Droideka::set_movement(Droideka_Movement mvmt, bool overwriting = false)
{
  if (overwriting)
  {
    movement = mvmt;
    return NO_ERROR;
  }
  else
  {
    if (movement.started == false || movement.finished == true)
    {
      if (current_position == mvmt.start_position)
      {
        movement = mvmt;
        digitalWrite(led[info_led], 0);
        return NO_ERROR;
      }
      else
      {
        digitalWrite(led[info_led], 1);
        return CURRENT_POS_AND_STARTING_POS_NOT_MATCHING;
      }
    }
    else
    {
      return MOVING_THUS_UNABLE_TO_SET_MOVEMENT;
    }
  }
}

ErrorCode Droideka::add_position(Droideka_Position pos, unsigned long time, int8_t one_leg = -1)
{
  if (movement.started == false || movement.finished == true)
  {
    movement.add_position(current_position, pos, time, one_leg);
    return NO_ERROR;
  }
  else
  {
    return MOVING_THUS_UNABLE_TO_ADD_POSITION;
  }
}

ErrorCode Droideka::keep_going()
{
  if (movement.started == false)
  {
    return NO_ERROR;
  }
  if (movement.started == true && movement.finished == true)
  {
    movement.keep_going();
    return NO_ERROR;
  }
  else
  {
    return MOVEMENT_NOT_FINISHED;
  }
}

ErrorCode Droideka::next_movement_sequence(MovementSequence ms)
{
  if (movement.type == ROBOT_TRAJ && movement.finished == false)
  {
    movement.next_seq = ms;
    return NO_ERROR;
  }
}

ErrorCode Droideka::next_movement_sequence(MovementSequence ms, float next_long, float next_lat, float next_ang)
{
  if (movement.type == ROBOT_TRAJ && movement.finished == false)
  {
    movement.next_seq = ms;
    movement.next_longitudinal = next_long;
    movement.next_lateral = next_lat;
    movement.next_angle = next_ang;

    return NO_ERROR;
  }
}