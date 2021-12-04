#include "Droideka.h"

Droideka::Droideka(HardwareSerial *serial_servos, int8_t tXpin_servos, int8_t rx, int8_t tx, int16_t thresh[NB_MAX_DATA], String btHardware, int8_t imu_int_pin)
{
  /* Initializes the Droideka.
   *
   * Use only if SoftwareSerial is used to communicate with the Bluetooth Receiver.
   * Should not be used with a Teensy mocrocontroller as there are plenty of Hardware serials.
   *
   *  serial_servos : Serial Port used to communicate with the LX-16A servos.
   * tXpin_servos : TX pin of the Serial port to be used in open drain mode.
   * rx, tx : towards bluetooth receiver
   * thresh : Holds the threshold for the Universal_Receiver lib
   * btHardware : which version of BT receiver you use. specified in Universal_Receiver lib
   */
  initialize(serial_servos, tXpin_servos);
  droideka_rec = new Universal_Receiver(rx, tx, btHardware);
  initialize_imu(imu_int_pin);
  initialize_pid();
  status_led(CRGB::Green);
  delay(1000);
  status_led(CRGB::Black);
  initialize_position();
}

Droideka::Droideka(HardwareSerial *serial_servos, int8_t tXpin_servos, HardwareSerial *serial_receiver, int16_t thresh[NB_MAX_DATA], String btHardware, int8_t imu_int_pin)
{
  /* Initializes the Droideka.
   *
   * Use only if a hardware Serial port is used to communicate with the Bluetooth Receiver.
   *
   * serial_servos : Serial Port used to communicate with the LX-16A servos.
   * tXpin_servos : TX pin of the Serial port to be used in open drain mode.
   * serial_receiver : towards bluetooth receiver
   * thresh : Holds the threshold for the Universal_Receiver lib
   * btHardware : which version of BT receiver you use. specified in Universal_Receiver lib
   */
  initialize(serial_servos, tXpin_servos);
  droideka_rec = new Universal_Receiver(serial_receiver, btHardware);
  initialize_imu(imu_int_pin);
  initialize_pid();
  status_led(CRGB::Green);
  delay(1000);
  status_led(CRGB::Black);
  initialize_position();
}

void Droideka::initialize(HardwareSerial *serial_servos, int8_t tXpin_servos)
{
  /* Initializes the Servos and LED. Also checks battery voltage.
   *
   * serial_servos : Serial Port used to communicate with the LX-16A servos.
   * tXpin_servos : TX pin of the Serial port to be used in open drain mode.
   */
  servoBus.begin(serial_servos, tXpin_servos);
  servoBus.debug(false);
  servoBus.retry = 0;

  for (int8_t ii = 0; ii < MOTOR_LONG_NB; ii++)
  {
    servos[ii] = new LX16AServo(&servoBus, ii);
  }

  prev_status_leds[0] = CRGB::Black;
  leds[0] = CRGB::Black;
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  status_led(CRGB::Black);

  while (check_voltage() != NO_ERROR)
  {
  }
  status_led(CRGB::Blue);

  movement.finished = true;
}

void Droideka::initialize_position()
{
  /* Goes to parked or unparked position depending on position at startup.
   * At startup we don't know what position or mode the robot is in. This checks the mode at startup and moves to the corresponding position.
   */
  DroidekaMode mode = get_mode();
  Droideka_Position curr = get_current_position();
  if (mode == WALKING)
  {
    if (curr == Droideka_Position(unparked))
    {
      current_position = Droideka_Position(unparked);
    }
    else
    {
      unpark();
    }
  }
  else if (mode == ROLLING)
  {
    if (curr == Droideka_Position(parked))
    {
      current_position = Droideka_Position(parked);
    }
    else
    {
      park(1000, false, true);
    }
  }
  current_mode = mode;
}

ErrorCode Droideka::check_voltage(bool overwriting)
{
  /* Battery Voltage checking function. If the battery is low we increase the frequency of the voltage checks.
   *
   * overwriting : true if you want to perform a voltage check immediately.
   */
  if (overwriting || sinceVoltageCheck >= voltage_check_timer)
  {
    sinceVoltageCheck = 0;
    uint32_t tmp = 0;
    min_voltage = 9000;
    for (int8_t ii = 0; ii < MOTOR_LONG_NB; ii++)
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
    avg_voltage = tmp / MOTOR_LONG_NB;

    if (min_voltage < 4500 || max_voltage < SERVOS_UNDER_VOLTAGE_LIMIT)
    {
      disable_motors();
      ErrorCode result = SERVOS_VOLTAGE_TOO_LOW;
      Serial.println("Servo voltage too low. Minimum: " + String(min_voltage) + " - Maximum: " + String(max_voltage));
      status_led(CRGB::Red);
      voltage_check_timer = VOLTAGE_CHECK_TIMER_HIGH_FREQ;
      return result;
    }
    else
    {
      voltage_check_timer = VOLTAGE_CHECK_TIMER;
      status_led(CRGB::Black);
      return NO_ERROR;
    }
  }
  return VOLTAGE_CHECK_NOT_PERFORMED;
}

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
inline void dmp_Data_Ready()
{
  /* Indicates if IMU data is ready.
   */
  mpuInterrupt = true;
}

ErrorCode Droideka::initialize_imu(int8_t imu_interrupt_pin)
{
  /* Initializes the IMU.
   * Almost a copy-paste from examples in MPU6050 (I2C-Dev) lib.
   *
   * imu_interrupt_pin : Teensy pin connected to the MPU6050 INT pin.
   */
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
    status_led(CRGB::Red);
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
    // mpu.PrintActiveOffsets();
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

    calibrate_pitch(5);

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

    status_led(CRGB::Red);
    return MPU_6050_DMP_INIT_FAILED;
  }
}

ErrorCode Droideka::read_imu()
{
  /* Reads the IMU data.
   * Almost a copy-paste from examples in MPU6050 (I2C-Dev) lib.
   */
  // if programming failed, don't try to do anything
  if (!dmpReady)
    return DMP_NOT_READY;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet
    // display Yaw, Pitch and Roll angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
  return NO_ERROR;
}

void Droideka::calibrate_pitch(int nb)
{
  /* Calibrates the pitch, either on a single measurement, or by taking the mean over several measurements. Used at startup, and when in unparked position (as it is assumed to be flat)
   * nb : number of measurements on which to perform a mean. If nb < 1, the calibrated pitch is stored as a single measurement.
   */
  if (nb <= 0)
  {
    nb = 1;
  }
  int ii = 0;

  float avg_pitch = 0.0;
  status_led(CRGB::Yellow);
  while (ii < nb)
  {
    if (read_imu() == NO_ERROR)
    {
      avg_pitch += ypr[2] * 180 / M_PI;
      ii++;
    }
  }
  avg_pitch = avg_pitch / (float)nb;
  calibrated_pitch = avg_pitch;
  status_led(prev_status_led);
}

void Droideka::status_led(CRGB color)
{
  /* Updates the status LED color and stores the previous color if needed.
   * color : the color you want the status LED to be.
   */
  prev_status_leds = leds[0];
  leds[0] = color;
  FastLED.show();
}

void Droideka::initialize_pid()
{
  /* Initializes the PID.
   */
  long_pid->SetOutputLimits(-100, 100);
  long_pid->SetSampleTime(PID_SAMPLE_TIME);
  // Serial.println("Kp:" + String(long_pid->GetKp()) + " Ki:" + String(long_pid->GetKi()) + " Kd:" + String(long_pid->GetKd()));
}

void Droideka::compute_pid()
{
  /* Computes PID Output for longitudinal motor stabilization.
   */
  if (pid_running == true)
  {
    Input = (double)ypr[2] * 180 / M_PI - calibrated_pitch;
  }

  // if (pid_running == true)
  // {
  //   Serial.print(long_pid->GetKp());
  //   Serial.print("\t");
  //   Serial.print(long_pid->GetKi());
  //   Serial.print("\t");
  //   Serial.print(long_pid->GetKd());
  //   Serial.print("\t");
  // }

  double command;
  if (pid_running == true)
  {
    if (long_pid->Compute())
    {
      command = Output;
      roll(command);

      Serial.print(Setpoint);
      Serial.print("\t");
      Serial.print(Input);
      // Serial.print("\t");
      // Serial.print(Output);
      // Serial.print("\t");
      // Serial.print(command);
      Serial.println();
    }
  }
}

void Droideka::start_pid()
{
  /* Puts PID controller in working mode.
   */
  if (pid_running == false && movement.finished == true)
  {
    Serial.println("PID on!");
    Output = 0;
    long_pid->SetMode(AUTOMATIC);
    pid_running = true;
  }
}

void Droideka::stop_pid()
{
  /* Stops PID controller from computing.
   */
  if (pid_running == true)
  {
    Serial.println("PID off!");
    long_pid->SetMode(MANUAL);
    roll(0);
    pid_running = false;
    delayed_function(DISABLE_LONG_SERVOS, 500);
  }
}

bool Droideka::receive_data()
{
  /* Receives data from bluetooth controller.
   */
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
        // Serial.println();
        return true;
      }
    }
    return false;
  }
  return false;
}

ErrorCode Droideka::roll(int speed)
{
  /* Commands the longitudinal motor to go to a specific speed
   *
   * speed : the desired motor speed. Between -100 and 100. Negative values make the motor turn backwards.
   */
  int mapped_speed = map(speed, -100, 100, -1000, 1000);
  // Choose speed
  if (mapped_speed >= -1000 && mapped_speed <= 1000)
  {
    servos[MOTOR_LONG_NB - 1]->motor_mode(mapped_speed);
  }
  else
  {
    return OUT_OF_BOUNDS_SPEED_SPECIFIED;
  }

  return NO_ERROR;
}

void Droideka::disable_motors()
{
  /* Disables all servos (legs and longitudinal)
   */
  for (int ii = 0; ii < MOTOR_LONG_NB; ii++)
  {
    servos[ii]->disable();
  }
}

void Droideka::disable_leg_motors()
{
  /* Disable all leg servos
   */
  for (int ii = 0; ii < MOTOR_NB; ii++)
  {
    servos[ii]->disable();
  }
}

void Droideka::disable_long_motor()
{
  /* Disables the longitudinal motor
   */
  servos[MOTOR_LONG_NB - 1]->disable();
}

State Droideka::read_servos_positions()
{
  /* Reads the last Servo State
   * It will be in encoder values.
   */
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
  /* Makes the robot move the position asked
   *
   * action : This holds the desired position and time span in which to do the move.
   */
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
  /* Checks if the encoder value of each motor is compatible with extremum mechanical values.
   *
   * leg_id : the leg for which to perform the calculations
   */
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
  /* Goes from angle values in degrees to encoder counts.
   *
   * motor_id : The specific motor.
   * deg_angle : the angle in degree to convert.
   */
  int32_t encoder_angle;
  encoder_angle = extreme_values_motor[motor_id][1] + extreme_values_motor[motor_id][3] * (int32_t)(deg_angle / servo_deg_ratio);

  return encoder_angle;
}

float Droideka::encoder_to_deg(int8_t motor_id, int32_t encoder_angle)
{
  /* Goes from angle values in encoder counts to degrees.
   *
   * motor_id : The specific motor.
   * encoder_angle : the angle in encoder counts to convert.
   */
  float deg_angle;
  deg_angle = (float)(encoder_angle - extreme_values_motor[motor_id][1]) * servo_deg_ratio / (float)extreme_values_motor[motor_id][3];

  return deg_angle;
}

DroidekaMode Droideka::get_mode()
{
  /* Checks the robot mode.
   * At startup the mode is undefined as we don't know how the legs are positioned.
   * If all feet are not touching the ground, then it is assumed the robot is in Rolling mode.
   * Else it is in walking mode.
   */
  if (current_mode == UNDEFINED)
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
  else
  {
    return current_mode;
  }
}

ErrorCode Droideka::change_mode()
{
  /* Switches from a mode to another.
   * If the robot is in maintenance mode, then it goes to Rolling mode.
   * Otherwise it switches from Walking to Rolling and vice-versa.
   */
  DroidekaMode mode = get_mode();
  if (mode == MAINTENANCE)
  {
    set_movement(Droideka_Movement(Droideka_Position(maintenance_pos), Droideka_Position(parked), 2000));
    delayed_function(DISABLE_LEG_SERVOS, 3000); // disabling the servos after the parking position is reached. 2*time is needed in theory. 3*time to have a bit of wiggle room.
    current_mode = ROLLING;
  }
  if (mode == WALKING)
  {
    bool over = false;
    if (movement.paused)
    {
      over = true;
    }
    park(1000, over);
    current_mode = ROLLING;
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
    if (!pid_running)
    {
      unpark();
      current_mode = WALKING;
    }
  }
  return NO_ERROR;
}

ErrorCode Droideka::in_position(Droideka_Position pos, Action &pos_act, int time)
{
  /* Transposes a desired position in (theta, rho, z) into an Action to be performed in a specific time span.
   *
   * pos : the desired Droideka_Position
   * pos_act : a placeholder action the will be used in act afterwards.
   * time : the time span.
   */
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

ErrorCode Droideka::move_into_position(Droideka_Position pos, int time)
{
  /* Uses previously defined funcs to move to a desired position in a specific time frame.
   * Also, when the robot is in unparked position, it is the appropriate time to calibrate the IMU, so ti does that as well.
   *
   * pos : the desired Droideka_Position
   * time : the time span.
   */
  if (current_position == Droideka_Position(unparked))
  {
    calibrate_pitch();
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

ErrorCode Droideka::park(int time, bool overwriting, bool skip_middle)
{
  /* Parking routine -> goes from unparked position to parked position for rolling.
   *
   * time : the time in which to perform each moves.
   * overwriting : true if you want to go in parked position no matter what position you are currently in. Used when movement is paused.
   * skip_middle : true if you want to go straight to parked position. false (default) if you want to go through th intermediate position.
   */
  ErrorCode result;
  if (skip_middle == false)
  {
    Droideka_Position curr = get_current_position();
    float temp[LEG_NB][3];
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

    result = set_movement(Droideka_Movement(current_position, temp_pos, time), overwriting);
    if (result == MOVING_THUS_UNABLE_TO_SET_MOVEMENT)
    {
      return MOVING_THUS_UNABLE_TO_SET_MOVEMENT;
    }
    result = add_position(Droideka_Position(parked), time);
    if (result == MOVING_THUS_UNABLE_TO_ADD_POSITION)
    {
      return MOVING_THUS_UNABLE_TO_ADD_POSITION;
    }
  }
  else
  {
    result = set_movement(Droideka_Movement(current_position, Droideka_Position(parked), time), overwriting);
    if (result == MOVING_THUS_UNABLE_TO_ADD_POSITION)
    {
      return MOVING_THUS_UNABLE_TO_ADD_POSITION;
    }
  }
  delayed_function(DISABLE_LEG_SERVOS, 3 * time); // disabling the servos after the parking position is reached. 2*time is needed in theory. 3*time to have a bit of wiggle room.
  return NO_ERROR;
}

ErrorCode Droideka::unpark(int time, bool overwriting, bool skip_middle)
{
  /* Unparking routine -> goes from parked position to unparked position for walking.
   *
   * time : the time in which to perform each moves.
   * overwriting : true if you want to go in unparked position no matter what position you are currently in. Never used at the moment.
   * skip_middle : true if you want to go straight to unparked position. false (default) if you want to go through th intermediate position.
   */

  ErrorCode result;
  if (skip_middle == false)
  {
    result = set_movement(Droideka_Movement(Droideka_Position(parked), Droideka_Position(unparking), time), overwriting);
    if (result == MOVING_THUS_UNABLE_TO_SET_MOVEMENT)
    {
      return MOVING_THUS_UNABLE_TO_SET_MOVEMENT;
    }
    result = add_position(Droideka_Position(unparked), time);
    if (result == MOVING_THUS_UNABLE_TO_ADD_POSITION)
    {
      return MOVING_THUS_UNABLE_TO_ADD_POSITION;
    }
  }
  else
  {
    result = set_movement(Droideka_Movement(current_position, Droideka_Position(unparked), time), overwriting);
    if (result == MOVING_THUS_UNABLE_TO_ADD_POSITION)
    {
      return MOVING_THUS_UNABLE_TO_ADD_POSITION;
    }
  }

  return NO_ERROR;
}

void Droideka::delayed_function()
{
  /* Sometimes, it is useful to be able to perform function at a certain timing after another action has been done.
   * This function checks if there are any function waiting to be performed.
   * Handles only one function at a time.
   */
  if (func != NOTHING)
  {
    if (since_event > event_time_limit)
    {
      if (func == DISABLE_SERVOS)
      {
        disable_motors();
        func = NOTHING;
      }
      else if (func == DISABLE_LEG_SERVOS)
      {
        disable_leg_motors();
        func = NOTHING;
      }
      else if (func == DISABLE_LONG_SERVOS)
      {
        disable_long_motor();
        func = NOTHING;
      }
    }
  }
}

void Droideka::delayed_function(DelayedFunction f, unsigned long t)
{
  /* Sometimes, it is useful to be able to perform function at a certain timing after another action has been done.
   * This function sets up the function and timing.
   * f : the function to be performed
   * t : the amount of time after which it has to be performed.
   */
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
  /* Goes to Maintenance position.
   */
  Droideka_Position maintenance_(maintenance_pos);
  ErrorCode result = set_movement(Droideka_Movement(current_position, maintenance_, 2000));
  delayed_function(DISABLE_SERVOS, 2500); // disabling the servos after the maintenance position is reached. 2*time is needed in theory. 3*time to have a bit of wiggle room.
  current_mode = MAINTENANCE;
  return result;
}

Droideka_Position Droideka::get_current_position()
{
  /* Measures the current leg position the robot is in.
   * Gets the servos last_state, transforms this in rads, then degrees, then in a Droidea_Position instance.
   */
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

void Droideka::stop_movement()
{
  /* Stops movement.
   * Can not be resumed.
   */
  if (movement.started == true && movement.finished == false)
  {
    movement.finished = true;
    movement.seq = STARTING_SEQUENCE;
    movement.next_seq = STARTING_SEQUENCE;
  }
}

void Droideka::pause_movement(bool pause)
{
  /* Pauses movement.
   * Can be resumed.
   */
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
  }
}

ErrorCode Droideka::next_movement()
{
  /* Computes the next position of the movement.
   * If the movement has not started, perform a voltage check to verify we are not asking to move while on too low battery voltage.
   * Then depending on the timing at which the function is called, it takes the appropriate position from the movement to move in, and performs the move with the appropriate timing.
   */
  if (movement.started == false && movement.finished == false)
  {
    if (movement.type != COG_TRAJ)
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
      if (!(current_position == movement.end_position))
      {
        move_into_position(movement.end_position, 0.0);
      }
      movement.finished = true;
      return NO_ERROR;
    }
    if (now - movement.start >= movement.time_iter[movement.iter] && now - movement.start < movement.time_span)
    {
      for (uint8_t ii = movement.iter + 1; ii < movement.nb_iter; ii++)
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
  return NO_ERROR;
}

ErrorCode Droideka::set_movement(Droideka_Movement mvmt, bool overwriting)
{
  /* Sets up the next Droideka_Movement to be performed by the robot.
   * It also checks if the current_position and the starting_position of the D_Movement are matching.
   *
   * mvmt : the movement to be performed
   * overwriting : true if the movement needs to be performed no matter what position the robot is in.
   */
  if (overwriting)
  {
    movement = mvmt;
    delayed_function(NOTHING, 0);
    return NO_ERROR;
  }
  else
  {
    if (movement.started == false || movement.finished == true)
    {
      if (current_position == mvmt.start_position)
      {
        movement = mvmt;
        status_led(CRGB::Black);
        delayed_function(NOTHING, 0);
        return NO_ERROR;
      }
      else
      {
        status_led(CRGB::Blue);
        return CURRENT_POS_AND_STARTING_POS_NOT_MATCHING;
      }
    }
    else
    {
      return MOVING_THUS_UNABLE_TO_SET_MOVEMENT;
    }
  }
}

ErrorCode Droideka::add_position(Droideka_Position pos, unsigned long time)
{
  /* Adds a position to a Droideka_Movement composed of a sequence of Droideka_Position.
   * Only do this when not moving (It could probably be done while moving, but i did ot have a use case for that yet).
   *
   * pos : the position to add
   * time : The time span to go from the previous position in the sequence to the one added.
   */
  if (movement.started == false || movement.finished == true)
  {
    movement.add_position(pos, time);
    return NO_ERROR;
  }
  else
  {
    return MOVING_THUS_UNABLE_TO_ADD_POSITION;
  }
}

ErrorCode Droideka::keep_going()
{
  /* Calls the keep_goign function of Droideka_Movement.
   */
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

void Droideka::next_movement_sequence(MovementSequence ms)
{
  /* While in a STABLE_GAIT or TROT_GAIT move, it is important to know if the movement continues or stops (depending on controller input) as the movement will not be the same.
   * This function specifies the next part of the movement.
   * Warning : This will not update direction changes.
   *
   * ms : the next MovementSequence.
   */
  if ((movement.type == STABLE_GAIT || movement.type == TROT_GAIT) && movement.finished == false)
  {
    movement.next_seq = ms;
  }
}

void Droideka::next_movement_sequence(MovementSequence ms, float next_long, float next_lat, float next_ang)
{
  /* Same as above. However, it specifies the future inputs in case of a direction change while moving.
   *
   * mvmt : the movement to be performed
   * next_long, next_lat, next_ang : holds the future longitudinal/transversal translation and rotation values.
   */
  if ((movement.type == STABLE_GAIT || movement.type == TROT_GAIT) && movement.finished == false)
  {
    movement.next_seq = ms;
    movement.next_longitudinal = next_long;
    movement.next_lateral = next_lat;
    movement.next_angle = next_ang;
  }
}