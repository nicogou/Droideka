#ifndef Droideka_h
#define Droideka_h

#include <Universal_Receiver.h>
#include <lx16a-servo.h>
#include "utils/constants.h"
//#include "utils/structs.h"
#include <Droideka_Position.h>
#include <Droideka_Movement.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "PID_v1.h"

class Droideka
{
public:
    // Create a Bluetooth Receiver
    Universal_Receiver *droideka_rec;

    // Create a ServoBus instance for the debug Board
    LX16ABus servoBus;
    LX16AServo *servos[MOTOR_LONG_NB];
    void disable_enable_motors();
    void disable_leg_motors();
    void disable_long_motor();
    uint16_t avg_voltage = 0;                                    // Holds the average input voltage of the servos in millivolts.
    uint16_t min_voltage = 0;                                    // Holds the minimum input voltage of the servos in millivolts.
    uint16_t max_voltage = 0;                                    // Holds the maximum input voltage of the servos in millivolts.
    uint16_t servo_voltage[MOTOR_LONG_NB];                       // Holds the input voltage of the servos in millivolts.
    int32_t voltage_check_timer = VOLTAGE_CHECK_TIMER_HIGH_FREQ; // holds the time between voltage checks.

    const float hip_length = HIP_LENGTH;                          // L2 -> length from knee to horizontal axis of the hip.
    const float tibia_length = TIBIA_LENGTH;                      // L1 -> length from tip of the leg to knee.
    const float servo_deg_ratio = SERVO_DEG_RATIO;                // Multiplier to go from servo encoder to degrees value.
    int32_t deg_to_encoder(int8_t motor_id, float deg_angle);     // Calculates the value to feed the motor from an angle value in degrees to encoder counts
    float encoder_to_deg(int8_t motor_id, int32_t encoder_angle); // Calculates the angle value in degrees from an angle value in encoder counts
    ErrorCode encode_leg_angles(int8_t leg_id);                   // Encodes each motor angle from degrees to encoder counts.

    // Variables to store the wanted motor angles in degrees, radians, and encoder counts.
    float motors_angle_deg[LEG_NB][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};       // Id 0 stores the shoudler angle, Id 1 the Hip angle, Id 2 the Knee angle. All in degrees.
    float motors_angle_rad[LEG_NB][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};       // Id 0 stores the shoudler angle, Id 1 the Hip angle, Id 2 the Knee angle. All in radians.
    int32_t motors_angle_encoder[LEG_NB][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; // Id 0 stores the shoudler angle, Id 1 the Hip angle, Id 2 the Knee angle. All in encoder counts.

    // Motor ids for the Droideka legs
    const unsigned int motor_ids[MOTOR_LONG_NB] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    // IDs 0, 1 and 2 represent the front left leg
    // IDs 3, 4 and 5 represent the front right leg
    // IDs 6, 7 and 8 represent the rear left leg
    // IDs 9, 10 and 11 represent the rear right leg

    const int led[LED_NB] = {RED_LED, GREEN_LED, BLUE_LED};
    const int problem_led = 0;
    const int ok_led = 1;
    const int info_led = 2;

    // Longitudinal Motor PID
    double Setpoint = 0.0, Input = 0.0, Output = 0.0; // Define PID variables.
    double Kp = 4.0, Ki = 0.0, Kd = 0.0;              // Define tuning parameters.
    PID *long_pid = new PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
    bool pid_running = false;
    bool pid_tunings_updated = false;
    double calibrated_pitch = 0;

    // MPU6050
    // class default I2C address is 0x68
    // specific I2C addresses may be passed as a parameter here
    // AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
    // AD0 high = 0x69
    MPU6050 mpu;
    // MPU control/status vars
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;        // [w, x, y, z]         quaternion container
    VectorFloat gravity; // [x, y, z]            gravity vector
    float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    Droideka(HardwareSerial *serial_servos, int8_t tXpin_servos, int8_t rx, int8_t tx, int16_t thresh[NB_MAX_DATA * 2], String btHardware);            // Class constructor.
    Droideka(HardwareSerial *serial_servos, int8_t tXpin_servos, HardwareSerial *serial_receiver, int16_t thresh[NB_MAX_DATA * 2], String btHardware); // Class constructor.

    void initialize(HardwareSerial *serial_servos, int8_t tXpin_servos); // Starts the Bluetooth controller and servo bus communication.
    ErrorCode initialize_imu(int8_t imu_interrupt_pin);                  // Starts i2c communication with MPU6050
    void read_imu();                                                     // Reads IMU data and stores it in ypr[]
    void initialize_pid();                                               // Starts the PID controller
    void compute_pid();                                                  // Calculates PID Output for the longitudinal motor
    void start_pid();                                                    // Starts the PID when needed
    void stop_pid();                                                     // Stops the PID when needed
    ErrorCode check_voltage(bool overwriting = false);                   // Checks battery voltage : on startup, before a Movement is done, and when a significant amounf of time has passed.
    elapsedMillis sinceVoltageCheck;                                     // Timer since last voltage check.

    // REMOTE CONTROL AND RECEIVER-RELATED FUNCTIONS
    bool receive_data();           // Receives data from Bluetooth controller.
    State lastServoState;          // Stores the last known state of the servos.
    State read_servos_positions(); // Reads the servo positions.

    void delayed_function();                         // Checks if a function must be operated after a certain time after an event. Example : disabling the leg servos after parking.
    void delayed_function(DelayedFunction f, int t); // Sets up the function f to be operated after t ms.
    DelayedFunction func = NOTHING;                  // Delayed function to be operated. Nothing at startup.
    elapsedMillis since_event;                       // Timer for a delayed function
    int event_time_limit = 0;                        // Store time when a delayed function has to be operated.

    // GENERAL MOVEMENT OF THE ROBOT
    DroidekaMode current_mode = UNDEFINED; // Current mode. UNDEFINED at startup before get_mode is called for the first time.
    DroidekaMode get_mode();               // Checks in what mode the robot currently is.
    ErrorCode change_mode();               // Handmes mode switching : Maintenance, walking or rolling.

    // ROLLING MODE
    ErrorCode roll(int speed = 0); // Makes longitudinal motor move to a speed between -100 and 100.

    // WALKING MODE
    ErrorCode in_position(Droideka_Position pos, Action &pos_act, int time); // Checks if the wanted position is reachable given the mechanical constraints of the robot.
    void act(Action *action);                                                // Sends move commands to the leg servos.

    ErrorCode move_into_position(Droideka_Position pos, int time = 0); // Moves into a Droideka_Position in time ms.
    ErrorCode park(int time = 1000, bool overwriting = false);         // Parking routine
    ErrorCode unpark(int time = 1000, bool overwriting = false);       // Unparking routine
    ErrorCode go_to_maintenance();                                     // Going to Maintenance position routine

    Droideka_Position get_current_position();                                                               // Read servo position to get current position. Not super-duper precise so seldom used.
    ErrorCode set_movement(Droideka_Movement mvmt, bool overwriting = false);                               // Sets up a Droideka_Movement for the robot to operate.
    ErrorCode next_movement();                                                                              // Determines at which point of the Droideka_Movement we are and moves accordingly.
    ErrorCode stop_movement();                                                                              // Stops movement. No resuming possible. This is only a software stop !
    ErrorCode pause_movement(bool pause = true);                                                            // Pauses movement. It can be resumed.
    ErrorCode add_position(Droideka_Position pos, unsigned long time);                                      // Adds a position to the Droideka_Movement. Used only if D_Movement is of SEQUENCE type.
    ErrorCode keep_going();                                                                                 // Calls the keep_going function of class D_Movement to see if the steps continue. That only happens in STABLE_GAIT and TROT_GAIT Droideka_Movement types.
    ErrorCode next_movement_sequence(MovementSequence ms);                                                  // In STABLE_GAIT or TROT_GAIT Movement types, indicates if the steps continue or stops.
    ErrorCode next_movement_sequence(MovementSequence ms, float next_long, float next_lat, float next_ang); // Same as above but with different direction.

    // Holds values for the parked position
    const float parked[LEG_NB][3] = {
        {THETA_PARKING, X_PARKING, Y_PARKING},
        {THETA_PARKING, X_PARKING, Y_PARKING},
        {THETA_PARKING, X_PARKING, Y_PARKING},
        {THETA_PARKING, X_PARKING, Y_PARKING}};
    // Holds values for the trnaisiton position between parking and unparking
    const float unparking[LEG_NB][3] = {
        {THETA_IDLE, X_IDLE, Y_NOT_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_NOT_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_NOT_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_NOT_TOUCHING}};
    // Holds values for the unparked position
    const float unparked[LEG_NB][3] = {
        {THETA_IDLE, X_IDLE, Y_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_TOUCHING}};
    // Holds values for the maintenance position
    const float maintenance_pos[LEG_NB][3] = {
        {THETA_MAINTENANCE, X_MAINTENANCE, Y_MAINTENANCE},
        {THETA_MAINTENANCE, X_MAINTENANCE, Y_MAINTENANCE},
        {THETA_MAINTENANCE, X_MAINTENANCE, Y_MAINTENANCE},
        {THETA_MAINTENANCE, X_MAINTENANCE, Y_MAINTENANCE}};

    Droideka_Position current_position = Droideka_Position(parked); // Stores the last position sent to the servos.

    // The following holds the minimum, middle and maximum values possible for the motors due to mechanical constraints.
    // The last parameter on each line represents the way of reading the encoder values (90 degrees is maximum or minimum encoder counts value).
    const int32_t extreme_values_motor[MOTOR_NB][4] = {
        {2520, 11520, 20520, 1},
        {7480, 12000, 22800, -1},
        {0, 7500, 22500, -1},
        {3000, 12240, 21000, -1},
        {1200, 12000, 16520, 1},
        {1500, 16500, 24000, 1},
        {3240, 12240, 21240, -1},
        {1200, 12000, 16520, 1},
        {1500, 16500, 24000, 1},
        {3240, 12000, 21240, 1},
        {7480, 12000, 22800, -1},
        {0, 7500, 22500, -1},
    };

private:
    Droideka_Movement movement; // Holds the Droideka_Movement to be opearted by the robot. This stores information about Center of Gravity's trajectory, precise legs movement etc.
};

#endif