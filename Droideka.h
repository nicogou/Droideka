#ifndef Droideka_h
#define Droideka_h

#include <Universal_Receiver.h>
#include <lx16a-servo.h>
#include "utils/constants.h"
//#include "utils/structs.h"
#include <Droideka_Position.h>
#include <Droideka_Movement.h>
#include <math.h>

class Droideka
{
public:
    // Create a Bluetooth Receiver
    Universal_Receiver *droideka_rec;

    // Create a ServoBus instance for the debug Board
    LX16ABus servoBus;
    LX16AServo *servos[MOTOR_NB];
    void disable_enable_motors();
    uint16_t avg_voltage = 0;         // Holds the average input voltage of the servos in millivolts.
    uint16_t min_voltage = 0;         // Holds the minimum input voltage of the servos in millivolts.
    uint16_t max_voltage = 0;         // Holds the maximum input voltage of the servos in millivolts.
    uint16_t servo_voltage[MOTOR_NB]; // Holds the input voltage of the servos in millivolts.

    const float hip_length = HIP_LENGTH;                       //L2 -> length from knee to horizontal axis of the hip.
    const float tibia_length = TIBIA_LENGTH;                   //L1 -> length from tip of the leg to knee.
    const float servo_deg_ratio = SERVO_DEG_RATIO;             // Multiplier to go from servo encoder to degrees value.
    int32_t deg_to_encoder(int motor_id, float deg_angle);     // Calculates the value to feed the motor from an angle value in degrees to encoder counts
    float encoder_to_deg(int motor_id, int32_t encoder_angle); // Calculates the angle value in degrees from an angle value in encoder counts
    ErrorCode encode_leg_angles(int leg_id);                   // Encodes each motor angle from degrees to encoder counts.

    // Variables to store the wanted motor angles in degrees, radians, and encoder counts.
    float motors_angle_deg[LEG_NB][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};       // Id 0 stores the shoudler angle, Id 1 the Hip angle, Id 2 the Knee angle. All in degrees.
    float motors_angle_rad[LEG_NB][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};       // Id 0 stores the shoudler angle, Id 1 the Hip angle, Id 2 the Knee angle. All in radians.
    int32_t motors_angle_encoder[LEG_NB][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; // Id 0 stores the shoudler angle, Id 1 the Hip angle, Id 2 the Knee angle. All in encoder counts.

    // Motor ids for the Droideka legs
    const unsigned int motor_ids[MOTOR_NB] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    // IDs 0, 1 and 2 represent the front left leg
    // IDs 3, 4 and 5 represent the front right leg
    // IDs 6, 7 and 8 represent the rear left leg
    // IDs 9, 10 and 11 represent the rear right leg

    // Longitudinal Motor
    int longitudinal_mot_pin_1;   // This pi and the following one are used to set the way the longitudinal motor spins.
    int longitudinal_mot_pin_2;   // They can also be used to brake the motor.
    int longitudinal_mot_pin_pwm; // This pin is used to send PWM commands to the longitudinal motor and thus set the speed

    Droideka(HardwareSerial *serial_servos, int tXpin_servos, int rx, int tx, int16_t thresh[NB_MAX_DATA * 2], String btHardware, int l_m_p_1, int l_m_p_2, int l_m_p_pwm);                  // Class constructor.
    Droideka(HardwareSerial *serial_servos, int tXpin_servos, HardwareSerial *serial_receiver, int16_t thresh[NB_MAX_DATA * 2], String btHardware, int l_m_p_1, int l_m_p_2, int l_m_p_pwm); // Class constructor.
    void initialize(HardwareSerial *serial_servos, int tXpin_servos, int l_m_p_1, int l_m_p_2, int l_m_p_pwm);                                                                               // Class initializer. Sets up motors.
    // Not all pins on the Mega and Mega 2560 support change interrupts, so only the following can be used for RX: 10, 11, 12, 13, 14, 15, 50, 51, 52, 53, A8 (62), A9 (63), A10 (64), A11 (65), A12 (66), A13 (67), A14 (68), A15 (69).
    ErrorCode check_voltage();

    // REMOTE CONTROL AND RECEIVER-RELATED FUNCTIONS
    bool receive_data(); // Receives data from receiver.
    State lastServoState;
    State read_servos_positions();

    // GENERAL MOVEMENT OF THE ROBOT
    // ErrorCode move(int throttle); // Responds to remote control commands depending on the mode.
    DroidekaMode get_mode(); // Checks in what mode the robot currently is.
    ErrorCode change_mode(); // Goes from walking to rolling mode and vice-versa.

    // ROLLING MODE
    // ErrorCode roll(int speed = 0); // Longitudinal movement of the robot.

    // WALKING MODE
    ErrorCode in_position(Droideka_Position pos, Action &pos_act, int time); // Checks if the wanted position is reachable given the mechanical constraints of the robot.
    void act(Action *action);                                                // Make the motors actually move.

    ErrorCode move_into_position(Droideka_Position pos, int time = 0);
    ErrorCode park(int time = 1000);   // Parking routine
    ErrorCode unpark(int time = 1000); // Unparking routine
    ErrorCode go_to_maintenance();
    // ErrorCode walk(int throttle_x, int throttle_y, unsigned long time = 8000000); // Walking routine (time in seconds)
    Droideka_Position get_current_position();
    ErrorCode set_movement(Droideka_Movement mvmt, bool overwriting = false);
    ErrorCode next_movement();
    ErrorCode stop_movement();
    ErrorCode add_position(Droideka_Position pos, unsigned long time, int one_leg = -1);

    const float parked[LEG_NB][3] = {
        {THETA_PARKING, X_PARKING, Y_PARKING},
        {THETA_PARKING, X_PARKING, Y_PARKING},
        {THETA_PARKING, X_PARKING, Y_PARKING},
        {THETA_PARKING, X_PARKING, Y_PARKING}};
    const float unparking[LEG_NB][3] = {
        {THETA_IDLE, X_IDLE, Y_NOT_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_NOT_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_NOT_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_NOT_TOUCHING}};
    const float unparked[LEG_NB][3] = {
        {THETA_IDLE, X_IDLE, Y_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_TOUCHING}};

    const float maintenance_pos[LEG_NB][3] = {
        {THETA_MAINTENANCE, X_MAINTENANCE, Y_MAINTENANCE},
        {THETA_MAINTENANCE, X_MAINTENANCE, Y_MAINTENANCE},
        {THETA_MAINTENANCE, X_MAINTENANCE, Y_MAINTENANCE},
        {THETA_MAINTENANCE, X_MAINTENANCE, Y_MAINTENANCE}};

    Droideka_Position current_position = Droideka_Position(parked);

    // The following holds the minimum, middle and maximum values possible for the motors due to mechanical constraints.
    // The last parameter on each line represents the way of reading the encoder values (90degrees is maximum or minimum encoder counts value).
    const int32_t extreme_values_motor[MOTOR_NB][4] = {
        {2520, 11520, 20520, 1},
        {9480, 12000, 22800, -1},
        {0, 12000, 22800, -1},
        {3000, 12000, 21000, -1},
        {1200, 12000, 14520, 1},
        {0, 12000, 24000, 1},
        {3240, 12240, 21240, -1},
        {1200, 12000, 14520, 1},
        {0, 12000, 24000, 1},
        {3240, 12240, 21240, 1},
        {9480, 12000, 22800, -1},
        {0, 12000, 24000, -1},
    };

private:
    Droideka_Movement movement;
};

#endif