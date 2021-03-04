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
    unsigned long last_millis;    // Not currently used.
    unsigned long interval = 100; // Not currently used.

    // Create a Bluetooth Receiver
    Universal_Receiver *droideka_rec;

    // Create a ServoBus instance for the debug Board
    LX16ABus servoBus;
    LX16AServo *servos[MOTOR_NB];

    float hip_length = HIP_LENGTH;                             //L2 -> length from knee to horizontal axis of the hip.
    float tibia_length = TIBIA_LENGTH;                         //L1 -> length from tip of the leg to knee.
    float servo_deg_ratio = SERVO_DEG_RATIO;                   // Multiplier to go from servo encoder to degrees value.
    int32_t deg_to_encoder(int motor_id, float deg_angle);     // Calculates the value to feed the motor from an angle value in degrees to encoder counts
    float encoder_to_deg(int motor_id, int32_t encoder_angle); // Calculates the angle value in degrees from an angle value in encoder counts
    ErrorCode encode_leg_angles(int leg_id);                   // Encodes each motor angle from degrees to encoder counts.

    // Variables to store the wanted motor angles in degrees, radians, and encoder counts.
    float motors_angle_deg[LEG_NB][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};       // Id 0 stores the shoudler angle, Id 1 the Hip angle, Id 2 the Knee angle. All in degrees.
    float motors_angle_rad[LEG_NB][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};       // Id 0 stores the shoudler angle, Id 1 the Hip angle, Id 2 the Knee angle. All in radians.
    int32_t motors_angle_encoder[LEG_NB][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; // Id 0 stores the shoudler angle, Id 1 the Hip angle, Id 2 the Knee angle. All in encoder counts.

    // Motor ids for the Droideka legs
    unsigned int motor_ids[MOTOR_NB] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
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

    // REMOTE CONTROL AND RECEIVER-RELATED FUNCTIONS
    bool receive_data(); // Receives data from receiver.
    State lastServoState;
    State read_servos_positions();

    int throttle_x = 0;
    int throttle_y = 0;
    int button1 = 1;
    int button2 = 1;
    int button3 = 1;
    // IDEA : create a structure similar to the one in Receiver.h to hold the values of throttle_x, throttle_y, button1, button2 and button3.
    // bool forward();   // TRUE if joystick is inclined forward, otherwise FALSE. This should probably be integrated in the Receiver class.
    // bool backward();  // TRUE if joystick is inclined backward, otherwise FALSE. This should probably be integrated in the Receiver class.
    // bool leftward();  // TRUE if joystick is inclined on the left, otherwise FALSE. This should probably be integrated in the Receiver class.
    // bool rightward(); // TRUE if joystick is inclined on the right, otherwise FALSE. This should probably be integrated in the Receiver class.
    // bool button1_pushed();   // TRUE if button 1 is pressed, otherwise FALSE.
    // bool button1_clicked();  // TRUE when button 1 goes from unpressed to pressed, otherwise FALSE.
    // bool button1_released(); // TRUE when button 1 goes from pressed to unpressed, otherwise FALSE.
    // bool button2_pushed();   // TRUE if button 2 is pressed, otherwise FALSE.
    // bool button2_clicked();  // TRUE when button 2 goes from unpressed to pressed, otherwise FALSE.
    // bool button2_released(); // TRUE when button 2 goes from pressed to unpressed, otherwise FALSE.
    // bool button3_pushed();   // TRUE if button 3 is pressed, otherwise FALSE.
    // bool button3_clicked();  // TRUE when button 3 goes from unpressed to pressed, otherwise FALSE.
    // bool button3_released(); // TRUE when button 3 goes from pressed to unpressed, otherwise FALSE.
    // IDEA : create a class or something to hold all remote control-related functions. Or make the Receiver public so we don't have to re-declare public functions ?

    // GENERAL MOVEMENT OF THE ROBOT
    // ErrorCode move(int throttle); // Responds to remote control commands depending on the mode.
    DroidekaMode get_mode(); // Checks in what mode the robot currently is.
    ErrorCode change_mode(); // Goes from walking to rolling mode and vice-versa.

    // ROLLING MODE
    // ErrorCode roll(int speed = 0); // Longitudinal movement of the robot.

    // WALKING MODE
    ErrorCode in_position(Droideka_Position pos, Action &pos_act, int time); // Checks if the wanted position is reachable given the mechanical constraints of the robot.
    void act(Action *action);                                                // Make the motors actually move.

    double last_action_millis = 0; // Time when the previous action was started
    int time_last_action;          // Time the previous action needs to be undertaken in
    int offset_time_last_action;   // Time between end of the previous action and the new one.

    ErrorCode move_into_position(Droideka_Position pos, int time = 0);
    ErrorCode park(int time = 1000);   // Parking routine
    ErrorCode unpark(int time = 1000); // Unparking routine
    ErrorCode go_to_maintenance();
    // ErrorCode walk(int throttle_x, int throttle_y, unsigned long time = 8000000); // Walking routine (time in seconds)
    Droideka_Position get_current_position();
    int walk_compute_state = 0;
    Droideka_Position current_position;
    Droideka_Movement *movement;

    float parked[LEG_NB][3] = {
        {THETA_PARKING, X_PARKING, Y_PARKING},
        {THETA_PARKING, X_PARKING, Y_PARKING},
        {THETA_PARKING, X_PARKING, Y_PARKING},
        {THETA_PARKING, X_PARKING, Y_PARKING}};
    float unparking[LEG_NB][3] = {
        {THETA_IDLE, X_IDLE, Y_NOT_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_NOT_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_NOT_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_NOT_TOUCHING}};
    float unparked[LEG_NB][3] = {
        {THETA_IDLE, X_IDLE, Y_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_TOUCHING},
        {THETA_IDLE, X_IDLE, Y_TOUCHING}};

    float maintenance_pos[LEG_NB][3] = {
        {THETA_MAINTENANCE, X_MAINTENANCE, Y_MAINTENANCE},
        {THETA_MAINTENANCE, X_MAINTENANCE, Y_MAINTENANCE},
        {THETA_MAINTENANCE, X_MAINTENANCE, Y_MAINTENANCE},
        {THETA_MAINTENANCE, X_MAINTENANCE, Y_MAINTENANCE}};

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
};

#endif