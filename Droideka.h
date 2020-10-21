#ifndef Droideka_h
#define Droideka_h

#include <Receiver.h>
#include <ServoBus.h>
#include <utils/utils.h>
#include <math.h>

class Droideka
{
private:
    unsigned long last_millis;    // Not currently used.
    unsigned long interval = 100; // Not currently used.

    // Create a Bluetooth Receiver
    Receiver rec;

    // Create two ServoBus instances, one for each debug board.
    ServoBus *servoBus_front;                                                                                // Communication with the debug board wired to the front two legs
    ServoBus *servoBus_back;                                                                                 // Communication with the debug board wired to the rear two legs
    int servo_bus_write_pin_front = SERVO_BUS_WRITE_PIN_FRONT;                                               // used by ServoBus lib. I assume it has the same function than Rx and Tx LEDs on the Arduino. Nothing currently wired to the pin.
    int servo_bus_write_pin_back = SERVO_BUS_WRITE_PIN_BACK;                                                 // used by ServoBus lib. I assume it has the same function than Rx and Tx LEDs on the Arduino. Nothing currently wired to the pin.
    static void receive_debug_board_position(uint8_t id, uint8_t command, uint16_t param1, uint16_t param2); // ServoBus Event set to this function. Should work but not tested, and not currently used.

    float hip_length = HIP_LENGTH;                         //L2 -> length from knee to horizontal axis of the hip.
    float tibia_length = TIBIA_LENGTH;                     //L1 -> length from tip of the leg to knee.
    float servo_deg_ratio = 0.24;                          // Multiplier to go from servo encoder to degrees value.
    int deg_to_encoder(int motor_id, float deg_angle);     // Calculates the value to feed the motor from an angle value in degrees to encoder counts
    float encoder_to_deg(int motor_id, int encoder_angle); // Calculates the angle value in degrees from an angle value in encoder counts
    ErrorCode encode_leg_angles(int leg_id);               // Encodes each motor angle from degrees to encoder counts.

    // Variables to store the wanted motor angles in degrees, radians, and encoder counts.
    float shoulder_angle_deg[LEG_NB] = {0, 0, 0, 0};
    float knee_angle_deg[LEG_NB] = {0, 0, 0, 0}; //phi 1
    float hip_angle_deg[LEG_NB] = {0, 0, 0, 0};  //phi 2
    float shoulder_angle_rad[LEG_NB] = {0, 0, 0, 0};
    float knee_angle_rad[LEG_NB] = {0, 0, 0, 0}; //phi 1
    float hip_angle_rad[LEG_NB] = {0, 0, 0, 0};  //phi 2
    int shoulder_angle_encoder[LEG_NB] = {0, 0, 0, 0};
    int knee_angle_encoder[LEG_NB] = {0, 0, 0, 0}; //phi 1
    int hip_angle_encoder[LEG_NB] = {0, 0, 0, 0};  //phi 2

    float t[TIME_SAMPLE];
    float tx[TIME_SAMPLE];
    float ty[TIME_SAMPLE];
    float alpha[TIME_SAMPLE];
    float reverse_tx[TIME_SAMPLE];
    float reverse_ty[TIME_SAMPLE];
    float reverse_alpha[TIME_SAMPLE];
    float shoulder_pos[LEG_NB][2] = {
        {-BODY_WIDTH / 2, BODY_LENGTH / 2},
        {BODY_WIDTH / 2, BODY_LENGTH / 2},
        {-BODY_WIDTH / 2, -BODY_LENGTH / 2},
        {BODY_WIDTH / 2, -BODY_LENGTH / 2}};
    float shoulder_mult[LEG_NB][2] = {
        {-1, 1},
        {1, 1},
        {-1, -1},
        {1, -1}};

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

public:
    Droideka(Stream *debugBoardStream_front, Stream *debugBoardStream_back);                         // Class constructor.
    void initialize(int l_m_p_1, int l_m_p_2, int l_m_p_pwm, int rec_rx, int rec_tx, int rec_state); // Class initializer. Sets up motors and Receiver depending on the setup.
    // Not all pins on the Mega and Mega 2560 support change interrupts, so only the following can be used for RX: 10, 11, 12, 13, 14, 15, 50, 51, 52, 53, A8 (62), A9 (63), A10 (64), A11 (65), A12 (66), A13 (67), A14 (68), A15 (69).

    // REMOTE CONTROL AND RECEIVER-RELATED FUNCTIONS
    bool receive_data(); // Receives data from receiver.
    State *read_debug_board_positions();
    // IDEA: actually use the previous function to check the position of the robot at startup, or if we reached the wanted position.

    int throttle_x = 0;
    int throttle_y = 0;
    int button1 = 1;
    int button2 = 1;
    int button3 = 1;
    // IDEA : create a structure similar to the one in Receiver.h to hold the values of throttle_x, throttle_y, button1, button2 and button3.
    bool forward();          // TRUE if joystick is inclined forward, otherwise FALSE. This should probably be integrated in the Receiver class.
    bool backward();         // TRUE if joystick is inclined backward, otherwise FALSE. This should probably be integrated in the Receiver class.
    bool leftward();         // TRUE if joystick is inclined on the left, otherwise FALSE. This should probably be integrated in the Receiver class.
    bool rightward();        // TRUE if joystick is inclined on the right, otherwise FALSE. This should probably be integrated in the Receiver class.
    bool button1_pushed();   // TRUE if button 1 is pressed, otherwise FALSE.
    bool button1_clicked();  // TRUE when button 1 goes from unpressed to pressed, otherwise FALSE.
    bool button1_released(); // TRUE when button 1 goes from pressed to unpressed, otherwise FALSE.
    bool button2_pushed();   // TRUE if button 2 is pressed, otherwise FALSE.
    bool button2_clicked();  // TRUE when button 2 goes from unpressed to pressed, otherwise FALSE.
    bool button2_released(); // TRUE when button 2 goes from pressed to unpressed, otherwise FALSE.
    bool button3_pushed();   // TRUE if button 3 is pressed, otherwise FALSE.
    bool button3_clicked();  // TRUE when button 3 goes from unpressed to pressed, otherwise FALSE.
    bool button3_released(); // TRUE when button 3 goes from pressed to unpressed, otherwise FALSE.
    // IDEA : create a class or something to hold all remote control-related functions. Or make the Receiver public so we don't have to re-declare public functions ?

    // GENERAL MOVEMENT OF THE ROBOT
    ErrorCode move(int throttle); // Responds to remote control commands depending on the mode.
    DroidekaMode get_mode();      // Checks in what mode the robot currently is.
    ErrorCode change_mode();      // Goes from walking to rolling mode and vice-versa.

    // ROLLING MODE
    ErrorCode roll(int speed = 0); // Longitudinal movement of the robot.

    // WALKING MODE
    ErrorCode in_position(Droideka_Position pos, Action &pos_act, int time); // Checks if the wanted position is reachable given the mechanical constraints of the robot.
    void act(Action *action);                                                // Make the motors actually move.

    double last_action_millis = 0; // Time when the previous action was started
    int time_last_action;          // Time the previous action needs to be undertaken in
    int offset_time_last_action;   // Time between end of the previous action and the new one.

    ErrorCode execute_sequence(int f_or_b, int start_sequence, int length_sequence, int time, int offset_time); // Goes to the next step of the specified sequence.
    ErrorCode move_into_position(Droideka_Position pos, int time = 0);
    ErrorCode park(int time = 1000);                                              // Parking routine
    ErrorCode unpark(int time = 1000);                                            // Unparking routine
    ErrorCode walk(int throttle_x, int throttle_y, unsigned long time = 8000000); // Walking routine (time in seconds)
    ErrorCode establish_cog_movement();                                           // Determines the movement of the center of gravity and the end position of the legs
    Droideka_Position get_current_position();
    Droideka_Position get_future_position(Droideka_Position start_pos, float trans_x[TIME_SAMPLE], float trans_y[TIME_SAMPLE], float angle[TIME_SAMPLE], unsigned long time_elapsed, int one_leg = -1);
    Droideka_Position get_final_position(Droideka_Position start_pos);
    int walk_compute_state = 0;
    unsigned long start_walk_time = 0;
    int leg_order[LEG_NB];
    bool leg_lifted[LEG_NB];
    int moving_leg_nb = 0;
    unsigned long delta_time;
    Droideka_Position get_lifted_position(int leg, Droideka_Position start_pos, Droideka_Position end_pos, unsigned long time_);
    Droideka_Position current_position;
    Droideka_Position final_pos;
    ErrorCode establish_legs_movement();
    Droideka_Position movement[TIME_SAMPLE];

    // The following holds the minimum, middle and maximum values possible for the motors due to mechanical constraints.
    // The last parameter on each line represents the way of reading the encoder values (90degrees is maximum or minimum encoder counts value).
    const int extreme_values_motor[MOTOR_NB][4] = {
        {105, 480, 855, 1},
        {395, 500, 950, -1},
        {0, 500, 1000, -1},
        {125, 500, 875, -1},
        {50, 500, 605, 1},
        {0, 500, 1000, 1},
        {135, 510, 885, -1},
        {50, 500, 605, 1},
        {0, 500, 1000, 1},
        {135, 510, 885, 1},
        {395, 500, 950, -1},
        {0, 500, 1000, -1},
    };
};

#endif