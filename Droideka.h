#ifndef Droideka_h
#define Droideka_h

#include <Receiver.h>
#include <ServoBus.h>
#include <utils/utils.h>

class Droideka
{
private:
    unsigned long baud_rate_bt = 38400;
    int state_pin_bt;

    unsigned long last_millis;
    unsigned long interval = 100;

    // Create a Bluetooth Receiver
    Receiver rec;

    //Create a ServoBus instance
    ServoBus *servoBus_front;
    ServoBus *servoBus_back;
    int servo_bus_write_pin = 13;
    static void receive_debug_board_position(uint8_t id, uint8_t command, uint16_t param1, uint16_t param2);
    float servo_deg_ratio = 0.24; // Multiplier to go from servo encoder to degrees value.

    ErrorCode encode_leg_angles(int leg_id);
    int deg_to_encoder(int motor_id, float deg_angle);
    float encoder_to_deg(int motor_id, int encoder_angle);

    float hip_length = HIP_LENGTH;     //L2
    float tibia_length = TIBIA_LENGTH; //L1

    // Motor ids for the Droideka legs
    unsigned int motor_ids[MOTOR_NB] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    // IDs 0, 1 and 2 represent the front left leg
    // IDs 3, 4 and 5 represent the front right leg
    // IDs 6, 7 and 8 represent the rear left leg
    // IDs 9, 10 and 11 represent the rear right leg

    // Longitudinal Motor
    int longitudinal_mot_pin_1;
    int longitudinal_mot_pin_2;
    int longitudinal_mot_pin_pwm;

public:
    Droideka(Stream *debugBoardStream_front, Stream *debugBoardStream_back);
    void initialize(int l_m_p_1, int l_m_p_2, int l_m_p_pwm, int rec_rx, int rec_tx, int rec_state); // Class initializer.
    // Not all pins on the Mega and Mega 2560 support change interrupts, so only the following can be used for RX: 10, 11, 12, 13, 14, 15, 50, 51, 52, 53, A8 (62), A9 (63), A10 (64), A11 (65), A12 (66), A13 (67), A14 (68), A15 (69).
    bool receive_data();
    ErrorCode move(int throttle);
    ErrorCode roll(int speed = 0);
    DroidekaMode get_mode();
    ErrorCode change_mode();

    ErrorCode in_position(Droideka_Position pos, Action &pos_act, int time);
    void act(Action *action);
    float shoulder_angle_deg[LEG_NB] = {0, 0, 0, 0};
    float knee_angle_deg[LEG_NB] = {0, 0, 0, 0}; //phi 1
    float hip_angle_deg[LEG_NB] = {0, 0, 0, 0};  //phi 2
    float shoulder_angle_rad[LEG_NB] = {0, 0, 0, 0};
    float knee_angle_rad[LEG_NB] = {0, 0, 0, 0}; //phi 1
    float hip_angle_rad[LEG_NB] = {0, 0, 0, 0};  //phi 2
    int shoulder_angle_encoder[LEG_NB] = {0, 0, 0, 0};
    int knee_angle_encoder[LEG_NB] = {0, 0, 0, 0}; //phi 1
    int hip_angle_encoder[LEG_NB] = {0, 0, 0, 0};  //phi 2

    void set_parking_position(Droideka_Position *park);
    void set_parking_position(float park[LEG_NB][3]);
    bool parking_updated = false;
    ErrorCode park(int time = 500, int offset_time = 500);
    ErrorCode unpark(int time = 500, int offset_time = 500);

    int throttle_x;
    int throttle_y;
    int button1;
    int button2;
    int button3;
    bool forward();
    bool backward();
    bool leftward();
    bool rightward();
    bool button1_pushed();
    bool button1_clicked();
    bool button1_released();
    bool button2_pushed();
    bool button2_clicked();
    bool button2_released();
    bool button3_pushed();
    bool button3_clicked();
    bool button3_released();

    State *read_debug_board_positions();

    float starting_position[LEG_NB][3] = {{ANG_1, X_1, Y_TOUCHING}, {ANG_2, X_2, Y_TOUCHING}, {ANG_1, X_1, Y_TOUCHING}, {ANG_2, X_2, Y_TOUCHING}};
    float parking_transition[LEG_NB][3] = {{ANG_1, X_1, Y_NOT_TOUCHING}, {ANG_2, X_2, Y_NOT_TOUCHING}, {ANG_1, X_1, Y_NOT_TOUCHING}, {ANG_2, X_2, Y_NOT_TOUCHING}};
    Droideka_Position *starting_position_walking = new Droideka_Position(starting_position);
    Droideka_Position *parking_transition_position = new Droideka_Position(parking_transition);
    Droideka_Position *parking_position;

    ErrorCode walk(int time = 500, int offset_time = 500);
    double last_action_millis = 0;
    int time_last_action;
    int offset_time_last_action;
    static const int nb_walking_sequence = 10;
    int current_position = -1; // -1 is parked, 0 to 9 is the id of the position in the walking sequence. Thus 9 is the starting walking position.
    float walking_sequence[nb_walking_sequence][LEG_NB][3] = {
        {{ANG_1, X_1, Y_TOUCHING},
         {ANG_2_3, X_2, Y_NOT_TOUCHING},
         {ANG_1, X_1, Y_TOUCHING},
         {ANG_2, X_2, Y_TOUCHING}},
        {{ANG_1, X_1, Y_TOUCHING},
         {ANG_3, X_3, Y_TOUCHING},
         {ANG_1, X_1, Y_TOUCHING},
         {ANG_2, X_2, Y_TOUCHING}},

        {{ANG_2, X_2, Y_TOUCHING},
         {ANG_1, X_1, Y_TOUCHING},
         {ANG_3, X_3, Y_TOUCHING},
         {ANG_1, X_1, Y_TOUCHING}},

        {{ANG_2, X_2, Y_TOUCHING},
         {ANG_1, X_1, Y_TOUCHING},
         {ANG_2_3, X_2, Y_NOT_TOUCHING},
         {ANG_1, X_1, Y_TOUCHING}},
        {{ANG_2, X_2, Y_TOUCHING},
         {ANG_1, X_1, Y_TOUCHING},
         {ANG_2, X_2, Y_TOUCHING},
         {ANG_1, X_1, Y_TOUCHING}},

        {{ANG_2_3, X_2, Y_NOT_TOUCHING},
         {ANG_1, X_1, Y_TOUCHING},
         {ANG_2, X_2, Y_TOUCHING},
         {ANG_1, X_1, Y_TOUCHING}},
        {{ANG_3, X_3, Y_TOUCHING},
         {ANG_1, X_1, Y_TOUCHING},
         {ANG_2, X_2, Y_TOUCHING},
         {ANG_1, X_1, Y_TOUCHING}},

        {{ANG_1, X_1, Y_TOUCHING},
         {ANG_2, X_2, Y_TOUCHING},
         {ANG_1, X_1, Y_TOUCHING},
         {ANG_3, X_3, Y_TOUCHING}},

        {{ANG_1, X_1, Y_TOUCHING},
         {ANG_2, X_2, Y_TOUCHING},
         {ANG_1, X_1, Y_TOUCHING},
         {ANG_2_3, X_2, Y_NOT_TOUCHING}},
        {{ANG_1, X_1, Y_TOUCHING},
         {ANG_2, X_2, Y_TOUCHING},
         {ANG_1, X_1, Y_TOUCHING},
         {ANG_2, X_2, Y_TOUCHING}}};

    ErrorCode slide(int time = 500, int offset_time = 500);
    static const int nb_sliding_sequence = 9;
    float sliding_sequence[nb_walking_sequence][LEG_NB][3] = {
        {{ANG_4, X_4, Y_TOUCHING},
         {ANG_5, X_5, Y_TOUCHING},
         {ANG_4, X_4, Y_TOUCHING},
         {ANG_5, X_5, Y_TOUCHING}},

        {{ANG_4, X_4, Y_NOT_TOUCHING},
         {ANG_5, X_5, Y_TOUCHING},
         {ANG_4, X_4, Y_TOUCHING},
         {ANG_5, X_5, Y_TOUCHING}},
        {{ANG_1, X_1, Y_TOUCHING},
         {ANG_5, X_5, Y_TOUCHING},
         {ANG_4, X_4, Y_TOUCHING},
         {ANG_5, X_5, Y_TOUCHING}},

        {{ANG_1, X_1, Y_TOUCHING},
         {ANG_5, X_5, Y_NOT_TOUCHING},
         {ANG_4, X_4, Y_TOUCHING},
         {ANG_5, X_5, Y_TOUCHING}},
        {{ANG_1, X_1, Y_TOUCHING},
         {ANG_2, X_2, Y_TOUCHING},
         {ANG_4, X_4, Y_TOUCHING},
         {ANG_5, X_5, Y_TOUCHING}},

        {{ANG_1, X_1, Y_TOUCHING},
         {ANG_2, X_2, Y_TOUCHING},
         {ANG_4, X_4, Y_NOT_TOUCHING},
         {ANG_5, X_5, Y_TOUCHING}},
        {{ANG_1, X_1, Y_TOUCHING},
         {ANG_2, X_2, Y_TOUCHING},
         {ANG_1, X_1, Y_TOUCHING},
         {ANG_5, X_5, Y_TOUCHING}},

        {{ANG_1, X_1, Y_TOUCHING},
         {ANG_2, X_2, Y_TOUCHING},
         {ANG_1, X_1, Y_TOUCHING},
         {ANG_5, X_5, Y_NOT_TOUCHING}},
        {{ANG_1, X_1, Y_TOUCHING},
         {ANG_2, X_2, Y_TOUCHING},
         {ANG_1, X_1, Y_TOUCHING},
         {ANG_2, X_2, Y_TOUCHING}}};

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