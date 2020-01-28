#ifndef Droideka_h
#define Droideka_h

#include <Receiver.h>
#include <ServoBus.h>

#define DEBUG_BOARD_BAUD_RATE 115200

#define MOTOR_NB 12
#define LEG_NB 4
#define TIBIA_LENGTH 7
#define HIP_LENGTH 7

struct State
{
    unsigned long timestamp;
    int positions[MOTOR_NB];
    bool is_position_updated[MOTOR_NB];
    bool correct_motor_reading;
};

struct Action
{
    int commands[MOTOR_NB][3]; // [position, span, activate]
    void set_time(int time)
    {
        for (int ii = 0; ii < MOTOR_NB; ii++)
        {
            commands[ii][1] = time;
        }
    }
    void set_active(bool activate = true)
    {
        for (int ii = 0; ii < MOTOR_NB; ii++)
        {
            if (activate)
            {
                commands[ii][2] = 1;
            }
            else
            {
                commands[ii][2] = 0;
            }
        }
    }

    void shoulders_active(bool activate = true)
    {
        for (int ii = 0; ii < LEG_NB; ii++)
        {
            if (activate)
            {
                commands[3 * ii][2] = 1;
            }
            else
            {
                commands[3 * ii][2] = 0;
            }
        }
    }

    void hips_active(bool activate = true)
    {
        for (int ii = 0; ii < LEG_NB; ii++)
        {
            if (activate)
            {
                commands[3 * ii + 1][2] = 1;
            }
            else
            {
                commands[3 * ii + 1][2] = 0;
            }
        }
    }

    void knees_active(bool activate = true)
    {
        for (int ii = 0; ii < LEG_NB; ii++)
        {
            if (activate)
            {
                commands[3 * ii + 2][2] = 1;
            }
            else
            {
                commands[3 * ii + 2][2] = 0;
            }
        }
    }

    void motor_active(int id, bool activate)
    {
        if (activate)
        {
            commands[id][2] = 1;
        }
        else
        {
            commands[id][2] = 0;
        }
    }
};

struct Droideka_Position
{
    float legs[LEG_NB][3]; // For each leg, id 0 stores the shoulder angle in degrees, id 1 and id 2 store resp. the x and y coordinates with respect to the leg frame.
                           // LEG_NB : id 0 is the front left leg, id 1 is the front right leg, id 2 is the rear left leg, id 3 is the rear right leg.
    bool valid_position;

    Droideka_Position(float position[LEG_NB][3])
    {
        for (int ii = 0; ii < LEG_NB; ii++)
        {
            if (position[ii][1] * position[ii][1] + position[ii][2] * position[ii][2] > (HIP_LENGTH + TIBIA_LENGTH) * (HIP_LENGTH + TIBIA_LENGTH))
            {
                valid_position = false;
                break;
            }
            else
            {
                valid_position = true;
            }
        }

        for (int ii = 0; ii < LEG_NB; ii++)
        {
            for (int jj = 0; jj < 3; jj++)
            {
                legs[ii][jj] = position[ii][jj];
            }
        }
    }
};

enum DroidekaMode
{
    WALKING = 0,
    ROLLING = 1,
};
typedef enum DroidekaMode DroidekaMode;

enum ErrorCode
{
    WAITING = 0,

    NO_ERROR = 1,

    WRONG_MOTOR_SPECIFIED = 100,
    OUT_OF_BOUNDS_SPEED_SPECIFIED = 101,

    OUT_OF_BOUNDS_SHOULDER_ANGLE = 200,
    OUT_OF_BOUNDS_HIP_ANGLE = 201,
    OUT_OF_BOUNDS_KNEE_ANGLE = 202,

    PARKING_POSITION_NOT_UPDATED = 300,
    PARKING_TRANSITION_POSITION_IMPOSSIBLE = 301,
    PARKING_POSITION_IMPOSSIBLE = 302,
    STARTING_WALKING_POSITION_IMPOSSIBLE = 303,

    POSITION_UNREACHABLE = 400,

    ROBOT_ALREADY_PARKED = 500,
    ROBOT_ALREADY_UNPARKED = 501,
    ROBOT_PARKED_WHEN_ASKED_TO_WALK = 502,
};
typedef enum ErrorCode ErrorCode;

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
    ErrorCode move_forward(int throttle);
    ErrorCode move(char motor = 'l', int speed = 0);
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

    float ang_1 = 45.0;
    float ang_2 = -11.0;
    float ang_3 = 65.0;
    float ang_2_3 = (ang_2 + ang_3) / 2;
    float x_1 = 4.7;
    float x_2 = 4.3;
    float x_3 = 7.0;
    float y_touching = -12.0;
    float y_not_touching = -6.0;
    float starting_position[LEG_NB][3] = {{ang_1, x_1, y_touching}, {ang_2, x_2, y_touching}, {ang_1, x_1, y_touching}, {ang_2, x_2, y_touching}};
    float parking_transition[LEG_NB][3] = {{ang_1, x_1, y_not_touching}, {ang_2, x_2, y_not_touching}, {ang_1, x_1, y_not_touching}, {ang_2, x_2, y_not_touching}};
    Droideka_Position *starting_position_walking = new Droideka_Position(starting_position);
    Droideka_Position *parking_transition_position = new Droideka_Position(parking_transition);
    Droideka_Position *parking_position;

    ErrorCode walk(int time = 500, int offset_time = 500);
    double last_action_millis = 0;
    int time_last_action;
    int offset_time_last_action;
    static const int nb_walking_sequence = 10;
    int current_position = -1; // -1 is parked, 0 to 9 is the id of the position in the walking sequence. Thus 9 is the starting walking position.
    bool stoppable_walking_sequence[nb_walking_sequence] = {false, true, true, false, true, false, true, true, false, true};
    float walking_sequence[nb_walking_sequence][LEG_NB][3] = {
        {{ang_1, x_1, y_touching},
         {ang_2_3, x_2, y_not_touching},
         {ang_1, x_1, y_touching},
         {ang_2, x_2, y_touching}},
        {{ang_1, x_1, y_touching},
         {ang_3, x_3, y_touching},
         {ang_1, x_1, y_touching},
         {ang_2, x_2, y_touching}},

        {{ang_2, x_2, y_touching},
         {ang_1, x_1, y_touching},
         {ang_3, x_3, y_touching},
         {ang_1, x_1, y_touching}},

        {{ang_2, x_2, y_touching},
         {ang_1, x_1, y_touching},
         {ang_2_3, x_2, y_not_touching},
         {ang_1, x_1, y_touching}},
        {{ang_2, x_2, y_touching},
         {ang_1, x_1, y_touching},
         {ang_2, x_2, y_touching},
         {ang_1, x_1, y_touching}},

        {{ang_2_3, x_2, y_not_touching},
         {ang_1, x_1, y_touching},
         {ang_2, x_2, y_touching},
         {ang_1, x_1, y_touching}},
        {{ang_3, x_3, y_touching},
         {ang_1, x_1, y_touching},
         {ang_2, x_2, y_touching},
         {ang_1, x_1, y_touching}},

        {{ang_1, x_1, y_touching},
         {ang_2, x_2, y_touching},
         {ang_1, x_1, y_touching},
         {ang_3, x_3, y_touching}},

        {{ang_1, x_1, y_touching},
         {ang_2, x_2, y_touching},
         {ang_1, x_1, y_touching},
         {ang_2_3, x_2, y_not_touching}},
        {{ang_1, x_1, y_touching},
         {ang_2, x_2, y_touching},
         {ang_1, x_1, y_touching},
         {ang_2, x_2, y_touching}}};

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