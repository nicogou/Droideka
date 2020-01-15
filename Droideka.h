#ifndef Droideka_h
#define Droideka_h

#include <Receiver.h>
#include <ServoBus.h>

#define DEBUG_BOARD_BAUD_RATE 115200

#define MOTOR_NB 12
#define LEG_NB 4

struct State
{
    unsigned long timestamp;
    uint16_t positions[MOTOR_NB];
    bool is_position_updated[MOTOR_NB];
    bool correct_motor_reading;
};

struct Action
{
    uint16_t commands[MOTOR_NB][3]; // [position, span, activate]
};

// struct Leg_Position
// {
//     float shoulder_angle;
//     float x_position;
//     float y_position;
// };

struct Droideka_Position
{
    float legs[LEG_NB][3];
    // Leg_Position legs[LEG_NB];
};

enum ErrorCode
{
    NO_ERROR = 1,

    WRONG_MOTOR_SPECIFIED = 100,
    OUT_OF_BOUNDS_SPEED_SPECIFIED = 101,

    OUT_OF_BOUNDS_SHOULDER_ANGLE = 200,
    OUT_OF_BOUNDS_HIP_ANGLE = 201,
    OUT_OF_BOUNDS_KNEE_ANGLE = 202,
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
    ServoBus *servoBus_;
    int servo_bus_write_pin = 13;
    State *read_debug_board_positions();
    static void receive_debug_board_position(uint8_t id, uint8_t command, uint16_t param1, uint16_t param2);
    float servo_deg_ratio = 0.24; // Multiplier to go from degrees to servo encoder value.

    float hip_length = 7.0;   //L2
    float tibia_length = 7.0; //L1
    float max_shoulder_angle = 90;
    float min_shoulder_angle = -90;
    float max_hip_angle = 38;
    float min_hip_angle = -105;
    float max_knee_angle = 60;
    float min_knee_angle = -60;

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
    Droideka(Stream *debugBoardStream);
    void initialize(int l_m_p_1, int l_m_p_2, int l_m_p_pwm, int rec_rx, int rec_tx, int rec_state); // Class initializer.
    // Not all pins on the Mega and Mega 2560 support change interrupts, so only the following can be used for RX: 10, 11, 12, 13, 14, 15, 50, 51, 52, 53, A8 (62), A9 (63), A10 (64), A11 (65), A12 (66), A13 (67), A14 (68), A15 (69).
    bool receive_data();
    ErrorCode move(char motor = 'l', int speed = 0);

    ErrorCode in_position(Droideka_Position pos, Action *pos_act);
    void act(Action *action);
    float shoulder_angle_deg[LEG_NB];
    float knee_angle_deg[LEG_NB]; //phi 1
    float hip_angle_deg[LEG_NB];  //phi 2
    float shoulder_angle_rad[LEG_NB];
    float knee_angle_rad[LEG_NB]; //phi 1
    float hip_angle_rad[LEG_NB];  //phi 2
    float shoulder_angle_encoder[LEG_NB];
    float knee_angle_encoder[LEG_NB]; //phi 1
    float hip_angle_encoder[LEG_NB];  //phi 2

    int throttle_x;
    int throttle_y;
    int button1;
    int button2;
    int button3;
};

#endif