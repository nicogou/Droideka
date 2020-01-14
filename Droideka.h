#ifndef Droideka_h
#define Droideka_h

#include <Receiver.h>
#include <ServoBus.h>

#define MOTOR_NB 12

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

enum ErrorCode
{
    NO_ERROR = 1,
    WRONG_MOTOR_SPECIFIED = 100,
    OUT_OF_BOUNDS_SPEED_SPECIFIED = 101,
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
    void act(Action *action);

    // Motor ids for the Droideka
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

    int throttle_x;
    int throttle_y;
    int button1;
    int button2;
    int button3;
};

#endif