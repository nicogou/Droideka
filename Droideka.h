#ifndef Droideka_h
#define Droideka_h

#include <EasyTransfer.h>

struct SEND_DATA_STRUCTURE
{
    // put your variable definitions here for the data you want to receive
    // THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO

    char data_to_display[17];
};

struct RECEIVE_DATA_STRUCTURE
{
    // put your variable definitions here for the data you want to receive
    // THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO

    int16_t joy_x;
    int16_t joy_y;
    int16_t mode;
    bool trig_left;
    bool trig_right;
};

enum ErrorCode
{
    NO_ERROR = 1,
    WRONG_MOTOR_SPECIFIED = 100,
    WRONG_MOTOR_DIRECTION_SPECIFIED = 101,
    NEGATIVE_SPEED_SPECIFIED = 102,
};
typedef enum ErrorCode ErrorCode;

class Droideka
{
private:
    unsigned long baud_rate_bt = 38400;

    unsigned long last_millis;
    unsigned long interval = 100;

    // create two EasyTransfer objects.
    EasyTransfer ET_in, ET_out;
    // give a name to the group of data
    RECEIVE_DATA_STRUCTURE rx_data;
    SEND_DATA_STRUCTURE tx_data;

    // Longitudinal Motor
    int longitudinal_mot_pin_1;
    int longitudinal_mot_pin_2;
    int longitudinal_mot_pin_pwm;

public:
    Droideka();
    void initialize(int l_m_p_1 = 0, int l_m_p_2 = 0, int l_m_p_pwm = 0); // Class initializer.
    bool receive_data();
    ErrorCode move(char motor = 'l', int speed = 0, char f_or_b = 'f');

    int16_t joy_x;
    int16_t joy_y;
    int16_t mode;
    bool trig_left;
    bool trig_right;

    int16_t previous_mode = -1;
};

#endif