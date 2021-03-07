#include "constants.h"

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

    SERVOS_VOLTAGE_TOO_LOW = 10,

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
    INVALID_MOVEMENT = 401,

    ROBOT_ALREADY_PARKED = 500,
    ROBOT_ALREADY_UNPARKED = 501,
    ROBOT_PARKED_WHEN_ASKED_TO_MOVE = 502,
};
typedef enum ErrorCode ErrorCode;

struct State
{
    unsigned long timestamp;
    int32_t positions[MOTOR_NB];
    bool is_position_updated[MOTOR_NB];
    bool correct_motor_reading = false;
};

struct Action
{
    int32_t angle[MOTOR_NB]; // Angle that the motor has to go to
    uint16_t span[MOTOR_NB]; // Time that the motor has to complete the move
    bool activate[MOTOR_NB]; // Disables the motor if we don't need it to move.
    void set_time(int time)
    {
        for (int ii = 0; ii < MOTOR_NB; ii++)
        {
            span[ii] = time;
        }
    }
    void set_active(bool active = true)
    {
        for (int ii = 0; ii < MOTOR_NB; ii++)
        {
            if (active)
            {
                activate[ii] = 1;
            }
            else
            {
                activate[ii] = 0;
            }
        }
    }

    void shoulders_active(bool active = true)
    {
        for (int ii = 0; ii < LEG_NB; ii++)
        {
            if (active)
            {
                activate[3 * ii] = 1;
            }
            else
            {
                activate[3 * ii] = 0;
            }
        }
    }

    void hips_active(bool active = true)
    {
        for (int ii = 0; ii < LEG_NB; ii++)
        {
            if (active)
            {
                activate[3 * ii + 1] = 1;
            }
            else
            {
                activate[3 * ii + 1] = 0;
            }
        }
    }

    void knees_active(bool active = true)
    {
        for (int ii = 0; ii < LEG_NB; ii++)
        {
            if (active)
            {
                activate[3 * ii + 2] = 1;
            }
            else
            {
                activate[3 * ii + 2] = 0;
            }
        }
    }

    void motor_active(int id, bool active)
    {
        if (active)
        {
            activate[id] = 1;
        }
        else
        {
            activate[id] = 0;
        }
    }
};