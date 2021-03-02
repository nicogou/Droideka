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
    int positions[MOTOR_NB];
    bool is_position_updated[MOTOR_NB];
    bool correct_motor_reading = false;
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