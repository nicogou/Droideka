#include "constants.h"

// Defines the mode of the robot. Is it rolling, walking, in maintenance, or in an unknown position.
enum DroidekaMode
{
    UNDEFINED = 0,
    WALKING = 1,
    ROLLING = 2,
    MAINTENANCE = 3,
};
typedef enum DroidekaMode DroidekaMode;

// Specifies the delayed function to be operated. This will be more populated as more functionalities are added.
enum DelayedFunction
{
    NOTHING = 0,
    DISABLE_SERVOS = 1,
    DISABLE_LEG_SERVOS = 2,
    DISABLE_LONG_SERVOS = 3,
};
typedef enum DelayedFunction DelayedFunction;

// Error codes when an error occurs.
enum ErrorCode
{
    WAITING = 0,

    NO_ERROR = 1,

    SERVOS_VOLTAGE_TOO_LOW = 10,
    VOLTAGE_CHECK_NOT_PERFORMED = 11,
    MOVING_THUS_UNABLE_TO_SET_MOVEMENT = 20,
    MOVING_THUS_UNABLE_TO_ADD_POSITION = 21,
    MOVEMENT_NOT_FINISHED = 22,
    CURRENT_POS_AND_STARTING_POS_NOT_MATCHING = 23,

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

    MPU_6050_CONNECTION_FAILED = 600,
    MPU_6050_DMP_INIT_FAILED = 601,
    DMP_NOT_READY = 602,
};
typedef enum ErrorCode ErrorCode;

// Defines the state of the servos.
struct State
{
    unsigned long timestamp;
    int32_t positions[MOTOR_NB];
    bool is_position_updated[MOTOR_NB];
    bool correct_motor_reading = false;
};

// Defines the commands to be sent to the servos.
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