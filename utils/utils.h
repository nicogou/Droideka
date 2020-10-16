#define DEBUG_BOARD_BAUD_RATE 115200

#define MOTOR_NB 12
#define LEG_NB 4
#define TIBIA_LENGTH 7
#define HIP_LENGTH 7
#define BODY_LENGTH 200 // TODO: verifier la valeur
#define BODY_WIDTH 150  // TODO: verifier la valeur
#define TIME_SAMPLE 1200
#define MAX_LONGITUDINAL_COG_MOVE 1
#define MAX_LATERAL_COG_MOVE 1
#define MAX_ANGLE_COG_MOVE 1

#define Y_TOUCHING -11.0
#define Y_NOT_TOUCHING -6.0

#define SERVO_BUS_WRITE_PIN_FRONT 13
#define SERVO_BUS_WRITE_PIN_BACK 13

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

    Droideka_Position::Droideka_Position() {}

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
    ROBOT_PARKED_WHEN_ASKED_TO_MOVE = 502,
};
typedef enum ErrorCode ErrorCode;