#define DEBUG_BOARD_BAUD_RATE 115200

#define MOTOR_NB 12
#define LEG_NB 4
#define TIBIA_LENGTH 7
#define HIP_LENGTH 7
#define SERVO_BUS_WRITE_PIN_FRONT 13
#define SERVO_BUS_WRITE_PIN_BACK 13

#define ANG_MAINTENANCE 0.0
#define X_MAINTENANCE HIP_LENGTH + TIBIA_LENGTH
#define Y_MAINTENANCE 0.0
#define START_MAINTENANCE_SEQUENCE 0
#define LENGTH_MAINTENANCE_SEQUENCE 1
#define END_MAINTENANCE_SEQUENCE START_MAINTENANCE_SEQUENCE + LENGTH_MAINTENANCE_SEQUENCE - 1

#define START_UNPARKING_SEQUENCE START_MAINTENANCE_SEQUENCE + LENGTH_MAINTENANCE_SEQUENCE
#define LENGTH_UNPARKING_SEQUENCE 2
#define END_UNPARKING_SEQUENCE START_UNPARKING_SEQUENCE + LENGTH_UNPARKING_SEQUENCE - 1

#define ANG_PARKING 90.0
#define X_PARKING 9.8
#define Y_PARKING 9.0
#define START_PARKING_SEQUENCE START_UNPARKING_SEQUENCE + LENGTH_UNPARKING_SEQUENCE
#define LENGTH_PARKING_SEQUENCE 2
#define END_PARKING_SEQUENCE START_PARKING_SEQUENCE + LENGTH_PARKING_SEQUENCE - 1

#define ANG_1 40.9
#define ANG_2 0.0
#define ANG_3 60.0
#define ANG_2_3 (ANG_2 + ANG_3) / 2
#define X_1 5.29
#define X_2 4.0
#define X_3 8.0
#define Y_TOUCHING -11.0
#define Y_NOT_TOUCHING -6.0
#define START_WALKING_SEQUENCE START_PARKING_SEQUENCE + LENGTH_PARKING_SEQUENCE
#define LENGTH_WALKING_SEQUENCE 10
#define END_WALKING_SEQUENCE START_WALKING_SEQUENCE + LENGTH_WALKING_SEQUENCE - 1

// Constants for turning. The routine starts with a rotation on the left
#define ANG_TWIST_LEFT_FL 69.8
#define ANG_TWIST_LEFT_FR -12.8
#define ANG_TWIST_LEFT_RL 18.1
#define ANG_TWIST_LEFT_RR 40.9
#define X_TWIST_LEFT_FL 5.40
#define X_TWIST_LEFT_FR 6.46
#define X_TWIST_LEFT_RL 6.56
#define X_TWIST_LEFT_RR 2.46
#define START_TURN_LEFT_SEQUENCE START_WALKING_SEQUENCE + LENGTH_WALKING_SEQUENCE
#define LENGTH_TURN_LEFT_SEQUENCE 16
#define END_TURN_LEFT_SEQUENCE START_TURN_LEFT_SEQUENCE + LENGTH_TURN_LEFT_SEQUENCE - 1

/* Constants for turning. The routine starts with a rotation on the right
 * THESE ARE NOT USED !!! TO TURN RIGHT, WE DO THE PREVIOUS ONE IN REVERSE INSTEAD.
 * This is preferred because the X values are smaller in the left sequence, so less chance of breaking plastic parts (TBC). 
 */
#define ANG_TWIST_RIGHT_FL 18.1
#define ANG_TWIST_RIGHT_FR1 40.9
#define ANG_TWIST_RIGHT_RL 69.8
#define ANG_TWIST_RIGHT_RR -12.8
#define X_TWIST_RIGHT_FL 5.90
#define X_TWIST_RIGHT_FR 2.36
#define X_TWIST_RIGHT_RL 5.34
#define X_TWIST_RIGHT_RR 7.52
#define START_TURN_RIGHT_SEQUENCE START_TURN_LEFT_SEQUENCE + LENGTH_TURN_LEFT_SEQUENCE
#define LENGTH_TURN_RIGHT_SEQUENCE 16
#define END_TURN_RIGHT_SEQUENCE START_TURN_RIGHT_SEQUENCE + LENGTH_TURN_RIGHT_SEQUENCE - 1

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
    ROBOT_PARKED_WHEN_ASKED_TO_MOVE = 502,
};
typedef enum ErrorCode ErrorCode;