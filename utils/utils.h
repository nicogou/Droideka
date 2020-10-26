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
#define Y_ZERO -7.0     // A vérifier
#define THETA_IDLE 45.0 // A vérifier
#define X_IDLE 4.0      // A vérifier

#define THETA_PARKING 90.0
#define X_PARKING 9.8
#define Y_PARKING 9.0

#define SERVO_BUS_WRITE_PIN_FRONT 13
#define SERVO_BUS_WRITE_PIN_BACK 13

float parked[LEG_NB][3] = {
    {THETA_PARKING, X_PARKING, Y_PARKING},
    {THETA_PARKING, X_PARKING, Y_PARKING},
    {THETA_PARKING, X_PARKING, Y_PARKING},
    {THETA_PARKING, X_PARKING, Y_PARKING}};
float unparking[LEG_NB][3] = {
    {THETA_IDLE, X_IDLE, Y_NOT_TOUCHING},
    {THETA_IDLE, X_IDLE, Y_NOT_TOUCHING},
    {THETA_IDLE, X_IDLE, Y_NOT_TOUCHING},
    {THETA_IDLE, X_IDLE, Y_NOT_TOUCHING}};
float unparked[LEG_NB][3] = {
    {THETA_IDLE, X_IDLE, Y_TOUCHING},
    {THETA_IDLE, X_IDLE, Y_TOUCHING},
    {THETA_IDLE, X_IDLE, Y_TOUCHING},
    {THETA_IDLE, X_IDLE, Y_TOUCHING}};

float shoulder_pos[LEG_NB][2] = {
    {-BODY_WIDTH / 2, BODY_LENGTH / 2},
    {BODY_WIDTH / 2, BODY_LENGTH / 2},
    {-BODY_WIDTH / 2, -BODY_LENGTH / 2},
    {BODY_WIDTH / 2, -BODY_LENGTH / 2}};
float shoulder_mult[LEG_NB][2] = {
    {-1, 1},
    {1, 1},
    {-1, -1},
    {1, -1}};

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

struct Movement
{
    Droideka_Position start_position;
    Droideka_Position end_position;
    Droideka_Position positions[TIME_SAMPLE];
    bool valid_movement = true;
    bool stable_movement; // A implémenter.

    unsigned long start_walk_time;

    int leg_order[LEG_NB];
    bool leg_lifted[LEG_NB];
    int moving_leg_nb = 0;
    unsigned long delta_time;

    float tx[TIME_SAMPLE];
    float ty[TIME_SAMPLE];
    float alpha[TIME_SAMPLE];
    float reverse_tx[TIME_SAMPLE];
    float reverse_ty[TIME_SAMPLE];
    float reverse_alpha[TIME_SAMPLE];

    Movement::Movement() {}

    Movement::Movement(Droideka_Position start_position_, int throttle_x, int throttle_y)
    {
        start_position = start_position_;
        establish_cog_movement(throttle_x, throttle_y);
        end_position = get_final_position(start_position);
        establish_legs_movement();
        for (int ii = 0; ii < TIME_SAMPLE; ii++)
        {
            valid_movement *= positions[ii].valid_position;
        }
        start_walk_time = micros();
    }

    ErrorCode establish_cog_movement(int throttle_x, int throttle_y)
    {
        if (throttle_x > 0 && abs(throttle_x) > abs(throttle_y))
        // Moving forward.
        {
            for (int ii = 0; ii < TIME_SAMPLE; ii++)
            {
                tx[ii] = MAX_LONGITUDINAL_COG_MOVE * ii / TIME_SAMPLE;
                ty[ii] = 0;
                alpha[ii] = 0;
            }
            leg_order[3] = 1;
            leg_order[1] = 2;
            leg_order[2] = 3;
            leg_order[0] = 4;

            for (int ii = 0; ii < LEG_NB; ii++)
            {
                leg_lifted[ii] = false;
            }
            moving_leg_nb = 4;
            delta_time = TIME_SAMPLE / (moving_leg_nb * 4);
        }

        for (int ii = 0; ii < TIME_SAMPLE; ii++)
        {
            reverse_tx[ii] = tx[TIME_SAMPLE - ii] * -1;
            reverse_ty[ii] = ty[TIME_SAMPLE - ii] * -1;
            reverse_alpha[ii] = alpha[TIME_SAMPLE - ii] * -1;
        }

        return NO_ERROR;
    }

    ErrorCode establish_cog_movement_advanced(int throttle_x, int throttle_y)
    {
        if (throttle_x > 0 && abs(throttle_x) > abs(throttle_y))
        // Moving forward.
        {
            float move_x = throttle_x * MAX_LONGITUDINAL_COG_MOVE / 100; // throttle_x is between 0 and 100.
            float move_y = throttle_y * MAX_LATERAL_COG_MOVE / 100;      // throttle_y is between 0 and 100.
            float move_angle = 0;                                        // To be determined.

            for (int ii = 0; ii < TIME_SAMPLE; ii++)
            {
                ty[ii] = move_x * ii / TIME_SAMPLE;
                tx[ii] = ty[ii] * move_x / move_y + ty[ii] / move_y * (1 - ty[ii] / move_y) * (ty[ii] * (2 * move_x / move_y - tan(PI / 2 + move_angle)) - move_x);
                alpha[ii] = move_angle * ii / TIME_SAMPLE;
            }
            leg_order[3] = 1;
            leg_order[1] = 2;
            leg_order[2] = 3;
            leg_order[0] = 4;

            for (int ii = 0; ii < LEG_NB; ii++)
            {
                leg_lifted[ii] = false;
            }
            moving_leg_nb = 4;
            delta_time = TIME_SAMPLE / (moving_leg_nb * 4);
        }

        for (int ii = 0; ii < TIME_SAMPLE; ii++)
        {
            reverse_tx[ii] = tx[TIME_SAMPLE - ii] * -1;
            reverse_ty[ii] = ty[TIME_SAMPLE - ii] * -1;
            reverse_alpha[ii] = alpha[TIME_SAMPLE - ii] * -1;
        }

        return NO_ERROR;
    }

    bool establish_stableness()
    {
        float start_global_position[LEG_NB][2];
        float final_global_position[LEG_NB][2];
        float first_pair[2][2];
        float second_pair[2][2];
        float cog_pair[2][2];
        float abc[3][3]; // 0 = first pair, 1 = second pair, 3 = CoG deltas. 0 = a, 1 = b, 2 = c.
        float intersection[2][2];
        float mid_cog[2];
        float distances[2];

        for (int ii = 0; ii < LEG_NB; ii++)
        {
            start_global_position[ii][0] = shoulder_pos[ii][0] + shoulder_mult[ii][0] * start_position.legs[ii][1] * cos(start_position.legs[ii][0]);
            start_global_position[ii][1] = shoulder_pos[ii][1] + shoulder_mult[ii][1] * start_position.legs[ii][1] * sin(start_position.legs[ii][0]);
            final_global_position[ii][0] = tx[TIME_SAMPLE] + start_global_position[ii][0] * cos(alpha[TIME_SAMPLE]) - start_global_position[ii][1] * sin(alpha[TIME_SAMPLE]);
            final_global_position[ii][1] = ty[TIME_SAMPLE] + start_global_position[ii][0] * sin(alpha[TIME_SAMPLE]) + start_global_position[ii][1] * cos(alpha[TIME_SAMPLE]);

            if (leg_order[ii] == 1)
            {
                first_pair[0][0] = final_global_position[ii][0];
                first_pair[0][1] = final_global_position[ii][1];
                first_pair[1][0] = start_global_position[3 - ii][0];
                first_pair[1][1] = start_global_position[3 - ii][1];
                abc[0][0] = -(first_pair[1][1] - first_pair[0][1]) / (first_pair[1][0] - first_pair[0][0]);
                abc[0][1] = 1;
                abc[0][2] = -first_pair[1][1] + (first_pair[1][1] - first_pair[0][1]) / (first_pair[1][0] - first_pair[0][0]) * first_pair[1][0];
            }
            if (leg_order[ii] = 3)
            {
                second_pair[0][0] = final_global_position[ii][0];
                second_pair[0][1] = final_global_position[ii][1];
                second_pair[1][0] = start_global_position[3 - ii][0];
                second_pair[1][1] = start_global_position[3 - ii][1];
                abc[1][0] = -(second_pair[1][1] - second_pair[0][1]) / (second_pair[1][0] - second_pair[0][0]);
                abc[1][1] = 1;
                abc[1][2] = -second_pair[1][1] + (second_pair[1][1] - second_pair[0][1]) / (second_pair[1][0] - second_pair[0][0]) * second_pair[1][0];
            }
        }

        cog_pair[0][0] = 0;
        cog_pair[0][1] = 0;
        cog_pair[1][0] = tx[TIME_SAMPLE];
        cog_pair[1][1] = ty[TIME_SAMPLE];
        abc[2][0] = -(cog_pair[1][1] - cog_pair[0][1]) / (cog_pair[1][0] - cog_pair[0][0]);
        abc[2][1] = 1;
        abc[2][2] = -cog_pair[1][1] + (cog_pair[1][1] - cog_pair[0][1]) / (cog_pair[1][0] - cog_pair[0][0]) * cog_pair[1][0];
        mid_cog[0] = (cog_pair[0][0] + cog_pair[1][0]) / 2;
        mid_cog[1] = (cog_pair[0][1] + cog_pair[1][1]) / 2;

        intersection[0][0] = (abc[0][1] * abc[2][2] - abc[2][1] * abc[0][2]) / (abc[0][0] * abc[2][1] - abc[2][0] * abc[0][2]);
        intersection[0][1] = (abc[2][0] * abc[0][2] - abc[0][0] * abc[2][2]) / (abc[0][0] * abc[2][1] - abc[2][0] * abc[0][2]);
        intersection[1][0] = (abc[1][1] * abc[2][2] - abc[2][1] * abc[1][2]) / (abc[1][0] * abc[2][1] - abc[2][0] * abc[1][2]);
        intersection[1][1] = (abc[2][0] * abc[1][2] - abc[1][0] * abc[2][2]) / (abc[1][0] * abc[2][1] - abc[2][0] * abc[1][2]);

        distances[0] = sqrt((mid_cog[0] - intersection[0][0]) * (mid_cog[0] - intersection[0][0]) + (mid_cog[1] - intersection[0][1]) * (mid_cog[1] - intersection[0][1]));
        distances[1] = sqrt((mid_cog[0] - intersection[1][0]) * (mid_cog[0] - intersection[1][0]) + (mid_cog[1] - intersection[1][1]) * (mid_cog[1] - intersection[1][1]));
    }

    Droideka_Position get_future_position(Droideka_Position start_pos, float trans_x[TIME_SAMPLE], float trans_y[TIME_SAMPLE], float angle[TIME_SAMPLE], unsigned long time_elapsed, int one_leg = -1)
    {
        if (time_elapsed < 0 || time_elapsed > TIME_SAMPLE)
        {
            return start_pos; // Not sure this is right if time_elapsed > TIME_SAMPLE.
        }

        float temp[LEG_NB][2];           // x and y final coordinates of each feet
        float temp_final_pos[LEG_NB][3]; // used to build the Droideka_Position object by calculating rho, theta and z thanks to x and y stored in temp.

        for (int ii = 0; ii < LEG_NB; ii++)
        {
            if (one_leg != -1)
            {
                ii = one_leg;
            }

            temp[ii][0] = shoulder_pos[ii][0] * (cos(angle[time_elapsed]) - 1) + shoulder_pos[ii][1] * sin(angle[time_elapsed]) + shoulder_mult[ii][0] * sqrt((start_pos.legs[ii][1] * cos(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][0] * trans_x[time_elapsed]) * (start_pos.legs[ii][1] * cos(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][0] * trans_x[time_elapsed]) + (start_pos.legs[ii][1] * sin(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][1] * trans_y[time_elapsed]) * (start_pos.legs[ii][1] * sin(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][1] * trans_y[time_elapsed])) * cos(shoulder_mult[ii][0] * shoulder_mult[ii][1] * atan((start_pos.legs[ii][1] * cos(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][0] * trans_x[time_elapsed]) / (start_pos.legs[ii][1] * sin(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][1] * trans_y[time_elapsed])) - angle[time_elapsed]);
            temp[ii][1] = shoulder_pos[ii][1] * (cos(angle[time_elapsed]) - 1) + shoulder_pos[ii][0] * sin(angle[time_elapsed]) + shoulder_mult[ii][0] * sqrt((start_pos.legs[ii][1] * cos(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][0] * trans_x[time_elapsed]) * (start_pos.legs[ii][1] * cos(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][0] * trans_x[time_elapsed]) + (start_pos.legs[ii][1] * sin(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][1] * trans_y[time_elapsed]) * (start_pos.legs[ii][1] * sin(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][1] * trans_y[time_elapsed])) * sin(shoulder_mult[ii][0] * shoulder_mult[ii][1] * atan((start_pos.legs[ii][1] * cos(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][0] * trans_x[time_elapsed]) / (start_pos.legs[ii][1] * sin(3.141592 * start_pos.legs[ii][1] / 180) - shoulder_mult[ii][1] * trans_y[time_elapsed])) - angle[time_elapsed]);
            temp_final_pos[ii][2] = start_pos.legs[ii][2];
            temp_final_pos[ii][1] = sqrt(temp[ii][0] * temp[ii][0] + temp[ii][1] * temp[ii][1]);
            if (temp_final_pos[ii][1] == 0) // Si x et y sont nuls
            {
                temp_final_pos[ii][0] = 0; // TODO : réfléchir à cette valeur. Est-il possible de déterminer theta si x et y sont nuls?
            }
            else if (temp[ii][0] == 0) // Si rho est non nul, alors si x est nul, y est non nul et on peut diviser par y.
            {
                temp_final_pos[ii][0] = temp[ii][1] / abs(temp[ii][1]) * 90; // Si x est nul, rho vaut + ou - 90°, determiné par le signe de y.
            }
            else
            {
                temp_final_pos[ii][0] = atan(temp[ii][1] / temp[ii][0]); // Dans le cas général, tan(theta) = y/x.
            }

            if (one_leg != -1)
            {
                ii = LEG_NB;
            }
        }
        Droideka_Position final_pos(temp_final_pos);
        return final_pos;
    }

    Droideka_Position get_final_position(Droideka_Position start_pos)
    {
        return get_future_position(start_pos, tx, ty, alpha, TIME_SAMPLE);
    }

    Droideka_Position get_lifted_position(int leg, Droideka_Position start_pos, Droideka_Position end_pos, unsigned long time_)
    {
        unsigned long debut_time = (leg_order[leg] - 1) * TIME_SAMPLE / moving_leg_nb + delta_time;
        unsigned long fin_time = leg_order[leg] * TIME_SAMPLE / moving_leg_nb;
        unsigned long mid_time = (debut_time + fin_time) / 2; // Not used.
        unsigned long interval_time = fin_time - debut_time;
        unsigned long time_from_lifting = time_ - debut_time;

        Droideka_Position debut_pos(get_future_position(start_pos, tx, ty, alpha, debut_time, leg).legs);
        Droideka_Position fin_pos(get_future_position(end_pos, reverse_tx, reverse_ty, reverse_alpha, TIME_SAMPLE - fin_time, leg).legs);

        // Between the lifting and putting back of the leg, theta and X are linear, wheras Y follows a quadratic curve (arbitrarily defined)

        float temp[LEG_NB][3];
        for (int ii = 0; ii < 2; ii++)
        {
            temp[leg][ii] = (fin_pos.legs[leg][ii] - debut_pos.legs[leg][ii]) / interval_time * time_from_lifting + debut_pos.legs[leg][ii];
        }
        temp[leg][2] = Y_NOT_TOUCHING - (Y_NOT_TOUCHING - Y_TOUCHING) * ((time_from_lifting - interval_time / 2) / (interval_time / 2)) * ((time_from_lifting - interval_time / 2) / (interval_time / 2));

        Droideka_Position result(temp);
        return result;
    }

    ErrorCode establish_legs_movement()
    {
        float temp[LEG_NB][3];
        unsigned long time_leg_starts_lifting;
        unsigned long time_leg_touches_ground_again;

        for (int ii = 0; ii < TIME_SAMPLE; ii++)
        {
            Droideka_Position temp_current_pos = get_future_position(start_position, tx, ty, alpha, ii);                                    // Calculates the position of the legs before the leg is lifted.
            Droideka_Position temp_future_pos = get_future_position(end_position, reverse_tx, reverse_ty, reverse_alpha, TIME_SAMPLE - ii); // Calculates the position of the legs after the leg has been lifted and put back on the ground.

            for (int jj = 0; jj < LEG_NB; jj++)
            {
                time_leg_starts_lifting = (leg_order[jj] - 1) * TIME_SAMPLE / moving_leg_nb + delta_time;
                time_leg_touches_ground_again = (leg_order[jj]) * TIME_SAMPLE / moving_leg_nb;

                if (ii <= time_leg_starts_lifting)
                {
                    for (int kk = 0; kk < 3; kk++)
                    {
                        temp[jj][kk] = temp_current_pos.legs[jj][kk];
                    }
                }
                else if (ii > time_leg_starts_lifting && ii <= time_leg_touches_ground_again)
                {
                    for (int kk = 0; kk < 3; kk++)
                    {
                        temp[jj][kk] = get_lifted_position(jj, start_position, end_position, ii).legs[jj][kk];
                    }
                }
                else if (ii > time_leg_touches_ground_again && ii <= TIME_SAMPLE)
                {
                    for (int kk = 0; kk < 3; kk++)
                    {
                        temp[jj][kk] = temp_future_pos.legs[jj][kk];
                    }
                }
            }
            positions[ii] = Droideka_Position(temp);
        }
        return NO_ERROR;
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
    INVALID_MOVEMENT = 401,

    ROBOT_ALREADY_PARKED = 500,
    ROBOT_ALREADY_UNPARKED = 501,
    ROBOT_PARKED_WHEN_ASKED_TO_MOVE = 502,
};
typedef enum ErrorCode ErrorCode;