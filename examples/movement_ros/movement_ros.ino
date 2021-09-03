#include <Droideka.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>
#define LMP1 3
#define LMP_PWM 4
#define IMU_INT_PIN 24

Droideka *droid_1;

int16_t thresholds[NB_MAX_DATA];
int time_ms = 1000;
int time_step = 4000;
float x = 0.0, y = 0.0, z = 0.0, alpha = 0.0;
bool start_move = false, continue_move = false, finish_move = false;
// float trans_x[TIME_SAMPLE];
// float trans_y[TIME_SAMPLE];
// float trans_z[TIME_SAMPLE];
// float rot[TIME_SAMPLE];

ros::NodeHandle nh;

int32_t joy_buttons[17] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int32_t last_joy_buttons[17] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void joyCallback(const sensor_msgs::Joy &msg)
{
    nh.loginfo("new joy");
    for (int ii = 0; ii < 17; ii++)
    {
        last_joy_buttons[ii] = joy_buttons[ii];
        joy_buttons[ii] = msg.buttons[ii];
    }

    if ((joy_buttons[0] == 1 && last_joy_buttons[0] == 0) || (joy_buttons[1] == 1 && last_joy_buttons[1] == 0) || (joy_buttons[2] == 1 && last_joy_buttons[2] == 0) || (joy_buttons[3] == 1 && last_joy_buttons[3] == 0))
    {
        start_move = true;
        continue_move = false;
        finish_move = false;
    }
    if (joy_buttons[2] == 1 && last_joy_buttons[2] == 0)
    {
        x = 2.0;
        y = 0.0;
        z = 0.0;
        alpha = 0.0;
    }
    if (joy_buttons[0] == 1 && last_joy_buttons[0] == 0)
    {
        x = -2.0;
        y = 0.0;
        z = 0.0;
        alpha = 0.0;
    }
    if (joy_buttons[3] == 1 && last_joy_buttons[3] == 0)
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        alpha = 1.0;
    }
    if (joy_buttons[1] == 1 && last_joy_buttons[1] == 0)
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        alpha = -1.0;
    }
    if (start_move)
    {
        droid_1->set_movement(Droideka_Movement(Droideka_Position(droid_1->unparked), x, y, z, alpha, time_step));
        start_move = false;
    }
    if (joy_buttons[0] == 1 || joy_buttons[1] == 1 || joy_buttons[2] == 1 || joy_buttons[3] == 1)
    {
        start_move = false;
        continue_move = true;
        finish_move = false;
    }
    if (joy_buttons[2] == 1)
    {
        x = 2.0;
        y = 0.0;
        z = 0.0;
        alpha = 0.0;
    }
    if (joy_buttons[0] == 1)
    {
        x = -2.0;
        y = 0.0;
        z = 0.0;
        alpha = 0.0;
    }
    if (joy_buttons[3] == 1)
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        alpha = 1.0;
    }
    if (joy_buttons[1] == 1)
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        alpha = -1.0;
    }
    if (continue_move)
    {
        droid_1->next_movement_sequence(INTERMEDIATE_SEQUENCE, x, y, alpha);
        continue_move = false;
    }
    if ((joy_buttons[0] == 0 && last_joy_buttons[0] == 1) || (joy_buttons[1] == 0 && last_joy_buttons[1] == 1) || (joy_buttons[2] == 0 && last_joy_buttons[2] == 1) || (joy_buttons[3] == 0 && last_joy_buttons[3] == 1))
    {
        droid_1->next_movement_sequence(FINISHING_SEQUENCE);
    }

    if (joy_buttons[5] == 1 && last_joy_buttons[5] == 0)
    {
        droid_1->pause_movement();
    }

    // Need to figure out analog inputs from the ps2 controller before enabling the following :
    // if (droid_1->droideka_rec->digitalState(7))
    // {
    //     if (droid_1->current_position == pked)
    //     {
    //     }
    //     else
    //     {
    //         droid_1->set_movement(Droideka_Movement(upked, droid_1->droideka_rec->analog[1], droid_1->droideka_rec->analog[0], droid_1->droideka_rec->analog[3], droid_1->droideka_rec->analog[2], 500), true);
    //     }
    // }
    // if (droid_1->droideka_rec->digitalRising(7))
    // {
    //     if (droid_1->current_position == pked)
    //     {
    //     }
    //     else
    //     {
    //         droid_1->set_movement(Droideka_Movement(droid_1->current_position, upked, time_ms));
    //     }
    // }

    if (joy_buttons[4] == 1 && last_joy_buttons[4] == 0)
    {
        droid_1->change_mode();
    }

    if (joy_buttons[9] == 1 && last_joy_buttons[9] == 0)
    {
        droid_1->go_to_maintenance();
    }

    if (joy_buttons[6] == 1 && last_joy_buttons[6] == 0)
    {
        droid_1->disable_enable_motors();
    }

    if (joy_buttons[13] == 1 && last_joy_buttons[13] == 0)
    {
        droid_1->roll(100);
    }
    if (joy_buttons[13] == 0 && last_joy_buttons[13] == 1)
    {
        droid_1->roll(0);
    }
    if (joy_buttons[14] == 1 && last_joy_buttons[14] == 0)
    {
        droid_1->roll(-100);
    }
    if (joy_buttons[14] == 0 && last_joy_buttons[14] == 1)
    {
        droid_1->roll(0);
    }

    if (joy_buttons[15] == 1 && last_joy_buttons[15] == 0)
    {
        droid_1->Setpoint = -25.0;
        droid_1->start_pid();
    }
    if (joy_buttons[15] == 0 && last_joy_buttons[15] == 1)
    {
        droid_1->Setpoint = 0.0;
    }
    if (joy_buttons[16] == 1 && last_joy_buttons[16] == 0)
    {
        droid_1->Setpoint = 25.0;
        droid_1->start_pid();
    }
    if (joy_buttons[16] == 0 && last_joy_buttons[16] == 1)
    {
        droid_1->Setpoint = 0.0;
    }
}

ros::Subscriber<sensor_msgs::Joy> joy_sub("/joy", &joyCallback);

void setup()
{
    for (int ii = 0; ii < NB_MAX_DATA; ii++)
    {
        thresholds[ii] = 4;
    }

    // for (int ii = 0; ii < TIME_SAMPLE; ii++)
    // {
    //     trans_x[ii] = 0;
    //     trans_y[ii] = 2.0 * ((float)ii + 1.0) / (float)TIME_SAMPLE;
    //     trans_z[ii] = 0;
    //     rot[ii] = 0;
    // }

    Serial.begin(9600);
    droid_1 = new Droideka(&Serial8, 35, &Serial2, thresholds, BT_HW_HC05, LMP1, LMP_PWM, IMU_INT_PIN);
    nh.initNode();
    nh.subscribe(joy_sub);

    Serial.println("Start");
    Serial.println();
}

void loop()
{
    nh.spinOnce();

    droid_1->check_voltage();
    droid_1->receive_data();
    Droideka_Position upked(droid_1->unparked);
    Droideka_Position pked(droid_1->parked);
    droid_1->read_imu();
    droid_1->compute_pid();
    // if (droid_1->droideka_rec->analogThreshold_2D(0, 1, 200, SUP_OR_EQUAL))
    // {
    //     droid_1->set_movement(Droideka_Movement(upked, -(droid_1->droideka_rec->analog[1] - droid_1->droideka_rec->middle[1]), droid_1->droideka_rec->analog[0] - droid_1->droideka_rec->middle[0], droid_1->droideka_rec->analog[3] - droid_1->droideka_rec->middle[3], -(droid_1->droideka_rec->analog[2] - droid_1->droideka_rec->middle[2]), 7500, true));
    //     droid_1->next_movement_sequence(INTERMEDIATE_SEQUENCE, -(droid_1->droideka_rec->analog[0] - droid_1->droideka_rec->middle[0]), droid_1->droideka_rec->analog[1] - droid_1->droideka_rec->middle[1], -(droid_1->droideka_rec->analog[2] - droid_1->droideka_rec->middle[2]));
    // }
    // if (droid_1->droideka_rec->analogThreshold_2D(0, 1, 200, INF))
    // {
    //     droid_1->next_movement_sequence(FINISHING_SEQUENCE);
    // }
    if (droid_1->droideka_rec->digitalFalling(0) || droid_1->droideka_rec->digitalFalling(1) || droid_1->droideka_rec->digitalFalling(2) || droid_1->droideka_rec->digitalFalling(3))
    {
        start_move = true;
        continue_move = false;
        finish_move = false;
    }
    if (droid_1->droideka_rec->digitalFalling(3))
    {
        x = 2.0;
        y = 0.0;
        z = 0.0;
        alpha = 0.0;
    }
    if (droid_1->droideka_rec->digitalFalling(1))
    {
        x = -2.0;
        y = 0.0;
        z = 0.0;
        alpha = 0.0;
    }
    if (droid_1->droideka_rec->digitalFalling(0))
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        alpha = 1.0;
    }
    if (droid_1->droideka_rec->digitalFalling(2))
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        alpha = -1.0;
    }
    if (start_move)
    {
        droid_1->set_movement(Droideka_Movement(upked, x, y, z, alpha, time_step));
        start_move = false;
    }
    if (droid_1->droideka_rec->digitalState(0) || droid_1->droideka_rec->digitalState(1) || droid_1->droideka_rec->digitalState(2) || droid_1->droideka_rec->digitalState(3))
    {
        start_move = false;
        continue_move = true;
        finish_move = false;
    }
    if (droid_1->droideka_rec->digitalState(3))
    {
        x = 2.0;
        y = 0.0;
        z = 0.0;
        alpha = 0.0;
    }
    if (droid_1->droideka_rec->digitalState(1))
    {
        x = -2.0;
        y = 0.0;
        z = 0.0;
        alpha = 0.0;
    }
    if (droid_1->droideka_rec->digitalState(0))
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        alpha = 1.0;
    }
    if (droid_1->droideka_rec->digitalState(2))
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        alpha = -1.0;
    }
    if (continue_move)
    {
        droid_1->next_movement_sequence(INTERMEDIATE_SEQUENCE, x, y, alpha);
        continue_move = false;
    }
    if (droid_1->droideka_rec->digitalRising(0) || droid_1->droideka_rec->digitalRising(1) || droid_1->droideka_rec->digitalRising(2) || droid_1->droideka_rec->digitalRising(3))
    {
        droid_1->next_movement_sequence(FINISHING_SEQUENCE);
    }
    if (droid_1->droideka_rec->digitalFalling(5))
    {
        droid_1->pause_movement();
    }
    if (droid_1->droideka_rec->digitalState(7))
    {
        if (droid_1->current_position == pked)
        {
        }
        else
        {
            droid_1->set_movement(Droideka_Movement(upked, droid_1->droideka_rec->analog[1], droid_1->droideka_rec->analog[0], droid_1->droideka_rec->analog[3], droid_1->droideka_rec->analog[2], 500), true);
        }
    }
    if (droid_1->droideka_rec->digitalRising(7))
    {
        if (droid_1->current_position == pked)
        {
        }
        else
        {
            droid_1->set_movement(Droideka_Movement(droid_1->current_position, upked, time_ms));
        }
    }
    if (droid_1->droideka_rec->digitalFalling(4))
    {
        droid_1->change_mode();
    }

    if (droid_1->droideka_rec->digitalFalling(9))
    {
        droid_1->go_to_maintenance();
    }

    if (droid_1->droideka_rec->digitalFalling(6))
    {
        droid_1->disable_enable_motors();
    }

    if (droid_1->droideka_rec->digitalFalling(12))
    {
        droid_1->roll(100);
    }
    if (droid_1->droideka_rec->digitalRising(12))
    {
        droid_1->roll(0);
    }
    if (droid_1->droideka_rec->digitalFalling(13))
    {
        droid_1->roll(-100);
    }
    if (droid_1->droideka_rec->digitalRising(13))
    {
        droid_1->roll(0);
    }

    if (droid_1->droideka_rec->digitalFalling(10))
    {
        droid_1->Setpoint = -25.0;
        droid_1->start_pid();
    }
    if (droid_1->droideka_rec->digitalRising(10))
    {
        droid_1->Setpoint = 0.0;
    }
    if (droid_1->droideka_rec->digitalFalling(11))
    {
        droid_1->Setpoint = 25.0;
        droid_1->start_pid();
    }
    if (droid_1->droideka_rec->digitalRising(11))
    {
        droid_1->Setpoint = 0.0;
    }

    droid_1->next_movement();
    droid_1->keep_going();
    droid_1->delayed_function();
}