#include <Droideka.h>
#define LMP1 3
#define LMP_PWM 4
#define IMU_INT_PIN 24

Droideka *droid_1;

int16_t thresholds[NB_MAX_DATA];
int time_ms = 1000;
int time_step = 1500;
int time_trot = 300;
float x = 0.0, y = 0.0, z = 0.0, alpha = 0.0;
bool start_move = false, continue_move = false, finish_move = false;
float pid_set = 25.0;

void setup()
{
    for (int ii = 0; ii < NB_MAX_DATA; ii++)
    {
        thresholds[ii] = 4;
    }

    Serial.begin(9600);
    droid_1 = new Droideka(&Serial8, 35, &Serial2, thresholds, BT_HW_HC05, LMP1, LMP_PWM, IMU_INT_PIN);
    // droid_1->long_pid->SetTunings(4.0, 0.0, 0.0);

    Serial.println("Start");
    Serial.println();
}

void loop()
{
    droid_1->check_voltage();
    droid_1->receive_data();

    droid_1->read_imu();
    droid_1->compute_pid();

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
    if (droid_1->droideka_rec->digitalFalling(0) || droid_1->droideka_rec->digitalFalling(1) || droid_1->droideka_rec->digitalFalling(2) || droid_1->droideka_rec->digitalFalling(3))
    {
        start_move = true;
        continue_move = false;
        finish_move = false;
    }
    if (start_move)
    {
        if (droid_1->droideka_rec->digitalState(7))
        {
            droid_1->set_movement(Droideka_Movement(TROT_GAIT, Droideka_Position(droid_1->unparked), x, y, z, alpha, time_trot));
        }
        else
        {
            droid_1->set_movement(Droideka_Movement(STABLE_GAIT, Droideka_Position(droid_1->unparked), x, y, z, alpha, time_step));
        }
        start_move = false;
    }
    if (droid_1->droideka_rec->digitalState(0) || droid_1->droideka_rec->digitalState(1) || droid_1->droideka_rec->digitalState(2) || droid_1->droideka_rec->digitalState(3))
    {
        start_move = false;
        continue_move = true;
        finish_move = false;
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
        droid_1->Setpoint = -pid_set;
        droid_1->start_pid();
    }
    if (droid_1->droideka_rec->digitalRising(10))
    {
        droid_1->Setpoint = 0.0;
    }
    if (droid_1->droideka_rec->digitalFalling(11))
    {
        droid_1->Setpoint = pid_set;
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