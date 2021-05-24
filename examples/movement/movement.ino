#include <Droideka.h>
#define LMP1 2
#define LMP2 3
#define LMP_PWM 4

Droideka *droid_1;

int16_t thresholds[NB_MAX_DATA];
int time_ms = 1000;
float trans_x[TIME_SAMPLE];
float trans_y[TIME_SAMPLE];
float trans_z[TIME_SAMPLE];
float rot[TIME_SAMPLE];

void setup()
{
    for (int ii = 0; ii < NB_MAX_DATA; ii++)
    {
        thresholds[ii] = 4;
    }

    for (int ii = 0; ii < TIME_SAMPLE; ii++)
    {
        trans_x[ii] = 0;
        trans_y[ii] = 2.0 * ((float)ii + 1.0) / (float)TIME_SAMPLE;
        trans_z[ii] = 0;
        rot[ii] = 0;
    }

    Serial.begin(9600);
    droid_1 = new Droideka(&Serial8, 35, &Serial2, thresholds, BT_HW_HC05, LMP1, LMP2, LMP_PWM);

    Serial.println("Start");
    Serial.println();
}

void loop()
{
    droid_1->check_voltage();
    droid_1->receive_data();
    Droideka_Position upked(droid_1->unparked);

    // if (droid_1->droideka_rec->analogThreshold_2D(0, 1, 200, SUP_OR_EQUAL))
    // {
    //     droid_1->set_movement(Droideka_Movement(upked, -(droid_1->droideka_rec->analog[1] - droid_1->droideka_rec->middle[1]), droid_1->droideka_rec->analog[0] - droid_1->droideka_rec->middle[0], droid_1->droideka_rec->analog[3] - droid_1->droideka_rec->middle[3], -(droid_1->droideka_rec->analog[2] - droid_1->droideka_rec->middle[2]), 7500, true));
    //     droid_1->next_movement_sequence(INTERMEDIATE_SEQUENCE, -(droid_1->droideka_rec->analog[0] - droid_1->droideka_rec->middle[0]), droid_1->droideka_rec->analog[1] - droid_1->droideka_rec->middle[1], -(droid_1->droideka_rec->analog[2] - droid_1->droideka_rec->middle[2]));
    // }
    // if (droid_1->droideka_rec->analogThreshold_2D(0, 1, 200, INF))
    // {
    //     droid_1->next_movement_sequence(FINISHING_SEQUENCE);
    // }
    if (droid_1->droideka_rec->digitalFalling(0))
    {
        droid_1->set_movement(Droideka_Movement(upked, (float)0.0, (float)0.0, (float)0.0, (float)1.0, 7500));
    }
    if (droid_1->droideka_rec->digitalState(0))
    {
        droid_1->next_movement_sequence(INTERMEDIATE_SEQUENCE, (float)0.0, (float)0.0, (float)1.0);
    }
    if (droid_1->droideka_rec->digitalRising(0))
    {
        droid_1->next_movement_sequence(FINISHING_SEQUENCE);
    }
    if (droid_1->droideka_rec->digitalFalling(5))
    {
        droid_1->pause_movement();
    }
    if (droid_1->droideka_rec->digitalState(7))
    {
        droid_1->set_movement(Droideka_Movement(upked, droid_1->droideka_rec->analog[1], droid_1->droideka_rec->analog[0], droid_1->droideka_rec->analog[3], droid_1->droideka_rec->analog[2], 500), true);
    }
    if (droid_1->droideka_rec->digitalRising(7))
    {
        droid_1->set_movement(Droideka_Movement(droid_1->current_position, upked, time_ms), true);
    }
    if (droid_1->droideka_rec->digitalFalling(3))
    {
        droid_1->stop_movement();
    }
    if (droid_1->droideka_rec->digitalFalling(1))
    {
        droid_1->set_movement(Droideka_Movement(droid_1->current_position, upked, time_ms));
    }
    if (droid_1->droideka_rec->digitalFalling(2))
    {
        Droideka_Position unparking_(droid_1->unparking);
        droid_1->set_movement(Droideka_Movement(droid_1->current_position, unparking_, time_ms));
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
    droid_1->next_movement();
    droid_1->keep_going();
}