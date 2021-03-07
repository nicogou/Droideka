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
    droid_1->receive_data();
    if (droid_1->droideka_rec->digitalFalling(0))
    {
        Droideka_Position curr(droid_1->unparked);
        Serial.println("Current Position");
        curr.print_position();
        droid_1->movement = Droideka_Movement(curr, trans_x, trans_y, trans_z, rot, time_ms);
    }
    if (droid_1->droideka_rec->digitalFalling(1))
    {
        Droideka_Position unparked_(droid_1->unparked);
        droid_1->move_into_position(unparked_, 1000);
        delay(1000);
    }
    if (droid_1->droideka_rec->digitalFalling(2))
    {
        Droideka_Position unparking_(droid_1->unparking);
        droid_1->move_into_position(unparking_, 1000);
        delay(1000);
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
}