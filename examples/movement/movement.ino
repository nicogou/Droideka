#include <Droideka.h>
#define LMP1 2
#define LMP2 3
#define LMP_PWM 4

Droideka *droid_1;

int16_t thresholds[NB_MAX_DATA];
int time_ms = 5000;

void setup()
{
    for (int ii = 0; ii < NB_MAX_DATA; ii++)
    {
        thresholds[ii] = 4;
    }

    Serial.begin(9600);
    droid_1 = new Droideka(&Serial8, 35, &Serial2, thresholds, BT_HW_HC05, LMP1, LMP2, LMP_PWM);

    Serial.println("Start");
    Serial.println();
}

void loop()
{
    droid_1->receive_data();
    if (droid_1->droideka_rec->isUpdated.bluetooth() && droid_1->droideka_rec->digitalFalling(0))
    {
        Droideka_Position curr(droid_1->unparked);
        Serial.println("Current Position");
        curr.print_position();
        droid_1->movement = Droideka_Movement(curr, 0, 0, 0, 0, true);
        Droideka_Position temp;
        for (int ii = 0; ii < TIME_SAMPLE; ii++)
        {
            if (ii == TIME_SAMPLE - 1)
            {
                temp = Droideka_Position(droid_1->unparked);
            }
            else
            {
                temp = droid_1->movement.positions[ii];
            }
            droid_1->move_into_position(temp, time_ms / TIME_SAMPLE);
            delay(time_ms / TIME_SAMPLE);
        }
    }
    if (droid_1->droideka_rec->isUpdated.bluetooth() && droid_1->droideka_rec->digitalFalling(1))
    {
        Droideka_Position unparked_(droid_1->unparked);
        droid_1->move_into_position(unparked_, 1000);
        delay(1000);
    }
    if (droid_1->droideka_rec->isUpdated.bluetooth() && droid_1->droideka_rec->digitalFalling(2))
    {
        Droideka_Position unparking_(droid_1->unparking);
        droid_1->move_into_position(unparking_, 1000);
        delay(1000);
    }
    if (droid_1->droideka_rec->isUpdated.bluetooth() && droid_1->droideka_rec->digitalFalling(4))
    {
        droid_1->change_mode();
    }

    if (droid_1->droideka_rec->isUpdated.bluetooth() && droid_1->droideka_rec->digitalFalling(9))
    {
        droid_1->go_to_maintenance();
    }

    if (droid_1->droideka_rec->digitalFalling(6))
    {
        droid_1->disable_enable_motors();
    }
}