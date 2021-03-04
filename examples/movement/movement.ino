
#include <Droideka.h>
#define LMP1 2
#define LMP2 3
#define LMP_PWM 4

Droideka *droid_1;

int time = 2000;
int time_offset = 2000;
double last_millis = 0;
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
        Droideka_Position curr = droid_1->get_current_position();
        Serial.println("Current Position");
        curr.print_position();
        droid_1->movement = new Droideka_Movement(curr, 0, 0);
        Droideka_Position temp;
        for (int ii = 0; ii < TIME_SAMPLE; ii++)
        {
            Serial.println("Position " + String(ii + 1));
            temp = droid_1->movement->get_future_position(curr, droid_1->movement->tx, droid_1->movement->ty, droid_1->movement->alpha, ii);
            temp.print_position();
            droid_1->move_into_position(temp, time_ms / TIME_SAMPLE);
            delay(time_ms / TIME_SAMPLE);
        }
    }

    if (droid_1->droideka_rec->isUpdated.bluetooth() && droid_1->droideka_rec->digitalFalling(4))
    {
        droid_1->change_mode();
    }

    if (droid_1->droideka_rec->isUpdated.bluetooth() && droid_1->droideka_rec->digitalFalling(5))
    {
        droid_1->go_to_maintenance();
    }
}