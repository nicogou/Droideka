#include <Droideka.h>
#define LMP1 52
#define LMP2 53
#define LMP_PWM 2
#define REC_RX 51
#define REC_TX 50
#define REC_STATE 48

Droideka *droid_1;

int time = 500;
// float parking[LEG_NB][3] = {{0.0, 14, 0}, {0, 14, 0}, {0, 14, 0}, {0, 14, 0}};
// float pos_2[LEG_NB][3] = {{0.0, 7.0, 7.0}, {0.0, 7.0, 7.0}, {0.0, 7.0, 7.0}, {0.0, 7.0, 7.0}};
float parking[LEG_NB][3] = {{0.0, 2.0, -10.0}, {0.0, 2.0, -10.0}, {0.0, 2.0, -10.0}, {0.0, 2.0, -10.0}};
float pos_2[LEG_NB][3] = {{0.0, 2.0, -8.6}, {0.0, 2.0, -8.6}, {0.0, 2.0, -8.6}, {0.0, 2.0, -8.6}};
Droideka_Position parking_pos(parking);
Droideka_Position pos_2_pos(pos_2);

void setup()
{
    Serial1.begin(DEBUG_BOARD_BAUD_RATE);
    Serial2.begin(DEBUG_BOARD_BAUD_RATE);
    droid_1 = new Droideka(&Serial2, &Serial1);
    Serial.begin(9600);
    droid_1->initialize(LMP1, LMP2, LMP_PWM, REC_RX, REC_TX, REC_STATE);
    Action test;
    test.set_active();
    Serial.println("Start");

    ErrorCode result;
    for (int repeat = 0; repeat < 5; repeat++)
    {
        Serial.print("Iteration ");
        Serial.print(repeat);
        Serial.println(" started");
        result = droid_1->in_position(parking_pos, test, time);
        Serial.println(result);

        if (result == NO_ERROR)
        {
            droid_1->act(&test);
        }

        delay(time + 1000);

        result = droid_1->in_position(pos_2_pos, test, time);
        Serial.println(result);

        if (result == NO_ERROR)
        {
            droid_1->act(&test);
        }

        delay(time + 1000);
        Serial.print("Iteration ");
        Serial.print(repeat);
        Serial.println(" finished");
    }
    Serial.println("Finished");
}

void loop()
{
}