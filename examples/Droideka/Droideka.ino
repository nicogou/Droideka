#include <Droideka.h>
#define LMP1 52
#define LMP2 53
#define LMP_PWM 2
#define REC_RX 51
#define REC_TX 50
#define REC_STATE 48

Droideka *droid_1;

float parking[LEG_NB][3] = {{90.0, 9.8, 9.0}, {90, 9.8, 9.0}, {90, 9.8, 9.0}, {90, 9.8, 9.0}};
Droideka_Position parking_pos(parking);

void setup()
{
    Serial1.begin(DEBUG_BOARD_BAUD_RATE);
    Serial2.begin(DEBUG_BOARD_BAUD_RATE);
    droid_1 = new Droideka(&Serial2, &Serial1);
    Serial.begin(9600);
    droid_1->initialize(LMP1, LMP2, LMP_PWM, REC_RX, REC_TX, REC_STATE);
    droid_1->set_parking_position(&parking_pos);
    Serial.println("Start");
    Serial.println("Get into parking position");
    Serial.println(droid_1->park());
}

void loop()
{
    // if (droid_1->receive_data())
    // {
    //     Serial.print(droid_1->move('l', droid_1->throttle_x));
    //     Serial.print("\tThrottle_x: ");
    //     Serial.print(droid_1->throttle_x);
    //     /*Serial.print("\tThrottle_y: ");
    //     Serial.print(droid_1->throttle_y);
    //     Serial.print("\tButton 1: ");
    //     Serial.print(droid_1->button1);
    //     Serial.print("\tButton 2: ");
    //     Serial.print(droid_1->button2);
    //     Serial.print("\tButton 3: ");
    //     Serial.print(droid_1->button3);*/
    //     Serial.println();
    // }
}