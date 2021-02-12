
#include <Droideka.h>
#define LMP1 52
#define LMP2 53
#define LMP_PWM 2
#define REC_RX 51
#define REC_TX 50
#define REC_STATE 48

Droideka *droid_1;

void print_(String string, bool newline = false)
{
    string = string + "\t";
    Serial.print(string);
    if (newline)
    {
        Serial.println();
    }
}

int time = 2000;
int time_offset = 2000;

void setup()
{
    Serial.begin(9600);
    Serial1.begin(DEBUG_BOARD_BAUD_RATE);
    Serial2.begin(DEBUG_BOARD_BAUD_RATE);
    droid_1 = new Droideka(&Serial2, &Serial1);
    droid_1->initialize(LMP1, LMP2, LMP_PWM, REC_RX, REC_TX, REC_STATE);
}

void loop()
{
    bool test = droid_1->receive_data();
    if (test)
    {
        Serial.print("Received\t\t");
        print_("Button 1: " + String(droid_1->button1));
        print_("Button 2: " + String(droid_1->button2));
        print_("Button 3: " + String(droid_1->button3));
        print_("Throttle X: " + String(droid_1->throttle_x));
        print_("Throttle Y: " + String(droid_1->throttle_y));
        print_("", true);
        if (droid_1->button2_clicked())
        {
            droid_1->change_mode();
        }
        Serial.println();
    }
    // droid_1->move(droid_1->throttle_x);
}