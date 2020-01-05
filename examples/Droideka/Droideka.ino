#include <Droideka.h>

Droideka droid_1;

void setup()
{
    Serial.begin(38400);
    droid_1.initialize();
}

void loop()
{
    Serial.println(droid_1.move());
    delay(5000);
}