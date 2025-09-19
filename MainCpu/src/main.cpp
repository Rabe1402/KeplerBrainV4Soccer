#include "KeplerBRAIN_V4.h"

void setup() {

    KEPLERBRAIN_INIT();

    Serial.begin(115200);
    Serial.println("Seas");

}

void loop() {

    WRITE_MOTOR(M1, 50);
    WRITE_MOTOR(M2, 50);
    delay(2000);
    WRITE_MOTOR(M1, -50);
    WRITE_MOTOR(M2, -50);
    delay(2000);
}