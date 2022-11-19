#include "mpu.h"

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  mpu_setup();
}

void loop() {
  mpu_get_data();

  /* Print the measures */
  mpu_display_data();
  delay(10);
}