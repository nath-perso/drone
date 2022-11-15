#include "mpu.h"

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  mpu_setup();
}

void loop() {
  mpu_get_data();

  /* Print the measures */
  mpu_display_data();
  delay(10);
}