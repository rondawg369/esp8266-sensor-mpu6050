#include <Wire.h>

const int MPU_addr=0x69;  // I2C address of the MPU-6050


void setup() {
  // put your setup code here, to run once:
  Wire.begin();

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  byte error;
  // We are using the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
  Wire.beginTransmission(MPU_addr);
  error = Wire.endTransmission();
 
  if (error == 0)
    {
      Serial.println("Device Found");
    }
  else
    {
      Serial.println("No Device Found");
    }
  delay(5000);  // Wait 5 seconds and scan again
}
