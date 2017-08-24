#include <Wire.h>

const int MPU_addr=0x68;  // I2C address of the MPU-6050
const int MPU_addr2=0x69; // Second I2C address of the MPU-6050

void setup() {
  pinMode(D0,OUTPUT);  
  
  Wire.begin();
  Serial.begin(115200);
}

void loop() {
 
  
  digitalWrite(D0,HIGH);
  Serial.print("D0 is HIGH ");
  check_I2c(MPU_addr);  
  check_I2c(MPU_addr2);
  Serial.println(" ");
  
  delay(1000);  // Wait 1 second, change address then scan
 
  digitalWrite(D0,LOW);
  Serial.print("D0 is LOW ");
  check_I2c(MPU_addr);  
  check_I2c(MPU_addr2);
  Serial.println(" ");
  
    
  delay(5000);  // Wait 5 seconds and scan again
}


byte check_I2c(byte addr){
   // We are using the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
  byte error;
  Wire.beginTransmission(addr);
  error = Wire.endTransmission();
  
  if (error == 0)
    {
      Serial.print(" Device Found at 0x");
      Serial.print(addr,HEX);
    }
  else
    {
      Serial.print(" No Device Found at 0x");
      Serial.print(addr,HEX);
    }
  return error;
  }
