 #include <Wire.h>
 #include <MPU6050.h>

const uint8_t MPU_addr=0x68;  // I2C address of the MPU-6050

// Scaling Constants 
const float MPU_GYRO_250_SCALE = 131.0;
const float MPU_GYRO_500_SCALE = 65.5;
const float MPU_GYRO_1000_SCALE = 32.8;
const float MPU_GYRO_2000_SCALE = 16.4;
const float MPU_ACCL_2_SCALE = 16384.0;
const float MPU_ACCL_4_SCALE = 8192.0;
const float MPU_ACCL_8_SCALE = 4096.0;
const float MPU_ACCL_16_SCALE = 2048.0;

struct rawdata {
  int16_t AcX;
  int16_t AcY;
  int16_t AcZ;
  int16_t Tmp;
  int16_t GyX;
  int16_t GyY;
  int16_t GyZ;
  };

struct scaleddata{
  float AcX;
  float AcY;
  float AcZ;
  float Tmp;
  float GyX;
  float GyY;
  float GyZ;
  };

struct STREG{
  uint8_t reg_13;
  uint8_t reg_14;
  uint8_t reg_15;
  uint8_t reg_16;
  } stRegs;

struct TESTVals{
  uint8_t XA_TEST;
  uint8_t YA_TEST;
  uint8_t ZA_TEST;
  uint8_t XG_TEST;
  uint8_t YG_TEST;
  uint8_t ZG_TEST;
  } TEST;


struct FTVals {
  float Xg;
  float Yg;
  float Zg;
  float Xa;
  float Ya;
  float Za;
  } FT;


void check_I2c(uint8_t addr);
void mpu6050Begin(uint8_t addr);
rawdata mpu6050Read(uint8_t addr, bool Debug);
void setMPU6050scales(uint8_t addr,uint8_t Gyro,uint8_t Accl);
void getMPU6050scales(uint8_t addr,uint8_t &Gyro,uint8_t &Accl);
scaleddata convertRawToScaled(uint8_t addr, rawdata data_in,bool Debug);
void calibrateMPU6050(uint8_t addr, rawdata &offsets,char up_axis, int num_samples,bool Debug);
rawdata averageSamples(rawdata * samps,int len);

void readSelfTestRegisters(uint8_t addr, STREG &RegsIn,bool Debug);
void assembleTESTValues(STREG Regs,TESTVals &Tests, bool Debug);
void generateFTValues(TESTVals Tests,FTVals &ftVal,bool Debug);
float FT_accel_calc(uint8_t TEST);
float FT_gyro_calc(uint8_t TEST);
FTVals runSelfTest(uint8_t addr, bool Debug);
float change(float STR,float FT);
void evalSelfTest(FTVals STR,FTVals FT,bool Debug);


rawdata offsets;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  mpu6050Begin(MPU_addr);
}

void loop() {
  FTVals STR;
  // Read Factory Trim Settings
  readSelfTestRegisters(MPU_addr, stRegs,true);
  assembleTESTValues(stRegs,TEST, true);
  generateFTValues(TEST,FT,true);

  // Run Self Test
  STR = runSelfTest(MPU_addr, true);
  
  // Evaluate Self Test results
  evalSelfTest(STR,FT,true);
  
  delay(5000);  // Wait 5 seconds and scan again
}


float change(float STR,float FT){
  // This is a simple calculation to determine the 
  // percent change.
  float perc;
  perc = (STR-FT)/FT;
  return perc;
  }


void mpu6050Begin(uint8_t addr){
// This function initializes the MPU-6050 IMU Sensor
//  It verifys the address is correct and wakes up the
//  MPU.
  if (checkI2c(addr)){
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

    delay(30);  // Ensure gyro has enough time to power up
    
    calibrateMPU6050(addr,offsets,'X', 10,true);
  }
}

bool checkI2c(uint8_t addr){
   // We are using the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
  Serial.println(" ");
  Wire.beginTransmission(addr);
  
  if (Wire.endTransmission() == 0)
    {
      Serial.print(" Device Found at 0x");
      Serial.println(addr,HEX);
      return true;
    }
  else
    {
      Serial.print(" No Device Found at 0x");
      Serial.println(addr,HEX);
      return false;
    }
}


rawdata mpu6050Read(uint8_t addr, bool Debug){
  // This function reads the raw 16-bit data values from
  // the MPU-6050

  rawdata values; 
  
  Wire.beginTransmission(addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(addr,14,true); // request a total of 14 registers
  values.AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
  values.AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  values.AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  values.Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  values.GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  values.GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  values.GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)


  values.AcX-=offsets.AcX;
  values.AcY-=offsets.AcY;
  values.AcZ-=offsets.AcZ;
  values.GyX-=offsets.GyX;
  values.GyY-=offsets.GyY;
  values.GyZ-=offsets.GyZ;
  
  if(Debug){
    Serial.print(" GyX = "); Serial.print(values.GyX);
    Serial.print(" | GyY = "); Serial.print(values.GyY);
    Serial.print(" | GyZ = "); Serial.print(values.GyZ);
    Serial.print(" | Tmp = "); Serial.print(values.Tmp); 
    Serial.print(" | AcX = "); Serial.print(values.AcX);
    Serial.print(" | AcY = "); Serial.print(values.AcY);
    Serial.print(" | AcZ = "); Serial.println(values.AcZ);  
  }

  return values;
}

void setMPU6050scales(uint8_t addr,uint8_t Gyro,uint8_t Accl){
  Wire.beginTransmission(addr);
  Wire.write(0x1B);  // write to register starting at 0x1B
  Wire.write(Gyro); // Self Tests Off and set Gyro FS to 250
  Wire.write(Accl); // Self Tests Off and set Accl FS to 8g
  Wire.endTransmission(true);
  calibrateMPU6050(addr,offsets,'X', 10,true);
}

void getMPU6050scales(uint8_t addr,uint8_t &Gyro,uint8_t &Accl){
  Wire.beginTransmission(addr);
  Wire.write(0x1B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(addr,2,true); // request a total of 14 registers
  Gyro = (Wire.read()&(bit(3)|bit(4)))>>3;
  Accl = (Wire.read()&(bit(3)|bit(4)))>>3;
}


scaleddata convertRawToScaled(uint8_t addr, rawdata data_in, bool Debug){

  scaleddata values;
  float scale_value = 0.0;
  uint8_t Gyro, Accl;

  getMPU6050scales(MPU_addr, Gyro, Accl);
  
  if(Debug){
    Serial.print("Gyro Full-Scale = ");
    }
  
  switch (Gyro){
    case 0:
      scale_value = MPU_GYRO_250_SCALE;
      if(Debug){
        Serial.println("±250 °/s");
        }
    break;
    case 1:
      scale_value = MPU_GYRO_500_SCALE;
    if(Debug){
        Serial.println("±500 °/s");
        }
    break;
    case 2:
      scale_value = MPU_GYRO_1000_SCALE;
    if(Debug){
        Serial.println("±1000 °/s");
        }
    break;
    case 3:
      scale_value = MPU_GYRO_2000_SCALE;
    if(Debug){
        Serial.println("±2000 °/s");
        }
    break;
    default:
    break;
  }

  values.GyX = (float) data_in.GyX / scale_value;
  values.GyY = (float) data_in.GyY / scale_value;
  values.GyZ = (float) data_in.GyZ / scale_value;

  scale_value = 0.0;
  if(Debug){
    Serial.print("Accl Full-Scale = ");
    }
  switch (Accl){
    case 0:
      scale_value = MPU_ACCL_2_SCALE;
      if(Debug){
        Serial.println("±2 g");
        }
    break;
    case 1:
      scale_value = MPU_ACCL_4_SCALE;
      if(Debug){
        Serial.println("±4 g");
        }
    break;
    case 2:
      scale_value = MPU_ACCL_8_SCALE;
      if(Debug){
        Serial.println("±8 g");
        }
    break;
    case 3:
      scale_value = MPU_ACCL_16_SCALE;
      if(Debug){
        Serial.println("±16 g");
        }
    break;
    default:
    break;
  }
  values.AcX = (float) data_in.AcX / scale_value;
  values.AcY = (float) data_in.AcY / scale_value;
  values.AcZ = (float) data_in.AcZ / scale_value;


  values.Tmp = (float) data_in.Tmp / 340.0 + 36.53;

  if(Debug){
    Serial.print(" GyX = "); Serial.print(values.GyX);
    Serial.print(" °/s| GyY = "); Serial.print(values.GyY);
    Serial.print(" °/s| GyZ = "); Serial.print(values.GyZ);
    Serial.print(" °/s| Tmp = "); Serial.print(values.Tmp); 
    Serial.print(" °C| AcX = "); Serial.print(values.AcX);
    Serial.print(" g| AcY = "); Serial.print(values.AcY);
    Serial.print(" g| AcZ = "); Serial.print(values.AcZ);Serial.println(" g");  
  }

  return values;
 }


  void calibrateMPU6050(uint8_t addr,rawdata &offsets,char up_axis ,int num_samples, bool Debug){
    // This function reads in the first num_samples and averages them
    //  to determine calibration offsets, which are then used in 
    //  when the sensor data is read.

    //  It simply assumes that the up_axis is vertical and that the sensor is not
    //  moving.
    rawdata temp[num_samples];
    int scale_value;
    uint8_t Gyro, Accl;

    for(int i=0; i<num_samples; i++){
      temp[i] = mpu6050Read(addr,false);
      }

    offsets = averageSamples(temp,num_samples);
    getMPU6050scales(MPU_addr, Gyro, Accl);
    
    switch (Accl){
      case 0:
        scale_value = (int)MPU_ACCL_2_SCALE;
      break;
      case 1:
        scale_value = (int)MPU_ACCL_4_SCALE;
      break;
      case 2:
        scale_value = (int)MPU_ACCL_8_SCALE;
      break;
      case 3:
        scale_value = (int)MPU_ACCL_16_SCALE;
      break;
      default:
      break;
    }
    
    
    switch(up_axis){
      case 'X':
        offsets.AcX -= scale_value;
      break;
      case 'Y':
        offsets.AcY -= scale_value;
      break;
      case 'Z':
        offsets.AcZ -= scale_value;
      break;
      default:
      break;
    }
    if(Debug){
      Serial.print(" Offsets:  GyX = "); Serial.print(offsets.GyX);
      Serial.print(" | GyY = "); Serial.print(offsets.GyY);
      Serial.print(" | GyZ = "); Serial.print(offsets.GyZ);
      Serial.print(" | AcX = "); Serial.print(offsets.AcX);
      Serial.print(" | AcY = "); Serial.print(offsets.AcY);
      Serial.print(" | AcZ = "); Serial.println(offsets.AcZ);  
      }
  }

  rawdata averageSamples(rawdata * samps,int len){
     rawdata out_data;
     scaleddata temp;
     
     temp.GyX = 0.0;
     temp.GyY = 0.0;
     temp.GyZ = 0.0;
     temp.AcX = 0.0;
     temp.AcY = 0.0;
     temp.AcZ = 0.0;
     
     for(int i = 0; i < len; i++){
     temp.GyX += (float)samps[i].GyX;
     temp.GyY += (float)samps[i].GyY;
     temp.GyZ += (float)samps[i].GyZ;
     temp.AcX += (float)samps[i].AcX;
     temp.AcY += (float)samps[i].AcY;
     temp.AcZ += (float)samps[i].AcZ;
      } 

     out_data.GyX = (int16_t)(temp.GyX/(float)len);
     out_data.GyY = (int16_t)(temp.GyY/(float)len);
     out_data.GyZ = (int16_t)(temp.GyZ/(float)len);
     out_data.AcX = (int16_t)(temp.AcX/(float)len);
     out_data.AcY = (int16_t)(temp.AcY/(float)len);
     out_data.AcZ = (int16_t)(temp.AcZ/(float)len);

     return out_data;
     
    }
  



void readSelfTestRegisters(uint8_t addr, STREG &RegsIn,bool Debug){
  // This function reads the self test registers and stores them in RegsIn
  // If Debug is true then it sends the contents of the registers to the serial port
  size_t temp;
  Wire.beginTransmission(addr);
  Wire.write(0x0D);  // starting with register 0x0D (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  temp = Wire.requestFrom(addr,4,true);  // request a total of 4 registers
  RegsIn.reg_13=Wire.read();      
  RegsIn.reg_14=Wire.read();  
  RegsIn.reg_15=Wire.read();  
  RegsIn.reg_16=Wire.read();
 
  if(Debug){
    Serial.print("Register 13 = "); Serial.print(RegsIn.reg_13,BIN);
    Serial.print(" | Register 14 = "); Serial.print(RegsIn.reg_14,BIN);
    Serial.print(" | Register 15 = "); Serial.print(RegsIn.reg_15,BIN);
    Serial.print(" | Register 16  = "); Serial.println(RegsIn.reg_16,BIN);
  }
}


void assembleTESTValues(STREG Regs,TESTVals &Tests, bool Debug){
  // This function converts the self test registers to the appropriate
  // TEST values which are then later used to calculate the FT Values.
  
  Tests.XG_TEST = Regs.reg_13&0b00011111;
  Tests.YG_TEST = Regs.reg_14&0b00011111;
  Tests.ZG_TEST = Regs.reg_15&0b00011111;
  Tests.XA_TEST = ((Regs.reg_13&0b11100000)>>3)|((Regs.reg_16>>4)&3);
  Tests.YA_TEST = ((Regs.reg_14&0b11100000)>>3)|((Regs.reg_16>>2)&3);
  Tests.ZA_TEST = ((Regs.reg_15&0b11100000)>>3)|((Regs.reg_16)&3); 

   if(Debug){
    Serial.print("XG_TEST = "); Serial.print(Tests.XG_TEST,BIN);
    Serial.print(" | YG_TEST = "); Serial.print(Tests.YG_TEST,BIN);
    Serial.print(" | ZG_TEST = "); Serial.print(Tests.ZG_TEST,BIN);
    Serial.print(" | XA_TEST = "); Serial.print(Tests.XA_TEST,BIN);
    Serial.print(" | YA_TEST = "); Serial.print(Tests.YA_TEST,BIN);
    Serial.print(" | ZA_TEST = "); Serial.println(Tests.ZA_TEST,BIN);
  }
}

void generateFTValues(TESTVals Tests,FTVals &ftVal,bool Debug){
  // This function generates the FT Values from the TEST values
  
  ftVal.Xg = FT_gyro_calc(Tests.XG_TEST);
  ftVal.Yg = -FT_gyro_calc(Tests.YG_TEST);  // The Y axis gyro setting is negative 
  ftVal.Zg = FT_gyro_calc(Tests.ZG_TEST);
  ftVal.Xa = FT_accel_calc(Tests.XA_TEST);
  ftVal.Ya = FT_accel_calc(Tests.YA_TEST);
  ftVal.Za = FT_accel_calc(Tests.ZA_TEST);
  
  if(Debug){
    Serial.print(" FT_XG = "); Serial.print(ftVal.Xg);
    Serial.print(" | FT_YG = "); Serial.print(ftVal.Yg);
    Serial.print(" | FT_ZG = "); Serial.print(ftVal.Zg);
    Serial.print(" | FT_XA = "); Serial.print(ftVal.Xa);
    Serial.print(" | FT_YA = "); Serial.print(ftVal.Ya);
    Serial.print(" | FT_ZA = "); Serial.println(ftVal.Za);  
  }
}

float FT_accel_calc(uint8_t TEST){
  // This is the Acceleration FT calculation
  float value = 0.0;
  if (TEST !=0){
    value = 4096*0.34*pow((0.92/0.34),(((float)TEST-1)/(pow(2,5)-2)));
  }
  return value;
}

float FT_gyro_calc(uint8_t TEST){
  // This is the gyro FT calculation
  float value = 0.0;
  if (TEST !=0){
    value = 25*131*pow(1.046,(float)TEST-1);
  }
  return value;
}

FTVals runSelfTest(uint8_t addr, bool Debug){
  // This function runs the self test, it begins by acquiring data when 
  //  the self test is off, then it triggers the self test, collects the
  //  data again and returns the STR values

  FTVals str;
  rawdata st_off, st_on;
    
  Wire.beginTransmission(addr);
  Wire.write(0x1B);  // write to register starting at 0x1B
  Wire.write(0b0000000); // Self Tests Off and set Gyro FS to 250
  Wire.write(0b0001000); // Self Tests Off and set Accl FS to 8g
  Wire.endTransmission(true);


  Wire.beginTransmission(addr);
  Wire.write(0x1B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(addr,2,true); // request a total of 14 registers
  Serial.print(" Gyro Config = ");Serial.println(Wire.read(),BIN);
  Serial.print(" Accl Config = ");Serial.println(Wire.read(),BIN);
  
  delay(500);
  
  Wire.beginTransmission(addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(addr,14,true); // request a total of 14 registers
  st_off.AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
  st_off.AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  st_off.AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  st_off.Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  st_off.GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  st_off.GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  st_off.GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
  Wire.beginTransmission(addr);
  Wire.write(0x1B);  // write to register starting at 0x1B
  Wire.write(0b1110000); // Initiate Self Tests and set Gyro FS to 250
  Wire.write(0b1111000); // Initiate Self Tests and set Accl FS to 8g
  Wire.endTransmission(true);

  Wire.beginTransmission(addr);
  Wire.write(0x1B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(addr,2,true); // request a total of 14 registers
  Serial.print(" Gyro Config = ");Serial.println(Wire.read(),BIN);
  Serial.print(" Accl Config = ");Serial.println(Wire.read(),BIN);

  delay(1000);

  Wire.beginTransmission(addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(addr,14,true); // request a total of 14 registers
  st_on.AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
  st_on.AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  st_on.AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  st_on.Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  st_on.GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  st_on.GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  st_on.GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Wire.beginTransmission(addr);
  Wire.write(0x1B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(addr,2,true); // request a total of 14 registers
  Serial.print(" Gyro Config = ");Serial.println(Wire.read(),BIN);
  Serial.print(" Accl Config = ");Serial.println(Wire.read(),BIN);
  

  Wire.beginTransmission(addr);
  Wire.write(0x1B);  // write to register starting at 0x1B
  Wire.write(0b0000000); // Self Tests Off and set Gyro FS to 250
  Wire.write(0b0001000); // Self Tests Off and set Accl FS to 8g
  Wire.endTransmission(true);

  Wire.beginTransmission(addr);
  Wire.write(0x1B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(addr,2,true); // request a total of 2 registers
  Serial.print(" Gyro Config = ");Serial.println(Wire.read(),BIN);
  Serial.print(" Accl Config = ");Serial.println(Wire.read(),BIN);

  delay(1000);
  
  str.Xg = (float)st_on.GyX - (float)st_off.GyX;
  str.Yg = (float)st_on.GyY - (float)st_off.GyY;
  str.Zg = (float)st_on.GyZ - (float)st_off.GyZ;
  str.Xa = (float)st_on.AcX - (float)st_off.AcX;
  str.Ya = (float)st_on.AcY - (float)st_off.AcY;
  str.Za = (float)st_on.AcZ - (float)st_off.AcZ;

  
  if(Debug){
    Serial.print(" GyX (off) = "); Serial.print(st_off.GyX);
    Serial.print(" | GyY (off) = "); Serial.print(st_off.GyY);
    Serial.print(" | GyZ (off) = "); Serial.print(st_off.GyZ);
    Serial.print(" | Tmp (off) = "); Serial.print(st_off.Tmp); 
    Serial.print(" | AcX (off) = "); Serial.print(st_off.AcX);
    Serial.print(" | AcY (off) = "); Serial.print(st_off.AcY);
    Serial.print(" | AcZ (off) = "); Serial.println(st_off.AcZ);
  
    Serial.print(" GyX (on) = "); Serial.print(st_on.GyX);
    Serial.print(" | GyY (on) = "); Serial.print(st_on.GyY);
    Serial.print(" | GyZ (on) = "); Serial.print(st_on.GyZ);
    Serial.print(" | Tmp (on) = "); Serial.print(st_on.Tmp); 
    Serial.print(" | AcX (on) = "); Serial.print(st_on.AcX);
    Serial.print(" | AcY (on) = "); Serial.print(st_on.AcY);
    Serial.print(" | AcZ (on) = "); Serial.println(st_on.AcZ);

    Serial.print(" STR_Xg = "); Serial.print(str.Xg);
    Serial.print(" | STR_Yg = "); Serial.print(str.Yg);
    Serial.print(" | STR_Zg = "); Serial.print(str.Zg);
    Serial.print(" | STR_Xa = "); Serial.print(str.Xa);
    Serial.print(" | STR_Ya = "); Serial.print(str.Ya);
    Serial.print(" | STR_Za = "); Serial.println(str.Za);
  
  }

  return str;
}

void evalSelfTest(FTVals STR,FTVals FT,bool Debug){
  // This function evaluates and reports the reults of
  //  the self test
  FTVals percs;
  
  percs.Xg = change(STR.Xg,FT.Xg);
  percs.Yg = change(STR.Yg,FT.Yg);
  percs.Zg = change(STR.Zg,FT.Zg);
  percs.Xa = change(STR.Xa,FT.Xa);
  percs.Ya = change(STR.Ya,FT.Ya);
  percs.Za = change(STR.Za,FT.Za);

  bool passed = true;

  if(abs(percs.Xg) > 14.0){
    Serial.println(" X axis Gyro Failed Self Test!");
    passed = false;
    };
  if(abs(percs.Yg) > 14.0){
    Serial.println(" Y axis Gyro Failed Self Test!");
    passed = false;
    };
  if(abs(percs.Zg) > 14.0){
    Serial.println(" Z axis Gyro Failed Self Test!");
    passed = false;
    };
  if(abs(percs.Xa) > 14.0){
    Serial.println(" X axis Accelerometer Failed Self Test!");
    passed = false;
    };
  if(abs(percs.Yg) > 14.0){
    Serial.println(" Y axis Accelerometer Failed Self Test!");
    passed = false;
    };
  if(abs(percs.Zg) > 14.0){
    Serial.println(" Z axis Accelerometer Failed Self Test!");
    passed = false;
    };
  
  if(passed){
    Serial.println(" ALL AXIS PASSED SELF TEST!");
    };

    
  if(Debug){
    Serial.print(" perc_Xg = "); Serial.print(percs.Xg);
    Serial.print("% | perc_Yg = "); Serial.print(percs.Yg);
    Serial.print("% | perc_Zg = "); Serial.print(percs.Zg);
    Serial.print("% | perc_Xa = "); Serial.print(percs.Xa);
    Serial.print("% | perc_Ya = "); Serial.print(percs.Ya);
    Serial.print("% | perc_Za = "); Serial.print(percs.Za); Serial.println("%");
  }
}

