#include <MPU6050.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiConfig.h> // WiFi and MQTT Configuration Info

ADC_MODE(ADC_VCC); 

const uint8_t MPU_addr=0x68;  // I2C address of the MPU-6050

const float calibVCC = 0.00118467153;  // Calibration based on measurement

MPU6050 testUnit(MPU_addr);

const byte interruptPin = D7;
const uint8_t threshold = 20;  // Adjust to change sensitivity to acceleration
const uint8_t duration = 10;   // Adjust to change duration of motion to trigger interrupt

uint8_t MAC_array[6];
char MAC_char[18];

WiFiClient espClient;
PubSubClient client(espClient);

char msg[50];



void setup() {
  Wire.begin();
  disableMotionInt();
  //Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);

  //Serial.println("I'm Awake!");
  //Serial.print("Motion detect Status : "); Serial.println(testUnit.getMotionStatus(),BIN);
  
  float getVcc = ESP.getVcc() * calibVCC;
  //Serial.print("Supply Voltage : ");Serial.print(getVcc,4);Serial.println(" V ");

  if (!client.connected()) {
     reconnect();
  }
  client.loop();

  strcpy(msg,"Motion Detected!! " );
  //Serial.print("Publish message: ");
  //Serial.println(msg);
  client.publish("FrontSensor/Motion", msg);
  
 
  strcpy(msg," " );
  dtostrf(getVcc,2,2,&msg[strlen(msg)]);
  //Serial.print("Publish message: ");
  //Serial.println(msg);
  client.publish("FrontSensor/Vcc", msg);
  

  
  //Serial.println("Going to sleep. . .");
  enableMotionInt();
  ESP.deepSleep(0);
}

void loop() {
  
}

void setup_wifi() {
 
  delay(10);
  //Serial.println();
  //Serial.print("Connecting to ");
  //Serial.println(ssid);
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //Serial.print(".");
  }
 
  //Serial.println("");
  //Serial.println("WiFi connected");
  //Serial.println("IP address: ");
  //Serial.println(WiFi.localIP());
}


void disableMotionInt(){
  // Ensure the accelerometer is running
  testUnit.setSleepEnabled(false);
  testUnit.setWakeCycleEnabled(false);
  testUnit.setStandbyXAccelEnabled(false);
  testUnit.setStandbyYAccelEnabled(false);
  testUnit.setStandbyZAccelEnabled(false);

  }

  
void enableMotionInt(){
  testUnit.initialize();
  testUnit.testConnection();

  // Ensure the accelerometer is running
  testUnit.setSleepEnabled(false);
  testUnit.setWakeCycleEnabled(false);
  testUnit.setStandbyXAccelEnabled(false);
  testUnit.setStandbyYAccelEnabled(false);
  testUnit.setStandbyZAccelEnabled(false);

  //Set the accelerometer HPF to reset settings
  testUnit.setDHPFMode(MPU6050_DHPF_RESET);

  //Set the accelerometer LPF to 256Hz Bandwidth
  testUnit.setDLPFMode(MPU6050_DLPF_BW_256);

  //Enable the motion interrupt
  testUnit.setIntEnabled(0b00000000);
  testUnit.setIntMotionEnabled(true);

  //Set the motion detection duration
  testUnit.setMotionDetectionDuration(duration); //Duration in ms

  //Set the motion detection threshold
  testUnit.setMotionDetectionThreshold(threshold); // Threshold in 2mg

  //1 ms delay
  delay(1);

  //Set the accelerometer HPF to HOLD settings
  testUnit.setDHPFMode(MPU6050_DHPF_HOLD);

  // Set the wakeup frequency
  testUnit.setWakeFrequency(MPU6050_WAKE_FREQ_5);
  testUnit.setStandbyXGyroEnabled(true);
  testUnit.setStandbyYGyroEnabled(true);
  testUnit.setStandbyZGyroEnabled(true);

  // Enable cycle mode
  testUnit.setWakeCycleEnabled(true);

  testUnit.setInterruptLatch(0);
  testUnit.setInterruptLatchClear(0);
  testUnit.setInterruptDrive(0);
  testUnit.setInterruptMode(1);  
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    //Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientID, mqtt_username, mqtt_password)) {
      //Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("FrontSensor/alive", "I'm Alive!");
    } else {
      //Serial.print("failed, rc=");
      //Serial.print(client.state());
      //Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

