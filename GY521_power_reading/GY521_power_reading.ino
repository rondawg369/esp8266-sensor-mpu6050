ADC_MODE(ADC_VCC); 
  
const float calib = 0.00118467153;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(" ");
  }

void loop() {
  // put your main code here, to run repeatedly:
  float getVcc = ESP.getVcc() * calib;
  Serial.println(getVcc,4);
  delay(1000);
}
