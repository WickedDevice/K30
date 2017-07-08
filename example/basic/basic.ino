#include <SoftwareSerial.h>
#include <K30.h>

int enable_pin = 18; // will be 17 on 3.1 shield (bugfix)

SoftwareSerial co2Serial(9, 10);  // RX, TX
K30 k30(&co2Serial);

void setup(){
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(enable_pin, OUTPUT);
  digitalWrite(enable_pin, HIGH);

}

void loop(){
  float sample = 0.0f;
  Serial.print(millis());
  Serial.print("\t");
  if(k30.getSample(&sample)){
    Serial.println(sample, 2);
  }
  else{
    Serial.println("Failed to Get Sample");
    digitalWrite(enable_pin, LOW);
    delay(100);
    digitalWrite(enable_pin, HIGH);
    delay(100);
  }

  delay(2000);
}
