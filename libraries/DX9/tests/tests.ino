#include <AP_DSM.h>
#include <DueTimer.h>
AP_DSM      AP_dsm;

void call_method_pointer () {
     AP_DSM *ptr= &AP_dsm;
    (ptr->decode_stream)();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  AP_dsm.initialise(&Serial1);

  //Timer8.attachInterrupt(call_method_pointer);
  //Timer8.start(11000);
}

uint32_t msec = 0;
void loop() {

while(Serial1.available()){
  Serial.println(Serial1.read());
}Serial.println("*** FULL ***");
  
  // put your main code here, to run repeatedly:
  // AP_dsm.decode_stream();
  if(millis() - msec >= 20){
    Serial.print(millis() - msec);
    Serial.print(F("   "));
    Serial.print(AP_dsm.values[0]);
    Serial.print(F("   "));
    Serial.print(AP_dsm.values[1]);
    Serial.print(F("   "));
    Serial.print(AP_dsm.values[2]);
    Serial.print(F("   "));
    Serial.print(AP_dsm.values[3]);
    Serial.print(F("   "));
    Serial.print(AP_dsm.values[4]);
    Serial.print(F("   "));
    Serial.print(AP_dsm.values[5]);
    Serial.print(F("   "));
    Serial.print(AP_dsm.values[6]);
    Serial.print(F("   "));
    Serial.print(AP_dsm.values[7]);
    Serial.print(F("   "));
    Serial.print(AP_dsm.values[8]);
    Serial.print(F("   "));
    Serial.println(AP_dsm.DT, 4);
    msec = millis(); 
  }
}
