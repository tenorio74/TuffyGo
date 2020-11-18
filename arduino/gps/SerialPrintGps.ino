
// This Program will output raw data from the GPS into the serial screen
#include <SoftwareSerial.h>
 
//Create software serial object to communicate with GPS 
SoftwareSerial gps(A1, A0);
 
void setup() {
  //Begin serial comunication with Arduino and Arduino IDE (Serial Monitor)
  Serial.begin(9600);
  while(!Serial);
  Serial.print("Testing TinyGPS library v. "); Serial.println(TinyGPS::library_version());
   
  //Being serial communication witj Arduino and GPS Module //Important rate must be 9600
  gps.begin(9600);
  delay(1000);
 
  Serial.println("Setup Complete!");
}
 
void loop() {
  //Read SIM800 output (if available) and print it in Arduino IDE Serial Monitor
  if(gps.available()){
    Serial.write(gps.read());
  }
  //Read Arduino IDE Serial Monitor inputs (if available) and send them to SIM800
  if(Serial.available()){    
    gps.write(Serial.read());
  }
}
