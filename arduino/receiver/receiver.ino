#include <SPI.h>
#include "RF24.h"
int LedMessage=6;
int LedAlarm=5;
unsigned long CounterWD=0;
int limitWD=6000;
/****************** User Config ***************************/
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(7,8);

byte addresses[][6] = {"1Node","2Node"};

void setup() {
  Serial.begin(115200);
  pinMode(LedMessage,OUTPUT);
  pinMode(LedAlarm,OUTPUT);
  radio.begin();
  
  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);

  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1,addresses[0]);
  
  // Start the radio listening for data
  radio.startListening();
}

void loop() {
    float got_voltage;   
    if( radio.available() ){
      digitalWrite(LedAlarm,HIGH);                                                                
      while (radio.available()) {                                                                 
        radio.read( &got_voltage, sizeof(float) );
        Serial.println(got_voltage);                  
      }
      if (got_voltage>=3.5)  {
        digitalWrite(LedMessage,HIGH);
        Serial.println("QueSUCEDE");
      }
      else {
        digitalWrite(LedMessage,LOW);
        Serial.println("NADASUCEDE");
      }
      radio.stopListening();                                        // First, stop listening so we can talk   
      radio.write( &got_voltage, sizeof(char) );              // Send the final one back.      
      radio.startListening(); 
      
      CounterWD=0;  
   }
   CounterWD=CounterWD+1;
   if(CounterWD>=limitWD){
      digitalWrite(LedAlarm,LOW);
      CounterWD=limitWD+1;
   }
} // Loop

