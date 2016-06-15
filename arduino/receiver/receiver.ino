
/*
* Getting Started example sketch for nRF24L01+ radios
* This is a very basic example of how to send data from one node to another
* Updated: Dec 2014 by TMRh20
*/

#include <SPI.h>
#include "RF24.h"
int PinLed1=5 ;
int PinLed2=6;
unsigned long Counter=0;
/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 1;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(7,8);

/**********************************************************/

byte addresses[][6] = {"1Node","2Node"};

// Used to control whether this node is sending or receiving
bool role = 0;

void setup() {
  Serial.begin(115200);
  Serial.println(F("RF24/examples/GettingStarted"));
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));
  pinMode(PinLed1,OUTPUT);
  pinMode(PinLed2,OUTPUT);
  radio.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  
  // Open a writing and reading pipe on each radio, with opposite addresses
  if(radioNumber){
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1,addresses[0]);
  }else{
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }
  
  // Start the radio listening for data
  radio.startListening();
}

void loop() {
  
  




/****************** Pong Back Role ***************************/

  if ( role == 0 )
  {
    float got_voltage;
    
    if( radio.available()){

      digitalWrite(PinLed2,LOW);
                                                                        // Variable for the received timestamp
      while (radio.available()) {                                                                  // While there is data ready
        radio.read( &got_voltage, sizeof(float) );
        Serial.println(got_voltage);// Get the payload
      }
      if (got_voltage>=3.5)  {
        digitalWrite(PinLed1,HIGH);
      }
      else {
        digitalWrite(PinLed1,LOW);
      }
      radio.stopListening();                                        // First, stop listening so we can talk   
      radio.write( &got_voltage, sizeof(char) );              // Send the final one back.      
      radio.startListening();                                       // Now, resume listening so we catch the next packets.     
      Serial.print(F("Sent response "));
      Serial.println(got_voltage);
      Counter=0;  
   }
   Counter=Counter+1;
   if(Counter>=2300){
      digitalWrite(PinLed2,HIGH);
   }
   }

   
 

 //Serial.println(radio.available());




/****************** Change Roles via Serial Commands ***************************/

  if ( Serial.available() )
  {
    char c = toupper(Serial.read());
    if ( c == 'T' && role == 0 ){      
      Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
      role = 1;                  // Become the primary transmitter (ping out)
    
   }else
    if ( c == 'R' && role == 1 ){
      Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));      
       role = 0;                // Become the primary receiver (pong back)
       radio.startListening();
       
    }
  }


} // Loop

