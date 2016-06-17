#include <SPI.h>
#include "RF24.h"

/****************** User Config ***************************/
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(7, 8);
int LedPin1 = 4;
int botonAnalog = 0;
/**********************************************************/
byte addresses[][6] = {"1Node", "2Node"};

void setup() {
  Serial.begin(115200);
  Serial.println(F("RF24/examples/GettingStarted"));
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));
  pinMode(LedPin1, OUTPUT);
  radio.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);

  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  // Start the radio listening for data
  radio.startListening();
}

void loop() {
  radio.stopListening();                                    // First, stop listening so we can talk.
  float enviar = (5.0 / 1023) * analogRead(botonAnalog);

  //    Serial.println(F("Now sending"));

  unsigned long start_time = micros();                             // Take the time, and send it.  This will block until complete
  if (!radio.write( &enviar, sizeof(float) )) {
    Serial.println(F("failed"));
    digitalWrite(LedPin1, LOW);
  }
  radio.startListening();                                    // Now, continue listening
  unsigned long started_waiting_at = micros();               // Set up a timeout period, get the current microseconds
  boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not
  while ( ! radio.available() ) {                            // While nothing is received
    if (micros() - started_waiting_at > 2000000 ) {            // If waited longer than 200ms, indicate timeout and exit while loop
      timeout = true;
      break;
    }
  }

  if ( timeout ) {
    digitalWrite(LedPin1, LOW); // Describe the results
    Serial.println(F("Failed, response timed out."));
  } else {
    unsigned long got_time;                                 // Grab the response, compare, and send to debugging spew
    radio.read( &got_time, sizeof(unsigned long) );
    //       unsigned long end_time = micros();
    digitalWrite(LedPin1, HIGH);
  }
  // Try again 1s later
  delay(50);

} // Loop

