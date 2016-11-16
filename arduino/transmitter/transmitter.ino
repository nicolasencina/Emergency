#include <SPI.h>
#include "RF24.h"

/****************** User Config ***************************/
/* Pines para CE y CSN respectivamente a los pines 3 y 4 */

#define STATE_LED 2             //LED se encenderá cuando no exista comunicación
#define INTERRUPTOR_PIN 0       //Se utiliza para medir un valor de voltaje a enviar
#define CE_PIN 3
#define CSN_PIN 4

RF24 radio(CE_PIN, CSN_PIN);
byte addresses[][6] = {"1Node", "2Node"};

void setup() {
  Serial.begin(115200);
  pinMode(STATE_LED, OUTPUT);

  radio.begin();
  radio.setPALevel(RF24_PA_LOW);

  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  
  // Start the radio listening for data
  radio.stopListening();
}

void loop() {
  //radio.stopListening();                                    // First, stop listening so we can talk.
  int16_t mensaje = analogRead( INTERRUPTOR_PIN );                            

  boolean enviar = radio.write( &mensaje, sizeof(int16_t) );
  Serial.println(enviar);

  if (enviar){  
    digitalWrite(STATE_LED, LOW);
  }
  else {
    digitalWrite(STATE_LED, HIGH);
  }
  
  // Try again 1s later
  delay(1000);
  
} 

