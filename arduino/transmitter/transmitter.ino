#include <SPI.h>
#include "RF24.h"

/****************** User Config ***************************/
/* Pines para CE y CSN respectivamente a los pines 3 y 4 */
RF24 radio(3, 4);
int LedPin1 = 2;        //LED se encenderá cuando exista comunicación
int botonAnalog = 0;    //Se utiliza para medir un valor de voltaje a enviar
/**********************************************************/
byte addresses[][6] = {"1Node", "2Node"};

void setup() {
  Serial.begin(115200);
  pinMode(LedPin1, OUTPUT);

  radio.begin();
  radio.setPALevel(RF24_PA_LOW);

  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  
  // Start the radio listening for data
  radio.startListening();
}

void loop() {
  radio.stopListening();                                    // First, stop listening so we can talk.
  int16_t enviar = analogRead(botonAnalog);                            

  boolean intento=radio.write( &enviar, sizeof(int16_t) );

  if (intento){
    digitalWrite(LedPin1, LOW);
  } else{
    digitalWrite(LedPin1, HIGH);
  }

  
  // Try again 1s later
  delay(50);
  
} 

