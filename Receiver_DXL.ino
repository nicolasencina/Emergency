#define LOGGER_MIN_SEVERITY LOGGER_SEVERITY_INFO
#include <SerialDXL.h>

// Librerías para comunicación RF
#include <SPI.h>
#include "RF24.h"

// LED DXL basic config
#define RECEIVER_MODEL 100
#define RECEIVER_FIRMWARE 100
#define RECEIVER_MMAP_SIZE 1 // Use 1 variable

int CE_pin = 4;
int CSN_pin = 5;
byte addresses[][6] = {"1Node","2Node"};

unsigned long CounterWD=0;
int limitWD=6000;

/**
 * @brief LED control using DXL communication protocol
 * @details LED control using Dynamixel communication protocol over RS485.
 * This implementation uses a 1 uint8_t variable for LED state in address 6 
 * of memory map (MMap).
 * 
 * @param dir_pin Toggle communication pin.
 * @param reset_pin Pin for reset device
 * @param led_pin LED pin.
 */
class ReceiverDXL: public DeviceDXL<RECEIVER_MODEL, RECEIVER_FIRMWARE, RECEIVER_MMAP_SIZE>
{
  public:
    ReceiverDXL(uint8_t dataControlPin, uint8_t reset_pin, uint8_t led_pin, uint8_t led_message):
    DeviceDXL(), // Call parent constructor
    reset_pin_(reset_pin),    // Reset pin
    led_pin_(led_pin),        // LED pin
    command_(MMap::Access::RW, MMap::Storage::RAM) // Led command
    {
      // Config pins
      pinMode(dataControlPin, OUTPUT);
      pinMode(reset_pin_, INPUT);
      pinMode(led_pin_, OUTPUT);
      pinMode(led_message,OUTPUT);

      // Get mask and port for data control pin
      dataControlPinMask_ = digitalPinToBitMask(dataControlPin);
      dataControlPinReg_ = portOutputRegister(digitalPinToPort(dataControlPin));
    }

    void init()
    {
      DEBUG_PRINTLN("INIT");
      /*
       * Register variables
       */
      DeviceDXL::init();
      mmap_.registerVariable(&command_);
      mmap_.init();
      
      /*
       * Load default values
       */
      DEBUG_PRINTLN("Load default");
      mmap_.load(); // Load values from EEPROM
      DEBUG_PRINT("data: ");DEBUG_PRINTLN(command_.data);
      INFO_PRINT("id: ");INFO_PRINTLN_RAW(id_.data);

      RF24 radio(CE_pin, CSN_pin);
      radio.begin();
      radio.setPALevel(RF24_PA_LOW);
      radio.openWritingPipe(addresses[1]);
      radio.openReadingPipe(1,addresses[0]);
      radio.startListening();

      /*
       * Read sensor data
       * e.g. Use ADC, check buttons, etc.
       */
    }

    void update()
    {
      if (command_.data == 1) digitalWrite(led_pin_, HIGH);
      else digitalWrite(led_pin_, LOW);
    }

    inline bool onReset()
    {
      return digitalRead(reset_pin_) == HIGH ? true : false;
    }

    inline void setTX()
    {
      *dataControlPinReg_ |= dataControlPinMask_;
    }

    inline void setRX()
    {
      *dataControlPinReg_ &= ~dataControlPinMask_;
    }



  private:
    // Communication direction pin
    uint8_t dataControlPinMask_;
    volatile uint8_t *dataControlPinReg_;

    const uint8_t reset_pin_; // Reset pin
    const uint8_t led_pin_; // LED pin
    float float_raw;
    
    // LED variable
    MMap::Integer<UInt8, 0, 1, 1>::type command_;

    
};
//LedDXL(uint8_t dataControlPin, uint8_t reset_pin, uint8_t led_pin):

ReceiverDXL receiver(7, 10, 13, 12);
SerialDXL<ReceiverDXL> serialDxl;

void setup() {
  Serial.begin(115200);
  delay(50);
  
  receiver.init();
  receiver.reset();
  receiver.mmap_.serialize();

  // Init serial communication using Dynamixel format
  serialDxl.init(&Serial3 ,&receiver);

  
  
}

void loop() {
  // Update msg buffer
  while (Serial3.available()){
    serialDxl.process(Serial3.read());
  }

    float got_voltage;   
    if( radio.available() ){
      digitalWrite(LedAlarm,HIGH);                                                                
      while (radio.available()) {                                                                 
        radio.read( &got_voltage, sizeof(float) );
        Serial.println(got_voltage);                  
      }
      if (got_voltage>=3.5)  {
        digitalWrite(LedMessage,HIGH);
      }
      else {
        digitalWrite(LedMessage,LOW);
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
  
  receiver.mmap_.deserialize();
  receiver.update();
  receiver.mmap_.serialize();
  
}
