#define LOGGER_MIN_SEVERITY LOGGER_SEVERITY_INFO
#include <SerialDXL.h>

// SPI and RF Communication libraries
#include <SPI.h>
#include "RF24.h"

// LED DXL basic config
#define RECEIVER_MODEL 100
#define RECEIVER_FIRMWARE 100
#define RECEIVER_MMAP_SIZE 2 // Use 1 variable


# define CE_PIN 4           // SPI connections pins
# define CSN_PIN 5          
# define LED_MESSAGE 12     // Message pin
# define LED_QOS 13         // Quality of Service pin
# define DATA_CONTROL 7     
# define RESET_PIN 10

/**
 * @brief Basic RF receiver using DXL communication protocol
 *
 * @param dir_pin Toggle communication pin.
 * @param reset_pin Pin for reset device
 */
 
class ReceiverDXL: public DeviceDXL<RECEIVER_MODEL, RECEIVER_FIRMWARE, RECEIVER_MMAP_SIZE>
{
  public:
    ReceiverDXL(uint8_t dataControlPin, uint8_t reset_pin, uint8_t led_qos, uint8_t led_message, RF24* radio):
    DeviceDXL(), // Call parent constructor
    reset_pin_(reset_pin),      // Reset pin
    led_qos_(led_qos),          // QOS Led
    radio_(radio),            
    led_message_(led_message),  
    counter_WD_(0),              // Watchdog Counter, useful in the measure of QOS
    
    status_(MMap::Access::RW, MMap::Storage::RAM), // Led command
    limit_WD_(MMap::Access::RW, MMap::Storage::EEPROM) // Upper Watchdog Counter Limit, it can be modified.
    {
      // Config pins
      pinMode(dataControlPin, OUTPUT);
      pinMode(reset_pin_, INPUT);
      pinMode(led_qos_, OUTPUT);
      pinMode(led_message_,OUTPUT);

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
      mmap_.registerVariable(&limit_WD_);
      mmap_.registerVariable(&status_);
      mmap_.init();
      
      /*
       * Load default values
       */
      DEBUG_PRINTLN("Load default");
      mmap_.load(); // Load values from EEPROM
      DEBUG_PRINT("data: ");DEBUG_PRINTLN(command_.data);
      INFO_PRINT("id: ");INFO_PRINTLN_RAW(id_.data);

      /*
       * Read sensor data
       * e.g. Use ADC, check buttons, etc.
       */

      //Initial setting to start the communication
      radio_->begin();
      radio_->setPALevel(RF24_PA_LOW);                // Working Power Level of the RF Module
      radio_->openWritingPipe(this->ADDRESSES[1]);    // Set the communications paths
      radio_->openReadingPipe(1,this->ADDRESSES[0]);  
      radio_->startListening();
      
    }



    void update()
    {
      int16_t got_voltage;   // Float variable to receive the transmitted voltage
         
      if( radio_->available() )           
      {
        digitalWrite(led_qos_, HIGH);     // If there is communnication, turn on QOS led                                                           
        while (radio_->available())   
        {
          radio_->read( &got_voltage, sizeof(int16_t) );    // Get Message               
        }

        // Turn on/Turn off Message Led Criteria
        if (got_voltage >= 3)   
        {
          digitalWrite(led_message_,LOW);
          status_.data = 1;
        }
        else
        {
          digitalWrite(led_message_,HIGH);
          status_.data = 0;
        }
        
        radio_->stopListening(); 
        radio_->write( &got_voltage, sizeof(int16_t) ); 
        radio_->startListening();
        counter_WD_ = 0;
     }


     //Serial.println(counter_WD_);
     counter_WD_ = counter_WD_+ 1;       // Increase WatchdogCounter in each iteration
     if(counter_WD_ >= 100*limit_WD_.data)     // If reaches a the limit value, turn off the QOS Led.
     {
        digitalWrite(led_qos_, LOW);
        counter_WD_ = 100*limit_WD_.data + 1;      
     }
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
    byte ADDRESSES[2][6] = {"1Node","2Node"};
    // Communication direction pin
    uint8_t dataControlPinMask_;
    volatile uint8_t *dataControlPinReg_;

    const uint8_t reset_pin_; // Reset pin
    const uint8_t led_qos_; // LED pin

    const uint8_t led_message_;  // Message Pin
    float float_raw;
    
    unsigned long counter_WD_;   // Watchdog Counter
                

    // Radio
    RF24* radio_;
    
    // LED variable
    MMap::Integer<UInt8, 0, 1, 1>::type status_;
    MMap::Integer<UInt8, 0, 100, 60>::type limit_WD_;
    
};

RF24 radio(CE_PIN, CSN_PIN);                                                      
ReceiverDXL receiver(DATA_CONTROL, RESET_PIN, LED_QOS, LED_MESSAGE, &radio);      
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
  
  receiver.mmap_.deserialize();
  receiver.update();
  receiver.mmap_.serialize();
  
}
