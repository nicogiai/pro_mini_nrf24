/*
 * See documentation at https://nRF24.github.io/RF24
 * See License information at root directory of this library
 * Author: Brendan Doherty (2bndy5)
 */


/**
 * Arduino Pro Mini Low power Sleep Example
 * https://gist.github.com/boseji/d4e031aa7ec14b498a7a6a1efedf6d55
 */
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

/**
 * A simple example of sending data from 1 nRF24L01 transceiver to another.
 *
 * This example was written to be used on 2 devices acting as "nodes".
 * Use the Serial Monitor to change each node's behavior.
 */
#include <SPI.h>
#include "printf.h"
#include "RF24.h"

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor
#include "DHT.h"

#define DHTPIN 2     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 3 (on the right) of the sensor to GROUND (if your sensor has 3 pins)
// Connect pin 4 (on the right) of the sensor to GROUND and leave the pin 3 EMPTY (if your sensor has 4 pins)
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);

// instantiate an object for the nRF24L01 transceiver
RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin

// Let these addresses be used for the pair
uint8_t address[][6] = {"1Node", "2Node"};
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 0; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
bool role = true;  // true = TX role, false = RX role

// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
float payload[2] = {0.0,0.0};

// WDT entry Flag
volatile uint8_t f_wdt=1;


//////////////////////////////////////////////////////////////////////////
// WDT Interrupt

ISR(WDT_vect)
{
  if(f_wdt == 0)
  {
    f_wdt=1; // Reset the Flag
  }
  else
  {
    Serial.println(F("WDT Overrun!!!"));
  }
}

//////////////////////////////////////////////////////////////////////////
// Sleep Configuration Function
//   Also wake-up after

void enterSleep(void)
{
  WDTCSR |= _BV(WDIE); // Enable the WatchDog before we initiate sleep

  //set_sleep_mode(SLEEP_MODE_PWR_SAVE);    /* Some power Saving */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);    /* Even more Power Savings */
  sleep_enable();

  /* Now enter sleep mode. */
  sleep_mode();
  sleep_bod_disable();  // Additionally disable the Brown out detector

  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */

  /* Re-enable the peripherals. */
  power_all_enable();
}

/*
//(0b100001);  // 8 seconds
//(0b100000);  // 4 seconds
void myWatchdogEnable(const byte interval) 
  {  
  MCUSR = 0;                          // reset various flags
  WDTCSR |= 0b00011000;               // see docs, set WDCE, WDE
  WDTCSR =  0b01000000 | interval;    // set WDIE, and appropriate delay

  wdt_reset();
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  sleep_mode();            // now goes to Sleep and waits for the interrupt
  } 
*/
void setup() {

  /*** Setup the WDT ***/
  cli();
  /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);

  /* In order to change WDE or the prescaler, we need to
  * set WDCE (This will allow updates for 4 clock cycles).
  */
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  /* set new watchdog timeout prescaler value */
  //WDTCSR = 1<<WDP1 | 1<<WDP2;             /* 1.0 seconds */
  //WDTCSR = 1<<WDP0 | 1<<WDP1 | 1<<WDP2; /* 2.0 seconds */
  //WDTCSR = 1<<WDP3;                     /* 4.0 seconds */
  WDTCSR = 1<<WDP0 | 1<<WDP3;           /* 8.0 seconds */

  /* Enable the WD interrupt (note no reset). */
  //WDTCSR |= _BV(WDIE); // Not here but when we go to Sleep
  sei();
  
  Serial.begin(9600);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }

  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {} // hold in infinite loop
  }

  // print example's introductory prompt
  Serial.println(F("RF24/examples/GettingStarted"));

  // To set the radioNumber via the Serial monitor on startup
  //Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
  //while (!Serial.available()) {
    // wait for user input
  //}
  //char input = Serial.parseInt();
  //radioNumber = input == 1;
  Serial.print(F("radioNumber = "));
  Serial.println((int)radioNumber);

  // role variable is hardcoded to RX behavior, inform the user of this
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);     // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1

  // additional setup specific to the node's role
  if (role) {
    radio.stopListening();  // put radio in TX mode
  } else {
    radio.startListening(); // put radio in RX mode
  }

  // For debugging info
  // printf_begin();             // needed only once for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  // radio.printPrettyDetails(); // (larger) function that prints human readable data

  dht.begin();

  pinMode(9, OUTPUT);
  digitalWrite(9,LOW);
  
} // setup

void loop() {

  // Only Execute this part One time
  if(f_wdt == 1) {

    //////////////////////////
    // PROCESSING BEGIN

    digitalWrite(9,LOW);        // LED Indication ON
  
    // Wait a few seconds between measurements.
    //delay(2000);
  
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    payload[1] = dht.readHumidity();
    // Read temperature as Celsius (the default)
    payload[0] = dht.readTemperature();
    
    if (role) {
      // This device is a TX node
  
      unsigned long start_timer = micros();                    // start the timer
      bool report = radio.write(&payload[0], sizeof(payload));      // transmit & save the report
      unsigned long end_timer = micros();                      // end the timer
  
      if (report) {
        Serial.print(F("Transmission successful! "));          // payload was delivered
        Serial.print(F("Time to transmit = "));
        Serial.print(end_timer - start_timer);                 // print the timer result
        Serial.print(F(" us. Sent: "));
        Serial.println(payload[0]);                               // print payload sent
        Serial.println(payload[1]);                               // print payload sent
      } else {
        Serial.println(F("Transmission failed or timed out")); // payload was not delivered
      }
  
      // to make this example readable in the serial monitor
      //delay(1000);  // slow transmissions down by 1 second
  
    } else {
      // This device is a RX node
  
      uint8_t pipe;
      if (radio.available(&pipe)) {             // is there a payload? get the pipe number that recieved it
        uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
        radio.read(&payload[0], bytes);            // fetch payload from FIFO
        Serial.print(F("Received "));
        Serial.print(bytes);                    // print the size of the payload
        Serial.print(F(" bytes on pipe "));
        Serial.print(pipe);                     // print the pipe number
        Serial.print(F(": "));
        Serial.println(payload[0]);                // print the payload's value
      }
    } // role
  
    if (Serial.available()) {
      // change the role via the serial monitor
  
      char c = toupper(Serial.read());
      if (c == 'T' && !role) {
        // Become the TX node
  
        role = true;
        Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
        radio.stopListening();
  
      } else if (c == 'R' && role) {
        // Become the RX node
  
        role = false;
        Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));
        radio.startListening();
      }
    }

    delay(100);  // give time to serial
    digitalWrite(9,HIGH);        // LED Indication OFF
    
    // PROCESSING END
    //////////////////////////

    /* Don't forget to clear the flag. */
    f_wdt = 0;

    /* Re-enter sleep mode. */
    enterSleep();
    
  } //if(f_wdt == 1)
  
} // loop
