/*
  Single-Channel TTN ABP Device
  By: Jim Lindblom
  SparkFun Electronics
  Date: July 31, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
  Feel like supporting our work? Buy a board from SparkFun!

  https://sparkle.sparkfun.com/sparkle/storefront_products/14893

  This example demonstrates how to use the SparkX ESP32 LoRa 1-CH Gateway as a LoRaWAN device.
  It is configured to transmit on a single channel -- making it compatible with the same board
  set up as a gateway.

  Library Dependencies:
  arduino-lmic: https://github.com/mcci-catena/arduino-lmic

  To use the example, ensure that you've first configured the arduino-lmic library as required.
  Then create an application and device on the Things Network. Configure it for ABP activation.
  Then copy the NETWORK SESSION KEY, APP SESSION KEY, and DEVICE ADDRESS into the global variables
  towards the top of this sketch.

  Once uploaded, press the "0" button to trigger a LoRa send. The device will send "Hello, World"
  on the set frequency, hoping your gateway hears it.

  This example is based on ttn-abp by: Thomas Telkamp and Matthijs Kooijman
*/


// Include the arduino-lmic library:
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

#include <TinyGPS++.h>



// Either or both can be enabled (in which case pressing the button will restart the timer)
#define SEND_BY_BUTTON 1  // Send a message when button "0" is pressed
//#define SEND_BY_TIMER 1 // Send a message every TX_INTERVAL seconds



// LoRaWAN NwSKEY, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0xFF,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5};

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0xFF,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5,0xA5};

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x01234567;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


float temperature;
float presion;
float latitud;
float longitud;
float altura;

static osjob_t sendjob;

// Pin mapping for the SparkX ESP32 LoRa 1-CH Gateway
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN,
  .dio = {26, 33, 32},
};

// If send-by-timer is enabled, define a tx interval
#ifdef SEND_BY_TIMER
#define TX_INTERVAL 60 // Message send interval in seconds
#endif


CayenneLPP lpp(51);
TinyGPSPlus gps;

// State machine event handler
void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      digitalWrite(5, HIGH); // Turn off LED after send is complete
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
#ifdef SEND_BY_TIMER
      // Schedule the next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
#endif
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

void displaySensorDetails(void)
{
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


void do_send(osjob_t* j) 
{

  if (LMIC.opmode & OP_TXRXPEND)   // Check if there is not a current TX/RX job running
    {
      Serial.println(F("OP_TXRXPEND, not sending"));
    } 
  else 
    {
      digitalWrite(5, LOW); 


      LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0); // Prepare upstream data transmission at the next possible time.
      
      Serial.println(F("Packet queued"));
    }
}



void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

#ifdef SEND_BY_BUTTON
  pinMode(0, INPUT);
#endif

  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);

  os_init(); // LMIC init 
  
  LMIC_reset(); // Reset the MAC state. Session and pending data transfers will be discarded.

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

#if defined(CFG_eu868) // EU channel setup
  // Set up the channel used by the Things Network and compatible with
  // our gateway.
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
#elif defined(CFG_us915) // US channel setup
  // Instead of using selectSubBand, which will cycle through a sub-band of 8
  // channels. We'll configure the device to only use one frequency.
// First disable all sub-bands
for (int b = 0; b < 8; ++b) {
  LMIC_disableSubBand(b);

  
}
// Then enable the channel(s) you want to use
LMIC_enableChannel(8); // 903.9 MHz

#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job -- Transmit a message on begin
  do_send(&sendjob);


  Serial.println("Pressure Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
}

void loop() {
  os_runloop_once();
  
#ifdef SEND_BY_BUTTON
  if (digitalRead(0) == LOW) 
    {
      while (digitalRead(0) == LOW) ;{}//antirebote}
      
      sensors_event_t event;
      bmp.getEvent(&event);

      if (event.pressure)
        {
          bmp.getTemperature(&temperature);
          Serial.print("Temperature: ");
          Serial.print(temperature);
          Serial.println(" C");

          presion = event.pressure;
          Serial.print("Pressure:    ");
          Serial.print(presion);
          Serial.println(" hPa");
        }       
      else
        {
          Serial.println("Sensor error");
        }
        
    smartDelay(1000);
    if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

    latitud = gps.location.lat();
    longitud = gps.location.lng();
    altura = gps.altitude.meters();
    
    Serial.println(latitud,5);
    Serial.println(longitud,5);
    Serial.println(altura);

      lpp.reset();    
      lpp.addGPS(1, latitud, longitud, altura);
      lpp.addTemperature(2, temperature);
      lpp.addBarometricPressure(3,presion);

      do_send(&sendjob);
  }
#endif
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial2.available())
      gps.encode(Serial2.read());
  } while (millis() - start < ms);
}
