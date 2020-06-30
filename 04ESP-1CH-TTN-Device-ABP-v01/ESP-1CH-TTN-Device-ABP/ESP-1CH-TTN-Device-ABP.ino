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

// Below sets the trigger for sending a new message to a gateway.
// Either or both can be enabled (in which case pressing the button will restart the timer)
#define SEND_BY_BUTTON 0  // Send a message when button "0" is pressed
#define SEND_BY_TIMER 1 // Send a message every TX_INTERVAL seconds

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x5B,0x9C,0x23,0x53,0x2D,0x14,0xDB,0x1D,0xA5,0x6B,0x33,0x8F,0x3B,0xF4,0x0E,0xC0 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0xFF,0x67,0xFC,0x65,0x0C,0xB4,0xE3,0x78,0xFF,0xDA,0x19,0x74,0x4A,0xB7,0x49,0xCB };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x260211B4;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Pin mapping for the SparkX ESP32 LoRa 1-CH Gateway
const lmic_pinmap lmic_pins = {
  .nss = 27,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 34,
  .dio = {18, 33, 32},
};

// If send-by-timer is enabled, define a tx interval
#ifdef SEND_BY_TIMER
#define TX_INTERVAL 60 // Message send interval in seconds
#endif

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
      digitalWrite(LED_BUILTIN, LOW); // Turn off LED after send is complete
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

// Transmit data from mydata
void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    digitalWrite(5, HIGH); // Turn on LED while sending
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    Serial.println(F("Packet queued"));
  }
}

void setup() {
  Serial.begin(115200);

  // Set button pin as input
#ifdef SEND_BY_BUTTON
  pinMode(0, INPUT);
#endif
  // Configure built-in LED -- will illuminate when sending
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

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
  //LMIC_enableChannel(17);
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job -- Transmit a message on begin
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
#ifdef SEND_BY_BUTTON
  if (digitalRead(0) == LOW) {
    while (digitalRead(0) == LOW) ;
    do_send(&sendjob);
  }
#endif
}
