/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 * Modified By DenniZr - First test version, did not test it on the hardware yet!!!
 *******************************************************************************/

 // TODO: investigate OTAA, good description here: Moteino, LMIC and OTAA Walkthrough https://github.com/lukastheiler/ttn_moteino
 // TODO: add flightmode. Good test script seems available here:  https://ukhas.org.uk/guides:ublox6


#define DEBUG 

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SoftwareSerial.h>  
  //https://www.pjrc.com/teensy/td_libs_TinyGPS.html explains to use NewSoftSerial however https://www.arduino.cc/en/Reference/softwareSerial explains that newsoftserial is part of SoftwareSerial in arduino version 1.0 and up
#include <TinyGPS.h>
#include "keys.h"
// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

//uint8_t mydata[9];   // mydata[9] allows you to read and write to mydata[0] .. mydata[8]. Higher numbers work but are invalid.
uint8_t mydata[20];  // a few bytes added to the memory buffer to play with
const unsigned message_size = 9;  // 9 bytes are needed into the ttn tracker service
//const unsigned message_size =11; //sending too large message makes the ttntracker ignore it, allowing us to see payload at ttn console

static osjob_t sendjob;

// Schedule TX event every this many seconds (might become longer due to duty cycle limitations).
//const unsigned TX_INTERVAL = 60;    // actually an additional bit is added for the duration of the radio processing
const unsigned TX_INTERVAL = 30;    // actually an additional bit is added for the duration of the radio processing

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4},
};


TinyGPS gps;
SoftwareSerial ss(8, 9);  // originally port 8, 9
//SoftwareSerial ss(9, 8);    // or try if wires have been reversed, can be tested by reviewing output in serial/debug window

// event gets hooked into the system
void onEvent (ev_t ev) {
    Serial.println("\n\nonEvent was called    ************************** ");
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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

// do_send call is scheduled in event handler
void do_send(osjob_t* j){  // same as https://github.com/tijnonlijn/RFM-node/blob/master/template%20ttnmapper%20node%20-%20scheduling%20removed.ino
    
    Serial.println("\ndo_send was called **********************************");
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        
        Serial.print("  mydata[] = ");
        Serial.print( mydata[0], HEX );
        Serial.print(" ");
        Serial.print( mydata[1], HEX );
        Serial.print(" ");
        Serial.print( mydata[2], HEX );
        Serial.print(" ");
        Serial.print( mydata[3], HEX );
        Serial.print(" ");
        Serial.print( mydata[4], HEX );
        Serial.print(" ");
        Serial.print( mydata[5], HEX );
        Serial.print(" ");
        Serial.print( mydata[6], HEX );
        Serial.print(" ");
        Serial.print( mydata[7], HEX );
        Serial.print(" ");
        Serial.print( mydata[8], HEX );
        Serial.print(" / ");
        Serial.print( mydata[9], HEX );
        Serial.print(" ");
        Serial.print( mydata[10], HEX );
        Serial.print("  sizeof(mydata) = ");
        Serial.print(sizeof(mydata));
        Serial.print("  fixed message_size = ");
        Serial.println(message_size);
    
        // LMIC_setTxData2(1, mydata, sizeof(mydata), 0);  //Dennis adjusted sizeof(mydata)-1 to sizeof(mydata); that is common with null terminated string as the terminating char(0) is not added to the message
        LMIC_setTxData2(1, mydata, message_size, 0);   //sending too large message makes the ttntracker ignore it, allowing us to see payload at ttn console
            // LMIC_setTxData2 (u1_t port, xref2u1_t data, u1_t dlen, u1_t confirmed)
            //  Prepare upstream data transmission at the next possible time. 
            //    parameter 1 = port
            //    parameter 2 = mydata, actually this is passed as a pointer (aka reference) to a byte array (aka memory buffer)
            //    parameter 3 = the world agrees this needs to be: sizeof(mydata)-1
            //    parameter 4 = Fourth parameter enables confirmed messages if value is 1, then read explanation at https://www.thethingsnetwork.org/forum/t/acknowledgements-downlink-ack-rfm95w-solved/1944
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);   // whether 9600 or 115200; the gps feed shows repeated char and cannot be interpreted, setting high value to release system time
    
    Serial.println();
    Serial.println();
    Serial.println("Starting Balloon RFM95");
    Serial.print("Simple TinyGPS library v. "); Serial.println(TinyGPS::library_version());
    Serial.println("by Mikal Hart");
    Serial.println();
  
    // GPS serial
    ss.begin(9600);         // software serial with GPS module. Reviews tell us software serial is not best choice; 
                            // https://www.pjrc.com/teensy/td_libs_TinyGPS.html explains to use UART Serial or NewSoftSerial 

    #ifdef VCC_ENABLE
      // For Pinoccio Scout boards
      pinMode(VCC_ENABLE, OUTPUT);
      digitalWrite(VCC_ENABLE, HIGH);
      delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    bool newData = false;
    unsigned long chars = 0;
    unsigned short sentences = 0, failed = 0;
   // static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
    
    uint32_t LatitudeBinary, LongitudeBinary;
    uint16_t altitudeGps;
    uint8_t accuracy; 
    int hdopNumber;  //100ths meter
    
    float flat, flon, alt;
    unsigned long age;  // 1000ths second
    unsigned long date, time;
    uint32_t sat;
    
    //delay(1000);    // do not delay - why delay and miss out some GPS data
    //led_on();    
  
    Serial.println();
    Serial.println("Read GPS data... ");
    char c;
    unsigned long start;
    start = millis();
    led_on(); 
    do 
    {   
      while (ss.available())
      {
        char c = ss.read();
        #ifdef DEBUG
        //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
        #endif
        if (gps.encode(c)) // Did a new valid sentence come in?
          newData = true;
      }
    } while (millis() - start < 1000);  
     
    start = millis();
    led_off(); 
    do 
    {   
      while (ss.available())
      {
        char c = ss.read();
        #ifdef DEBUG
        //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
        #endif
        if (gps.encode(c)) // Did a new valid sentence come in?
          newData = true;
      }
    } while (millis() - start < 3000);    
    
    // retrieve values
    gps.f_get_position(&flat, &flon, &age);  // lat -90.0 .. 90.0 as a 4 byte float, lon -180 .. 180 as a 4 byte float, age in seconds as a 4 byte unsigned long
    gps.get_datetime(&date, &time);   // // time in hhmmsscc, date in ddmmyy
    alt = gps.f_altitude();    // signed
    hdopNumber = gps.hdop();   //
    gps.stats(&chars, &sentences, &failed);
    sat = gps.satellites();
    
    LatitudeBinary = ((flat + 90) / 180) * 16777215;
    LongitudeBinary = ((flon + 180) / 360) * 16777215;
    
    mydata[0] = ( LatitudeBinary >> 16 ) & 0xFF;
    mydata[1] = ( LatitudeBinary >> 8 ) & 0xFF;
    mydata[2] = LatitudeBinary & 0xFF;
  
    mydata[3] = ( LongitudeBinary >> 16 ) & 0xFF;
    mydata[4] = ( LongitudeBinary >> 8 ) & 0xFF;
    mydata[5] = LongitudeBinary & 0xFF;
  
    altitudeGps = alt;
    mydata[6] = ( altitudeGps >> 8 ) & 0xFF;
    mydata[7] = altitudeGps & 0xFF;
  
    accuracy = hdopNumber*10;
    mydata[8] = accuracy & 0xFF;
    
    mydata[9] = 42;  // the * character as dummy
    mydata[10] = 42;  // the * character
    
    #ifdef DEBUG
    //show me something
    Serial.println();
    Serial.println();
    Serial.println("Interpreted GPS data:");
    //    long l_lat, l_lon;
    //    gps.get_position(&l_lat, &l_lon, &age);
    //    Serial.print("ALternative reading long LAT, LON=");
    //    Serial.print(l_lat, 6);         
    //    Serial.print(", ");
    //    Serial.println(l_lon, 6);         // 51 20033410, 2 45320553 hey --- different from my current pos

    Serial.print("date, time = ");
    Serial.print( date);   
    Serial.print(", ");
    Serial.println(time);   
    Serial.print("LAT, LON=");
    Serial.print( flat, 6);   
    Serial.print(", ");
    Serial.print(flon, 6); // 52.632656, 4.738389
    Serial.print(" hdopNumber=");
    Serial.print( hdopNumber);
    Serial.print(" alt=");
    Serial.print( alt );
    Serial.print(" SAT=");
    Serial.print( sat);
    
    Serial.print(" CHARS=");
    Serial.print(chars);
    Serial.print(" SENTENCES=");
    Serial.print(sentences);
    Serial.print(" AGE=");
    Serial.print(age);
    if (age == TinyGPS::GPS_INVALID_AGE)
      Serial.print(" (Age is invalid. No GPS fix detected)");
    Serial.print(" CSUM ERR=");
    Serial.println(failed);
    if (chars == 0)
      Serial.println("** No characters received from GPS: check wiring **");
    else if (age > 5000)
      Serial.println("Warning: possible stale GPS data (age over 5 seconds)");
    else
      Serial.println("GPS Data is fresh (age less than 5 seconds)");
  
    Serial.print("For TTN message LatitudeBinary, LongitudeBinary, altitudeGps, accuracy: ");
    Serial.print( LatitudeBinary, HEX);
    Serial.print(", ");
    Serial.print( LongitudeBinary, HEX );
    Serial.print(", ");
    Serial.print( altitudeGps, HEX );
    Serial.print(", ");
    Serial.println( accuracy, HEX );
    Serial.println("expected   CA DA F. 83 5E 9. 0 .. .. " );     
    Serial.print(  "mydata[] = ");
    Serial.print( mydata[0], HEX );
    Serial.print(" ");
    Serial.print( mydata[1], HEX );
    Serial.print(" ");
    Serial.print( mydata[2], HEX );
    Serial.print(" ");
    Serial.print( mydata[3], HEX );
    Serial.print(" ");
    Serial.print( mydata[4], HEX );
    Serial.print(" ");
    Serial.print( mydata[5], HEX );
    Serial.print(" ");
    Serial.print( mydata[6], HEX );
    Serial.print(" ");
    Serial.print( mydata[7], HEX );
    Serial.print(" ");
    Serial.print( mydata[8], HEX );
    Serial.print(" / ");
    Serial.print( mydata[9], HEX );
    Serial.print(" ");
    Serial.println( mydata[10], HEX );
    // if no fix was found:
    //    LAT, LON=1000.000000, 1000.000000 hdopNumber=-1 alt=1000000.00 SAT=255 CHARS=7830 SENTENCES=0 AGE=4294967295 (Age is invalid. No GPS fix detected) CSUM ERR=2
    //    Warning: possible stale GPS data (age over 5 seconds)
    //    For TTN message LatitudeBinary, LongitudeBinary, altitudeGps, accuracy: 60E38D8, 3471C6C, 4240, F6
    //    mydata[] = E 38 D8 47 1C 6C 42 40 F6

    #endif
    
    os_runloop_once();
    led_off();
    //delay(1000);
}

void led_on()
{
  digitalWrite(13, 1);
}

void led_off()
{
  digitalWrite(13, 0);
}


//     The example SodaqBallonTestPaulB by Paul was suggested to provide the valid binary format for TTNmapper.org 
//     in this example, the Sodaq_UBlox_GPS library is used
//     as we experience difficulty in formatting the correct values into our message, let's analyze what is done in detail in the example
//     
//          geographic Latitude measurements range from -90° to +90°, positive is nothern hemisphere
//          geographic Longitude measurements range from -180° to +180°, positive indicates east from Greenwich, England
//          >> my current position = 52.632656, 4.738389
//
//          these Sodaq_UBlox_GPS reads from GPS, then DegMi converted to DecDeg; then stored as double
//                  ((sodaq_gps.getLat() + 90) / 180) * 16777215
//                  ((sodaq_gps.getLon() + 180) / 360) * 16777215
//          for lat, available trough getLat as a double, these steps are performed:
//              90 added to recsale from -90.0 .. +90.0 to 0.0 .. 180.0, 
//              divided by 180 to scale from 0.0 .. 1.0,
//              multiplied by 16777215, being (256 * 256 * 256 - 1) to scale into a 3 byte number
//          LatitudeBinary and LongitudeBinary are stored as a uint32_t type; displayed as hex
//          >> my values would be 52.632656, 4.738389  >> 142.6, 184.7  >> 0.792, 0.513 >> 13,294,326 , 8,609,432 >> 00CA DAF6, 0083 5E98
//    
//          altitudeGps is stored as uint16_t
//          >> my value 26.7 >> 001A
//          accuracy is stored as uint8_t
//          >> my value  147 >> 0093
//    
//          In Paul's example, the payload is stored in 9 bytes in txBuffer[0] .. txBuffer[8]
//            
//            txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;     //   CA  --> expected in Alkmaar
//            txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;      //   DA  --> expected in Alkmaar
//            txBuffer[2] = LatitudeBinary & 0xFF;               //   F6  --> acceptable in Alkmaar
//            
//            txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;    //   83  --> expected in Alkmaar
//            txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;     //   5E  --> expected in Alkmaar
//            txBuffer[5] = LongitudeBinary & 0xFF;              //   98  --> acceptable in Alkmaar
//              
//            altitudeGps = sodaq_gps.getAlt();                  
//            txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;         //   00  --> expected in Alkmaar
//            txBuffer[7] = altitudeGps & 0xFF;                  //   1A  --> acceptable in Alkmaar
//            
//            hdopGps = sodaq_gps.getHDOP()*10;                  
//            txBuffer[8] = hdopGps & 0xFF;                      //   ??
//    
//       -=-=-=-=-=-=--==---=--=-=-=-=-= 
//          Key to do all the transformations and assignments correctly is the precision of the types
//            float is a single precision (32 bit, 4 bytes)  with a finesse of approx -2,147,483,648 .. 2,147,483,648
//            double is a double precision (64 bit, 8 bytes) floating point data type
//            Unsigned long  stores 32 bits (4 bytes)  0 .. 4,294,967,295
//            Unsigned short  stores 16 bits (2 bytes)  0 .. 65,535
//            uint8_t is unsigned 8-bit integer, 1 byte    0 .. 255
//            uint16_t is unsigned 16-bit integer, 2 bytes   0 .. 65,535
//            uint32_t is unsigned 32-bit integer, 4 bytes   0 .. 4,294,967,295
//          When we are using 3 bytes in our messages, 
//                 we can have unsigned int values ranging 0 .. 16,777,215

//     what is done here in our example...
//        In our version, we use the TinyGPS library, where we have a choice of:
//            f_get_position (easier to use, adds 2k for floating point libraries)
//            get_position (faster and lighter)
//        gps.f_get_position(&flatitude, &flongitude, &age)
//            Returns actual position, as float type values (not pointers). 
//              lat -90.0 .. 90.0, lon -180 .. 180
//              where 1 deg is approx 111,111 meters
//              and with floats having a finesse of a little over 9 decimal positions, using little over 2 postions before decimal point,
//                this leaves us 7 decimal positions of accuracy, or: the number format can distinct between 10cm precise locations
//        gps.get_position(&latitude, &longitude, &age)
//            Returns latitude*10,000 and longitude*10,000 as long type. The age variable must be unsigned long type.
//              lat -900,000 .. 900,000; lon -1,800,000 .. 1,800,000
//              where 1/10,000 degree is about 10 meters --> we'd like a finer grain so do not use get_position
//           Alternative explanation is that it returns degrees * 1,000,000  http://arduiniana.org/libraries/tinygps/
    
