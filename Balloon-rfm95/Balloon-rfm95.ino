/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * Do not forget to define the radio type correctly in config.h. in lmic library
 *
 * Modified By DenniZr - First test version
 *******************************************************************************/
 
 // done: investigate OTAA, good description here: Moteino, LMIC and OTAA Walkthrough https://github.com/lukastheiler/ttn_moteino
 // TODO: OTAA test code can be tested on device
 // TODO: Disable radio listening. 
 
 // TODO: add GPS flightmode. Good test script seems available here:  https://ukhas.org.uk/guides:ublox6
 // done: GPS power save. The u-center software can be used to make the configurations you want. See: https://www.youtube.com/watch?v=iWd0gCOYsdo
 // TODO: if no good read from GPS in last 2 minutes, then reset arduino as a manual reset fixes the problem?? Or auto-reset every 120 minutes. investigate
  
 // TODO: We will also create a new account 'Kaasfabriek' at some later day where our teams wil be building their stuff.
 // TODO: Create and apply new device ID

#define DEBUG     // if DEBUG is defined, some code is added to display some basic debug info
#define DEB // UG_XL  // if DEBUG_XL is defined, some code is added to display more detailed debug info

//////////////////////////////////////////////
// GPS libraries, mappings and things
//////////////////////////////////////////////
#include <SoftwareSerial.h> 
#include <TinyGPS.h>
//#include "libraries_adjusted/TinyGPS_adjusted_kaasfabriek/TinyGPS.h"   // <TinyGPS.h>
SoftwareSerial ss(3, 2);  // ss RX, TX --> GPS TXD, RXD
TinyGPS gps;

bool GPS_values_are_valid = true;
boolean  gpsEnergySavingWantsToActivate = false;      // this is set to true once gps fix is found
long  gpsEnergySavingStartDelayMillis = 15000;        // Device starts in normal mode. Energy saving is started xx time after a gps fix was found.
long  gpsEnergySavingWantsToActivateStartTime = 0;
int  gpsEnergySavingActivated = false;                // this is set to true once energy saving is activated

//////////////////////////////////////////////
// LMIC and RFM95 mapping and things
//////////////////////////////////////////////
#include <lmic.h>
#include <hal/hal.h>
//#include "libraries_adjusted/lmic_adjusted_kaasfabriek/lmic/lmic.h" //<lmic.h>
//#include "libraries_adjusted/lmic_adjusted_kaasfabriek/hal/hal.h"  // <hal/hal.h>
#include <SPI.h>  //MISO MOSI SCK stuff
#include "keys.h"  // the personal keys to identify our own nodes
const unsigned  TX_INTERVAL = 250;  // transmit interval
dr_t LMIC_DR_sequence[] = {DR_SF7, DR_SF10, DR_SF7, DR_SF7, DR_SF7, DR_SF7, DR_SF7, DR_SF9, DR_SF7, DR_SF7, DR_SF7, DR_SF7 };      //void LMIC_setDrTxpow (dr_t dr, s1_t txpow)
int  LMIC_DR_sequence_count = 12;
int  LMIC_DR_sequence_index = 0;

const  lmic_pinmap lmic_pins = { .nss = 14,    .rxtx = LMIC_UNUSED_PIN,    .rst = 10,    .dio = {17, 16, 15}, };

uint8_t  mydata[14];  // mydata[9] allows for GPS location. a few bytes added to the memory buffer to play with
const unsigned message_size = 9;  // 9 bytes are needed into the ttn tracker service
//const unsigned message_size =11; //sending too large message makes the ttntracker ignore it, allowing us to see payload at ttnonsole

static  osjob_t sendjob;

// os_ interfaces for callbacks only used in over-the-air activation, so functions can be left empty here
void  os_getArtEui (u1_t* buf) { }
void  os_getDevEui (u1_t* buf) { }
void  os_getDevKey (u1_t* buf) { } 

#include "kaasfabriek_gps.cpp"      // 
#include "kaasfabriek_rfm95.cpp"



//////////////////////////////////////////////////////////
//// Kaasfabriek routines for gps
////////////////////////////////////////////



void put_gpsvalues_into_sendbuffer(float flat, float flon, float alt, int hdopNumber)
{    
  uint32_t LatitudeBinary = ((flat + 90) / 180) * 16777215;
  uint32_t LongitudeBinary = ((flon + 180) / 360) * 16777215;
  uint16_t altitudeGps = alt;         // altitudeGps in meters, alt from tinyGPS is float in meters
  if (alt<0) altitudeGps=0;   // unsigned int wil not allow negative values and warps them to huge number, needs to be zero'ed
  // uint8_t accuracy = hdopNumber*10;   // needs to be /10 instead of *10 as per example JP
  uint8_t accuracy = hdopNumber/10;   // from TinyGPS horizontal dilution of precision in 100ths, TinyGPSplus seems the same in 100ths as per MNEMA string
  
  mydata[0] = ( LatitudeBinary >> 16 ) & 0xFF;
  mydata[1] = ( LatitudeBinary >> 8 ) & 0xFF;
  mydata[2] = LatitudeBinary & 0xFF;

  mydata[3] = ( LongitudeBinary >> 16 ) & 0xFF;
  mydata[4] = ( LongitudeBinary >> 8 ) & 0xFF;
  mydata[5] = LongitudeBinary & 0xFF;

  // altitudeGps in meters into unsigned int
  mydata[6] = ( altitudeGps >> 8 ) & 0xFF;
  mydata[7] = altitudeGps & 0xFF;

  // hdop in tenths of meter
  mydata[8] = accuracy & 0xFF;
  
  mydata[9] = 0;  // fill up next bytes in buffer, just for play. As-if null terminated string.
  mydata[10] = 0xFF;  // dummy filler byte
}


void process_gps_values()
{ 
  // retrieve some usefull values from GPS library
  float flat, flon, alt;
  unsigned long age; 
  int hdopNumber;  
  
  gps.f_get_position(&flat, &flon, &age);  // lat -90.0 .. 90.0 as a 4 byte float, lon -180 .. 180 as a 4 byte float, age in 1/1000 seconds as a 4 byte unsigned long
  alt = gps.f_altitude();    // signed float altitude in meters
  hdopNumber = gps.hdop();   // int 100ths of a meter

  // check if possibly invalid
  GPS_values_are_valid = true;
  if (flat == TinyGPS::GPS_INVALID_F_ANGLE)    GPS_values_are_valid = false;
  if (flon == TinyGPS::GPS_INVALID_F_ANGLE)    GPS_values_are_valid = false;
  if (hdopNumber == TinyGPS::GPS_INVALID_HDOP) GPS_values_are_valid = false;
  if (age == TinyGPS::GPS_INVALID_AGE)         GPS_values_are_valid = false;
  
  if (alt == TinyGPS::GPS_INVALID_F_ALTITUDE)  GPS_values_are_valid = false;   // if alt, hdop remain giving errors, possibly the GPS character read misses every start few characters of every feed. Solution: make the code lighter so it returns quicker to character read. Or process a bit of buffer while doing other actions, see TinyGPS example.

  // if valid, put into buffer
  if (GPS_values_are_valid) put_gpsvalues_into_sendbuffer( flat, flon, alt, hdopNumber);
    // after init, sendbuffer holds 0,0 lovation; after first fix it will retain the last valid location
  
  Serial.print(".");
  //show me something
  #ifdef DEBUG
  // keep some values out as seems to take performance and/or make for code to miss GPS sentences
  
  Serial.println();
  Serial.print("Data: ");
  if (GPS_values_are_valid) Serial.print("(valid) ");
  if (!GPS_values_are_valid) Serial.print("(** INVALID");
  if (flat == TinyGPS::GPS_INVALID_F_ANGLE)    {Serial.print(" lat="); Serial.print(flat);}
  if (flon == TinyGPS::GPS_INVALID_F_ANGLE)    {Serial.print(" lon="); Serial.print(flon);}
  if (hdopNumber == TinyGPS::GPS_INVALID_HDOP) {Serial.print(" hdop="); Serial.print(hdopNumber);}
  if (age == TinyGPS::GPS_INVALID_AGE)         {Serial.print(" age="); Serial.print(age);}
  if (alt == TinyGPS::GPS_INVALID_F_ALTITUDE)  {Serial.print(" alt="); Serial.print(alt);}
  if (!GPS_values_are_valid) Serial.print(" **) ");
  Serial.print("  LAT, LON=");
  Serial.print( flat, 6);   
  Serial.print(", ");
  Serial.print(flon, 6); // 52.632656, 4.738389
  Serial.print(" hdop=");
  Serial.print( hdopNumber);
  Serial.print(" alt=");
  Serial.print( alt );
  Serial.print(" AGE=");
  Serial.print(age);
  Serial.println("");
  #endif
  
  #ifdef DEBUG_XL
  unsigned long chars = 0;
  unsigned short sentences = 0, failed = 0;
  gps.stats(&chars, &sentences, &failed);
  
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENT=");
  Serial.print(sentences);
  Serial.print(" ERR=");
  Serial.print(failed);
  
  //uint32_t sat; 
  //sat = gps.satellites();
  
  //Serial.print(" SAT=");
  //Serial.print( sat);
  
  if (chars == 0)
    Serial.println("** No characters from GPS: check wiring **");
  else if (age > 5000)
    Serial.println("Warning: possible stale GPS data (age over 5 seconds)");
  else
    Serial.println("GPS Data is fresh (age less than 5 seconds)");

  Serial.print("For TTN message LatitudeBinary, LongitudeBinary, altitudeGps, accuracy: ");
//    Serial.print( LatitudeBinary, HEX);
//    Serial.print(", ");
//    Serial.print( LongitudeBinary, HEX );
//    Serial.print(", ");
//    Serial.print( altitudeGps, HEX );
//    Serial.print(", ");
//    Serial.println( accuracy, HEX );
  Serial.println("expected   CA DA F. 83 5E 9. 0 .. .. " );     
  Serial.println("    dummy   7F FF FF 7F FF FF 0 0 0 " );    
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
  if (message_size>6) Serial.print(" ");
  if (message_size>6) Serial.print( mydata[7], HEX );
  if (message_size>7) Serial.print(" ");
  if (message_size>7) Serial.print( mydata[8], HEX );
  if (message_size>8) Serial.print(" / ");
  if (message_size>8) Serial.print( mydata[9], HEX );
  if (message_size>9) Serial.print(" ");
  if (message_size>9) Serial.print( mydata[10], HEX );
  Serial.println("]");
  #endif    
}



//
// SOME EXPLANATION ON THE NUMBERS
//
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



//////////////////////////////////////////////////
// Kaasfabriek routines for rfm95
///////////////////////////////////////////////


// do_send call is scheduled in event handler
void do_send(osjob_t* j){  
  // starting vesion was same as https://github.com/tijnonlijn/RFM-node/blob/master/template%20ttnmapper%20node%20-%20scheduling%20removed.ino
    
    Serial.println("\ndo_send was called.");
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.

        #ifdef DEBUG
        Serial.println("  expected   CA DA F? 83 5E 9? 0 ?? ?? " );  
        Serial.println("    dummy   7F FF FF 7F FF FF 0 0 0 " );    
        #endif  
        Serial.print(" mydata[] = [");
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
        if (message_size>7) Serial.print(" ");
        if (message_size>7) Serial.print( mydata[7], HEX );
        if (message_size>8) Serial.print(" ");
        if (message_size>8) Serial.print( mydata[8], HEX );
        if (message_size>9) Serial.print(" / ");
        if (message_size>9) Serial.print( mydata[9], HEX );
        if (message_size>10) Serial.print(" ");
        if (message_size>10) Serial.print( mydata[10], HEX );
        Serial.print("]    ");
        
        Serial.print("DR [ ");
        Serial.print( LMIC_DR_sequence_index );
        Serial.print(" ] = ");
        Serial.print( LMIC_DR_sequence[LMIC_DR_sequence_index] );
        if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF7) Serial.print(" DR_SF7 "); 
        if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF8) Serial.print(" DR_SF8 "); 
        if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF9) Serial.print(" DR_SF9 "); 
        if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF10) Serial.print(" DR_SF10 "); 
        if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF11) Serial.print(" DR_SF11 "); 
        if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF12) Serial.print(" DR_SF12 "); 
        
        // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
        // for the ttn mapper always use SF7. For Balloon, up to SF12 can be used, however that will require 60 minutes quiet time
        LMIC_setDrTxpow(LMIC_DR_sequence[LMIC_DR_sequence_index],14);   // void LMIC_setDrTxpow (dr_t dr, s1_t txpow)... Set data rate and transmit power. Should only be used if data rate adaptation is disabled.
        
        LMIC_DR_sequence_index = LMIC_DR_sequence_index + 1;
        if (LMIC_DR_sequence_index >= LMIC_DR_sequence_count) LMIC_DR_sequence_index=0;

        // NOW SEND SOME DATA OUT
        //  LMIC_setTxData2( LORAWAN_APP_PORT, LMIC.frame, LORAWAN_APP_DATA_SIZE, LORAWAN_CONFIRMED_MSG_ON );
        LMIC_setTxData2(1, mydata, message_size, 0);   

        Serial.println(" - Packet queued");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


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


void lmic_init()
{
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

// Disable data rate adaptation - per http://platformio.org/lib/show/842/IBM%20LMIC%20framework%20v1.51%20for%20Arduino
//      and http://www.developpez.net/forums/attachments/p195381d1450200851/environnements-developpement/delphi/web-reseau/reseau-objet-connecte-lorawan-delphi/lmic-v1.5.pdf/
//LMIC_setAdrMode(0);     // Enable or disable data rate adaptation. Should be turned off if the device is mobile
    // Disable link check validation
    LMIC_setLinkCheckMode(0);  //Enable/disable link check validation. Link check mode is enabled by default and is used to periodically verify network connectivity. Must be called only if a session is established.
// Disable beacon tracking
//LMIC_disableTracking ();  // Disable beacon tracking. The beacon will be no longer tracked and, therefore, also pinging will be disabled.
// Stop listening for downstream data (periodical reception)
//LMIC_stopPingable();  //Stop listening for downstream data. Periodical reception is disabled, but beacons will still be tracked. In order to stop tracking, the beacon a call to LMIC_disableTracking() is required

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);   // void LMIC_setDrTxpow (dr_t dr, s1_t txpow)... Set data rate and transmit power. Should only be used if data rate adaptation is disabled.

}






// Schedule TX event every this many seconds (might become longer due to duty cycle limitations).
// https://www.thethingsnetwork.org/forum/t/limitations-data-rate-packet-size-30-seconds-uplink-and-10-messages-downlink-per-day-fair-access-policy/1300
//   Golden rule: 30 seconds air-time per device per day
//    For 10 bytes of payload (plus 13 bytes overhead), this is whispered to imply:
//    approx 20 messages per day at SF12  --> approx one per hour 
//    500 messages per day at SF7  --> 500 / 24 = 20 per hour, or 3 minutes in-between sends (average)
//  for 9 bytes message, the 500 count is upped to 523 or 22 messages per hour, or 2:45 minute average between sends
//
//  For each step-up in SF Spreading factor the air-time doubles, so the allowed number of messages gets divided in half
//    meaning for example you can send one SF8 for the same cost as two SF7 messages
//  Simply one setting and one interval would be easiest.
//
//      spreading | avg time between | nbr of messages
//      factor    | 9 byte sends     | per hour or per day
//     -----------|------------------|-----------------
//      DR_SF7    |  2:45 minutes    |  22 per hour, 528 per day
//      DR_SF8    |  5:30 minutes    |  11 per hour, 264 per day
//      DR_SF9    |  11 minutes      |  5.5 per hour, 132 per day
//      DR_SF10   |  22 minutes      |  2.75 per hour, 66 per day
//      DR_SF11   |  44 minutes      |  1.3 per hour, 33 per day
//      DR_SF12   |  1:28 hours      |              16.5 per day
// Sending pattern would be:
//     a. All messages are SF7, interval 2:45 minutes = 165 seconds (setting is at 105); in tests a setting of 130 results in 180 sec intervals so the library adds 50 sec.
//
//  However in tests we found that coverage in the Netherlands still varies, so we vant a few loud sends per hour.
//  Let's make a mix. You can do what you prefer... I'd prefer to see where my balloon is once every 5 or 6 minutes, and one loud send every half hour.
//     b. Loudest. One SF12 message - will fill up the hour and will not allow more messages. Not desirable.
//     c. Less loud. One SF11 uses 44 minutes, leaves budget for 6 messages as SF7. Total of 7 messages per hour, not frequent enough.
//     d. Even less loud. One DR_SF10 is 22 minutes of budget, one SF9 adds 11 minutes; leaves budget to send another 10 messages as SF7 to add another 27.5 min. 
//        Total 12 messages in one hour. 
//        interval 5 minutes, message stream: DR_SF7, DR_SF7, DR_SF7, DR_SF7, DR_SF7, DR_SF10, DR_SF7, DR_SF7, DR_SF7, DR_SF7, DR_SF7, DR_SF9 
//  let's build and test scenario (d)  with interval at 5 min = 300 sec (setting at 250)


///////////////////////////////////////////////
//  arduino init and main
///////////////////////////////////////////


void setup() {
    Serial.begin(115200);   // whether 9600 or 115200; the gps feed shows repeated char and cannot be interpreted, setting high value to release system time
    
    // load the send buffer with dummy location 0,0. This location 0,0 is recognized as dummy by TTN Mapper and will be ignored
    put_gpsvalues_into_sendbuffer( 0, 0, 0, 0);
    
    Serial.println("\n\nJunior Internet of Things RFM95 Starting ");
    Serial.print("This device is "); Serial.print(myDeviceName); Serial.print(" ("); Serial.print(DEVADDR); Serial.println(") ");
    Serial.println();
  
    // GPS serial
    ss.begin(9600);         // software serial with GPS module. Reviews tell us software serial is not best choice; 
                            // https://www.pjrc.com/teensy/td_libs_TinyGPS.html explains to use UART Serial or NewSoftSerial 

    lmic_init();  // code moved to sub as per example JP
    
    // Start job
    //do_send(&sendjob);
    // Start job delayed so system can look at GPS first
    os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(5), do_send);
            
}

void loop() {
    
    // process the serial feed from GPS module
    Serial.println("\nRead GPS... ");
    char c;
    unsigned long start = millis();
    do {   
      while (ss.available()) {
        char c = ss.read();
        Serial.write(c); // uncomment this line if you want to see the GPS data flowing
        #ifdef DEBUG_XL
        Serial.write(c); // uncomment this line if you want to see the GPS data flowing
        #endif
        
        if (gps.encode(c)) { // Did a new valid sentence come in?
            process_gps_values();
            
            // allow energy saving mode only if a fix has been achieved
            if(GPS_values_are_valid && !gpsEnergySavingWantsToActivate && !gpsEnergySavingActivated ) { 
              Serial.println("\nFirst gps fix found, counting down to switch to Energy Saving. ");
              gpsEnergySavingWantsToActivate = true;
              gpsEnergySavingWantsToActivateStartTime = millis();
            }  
         }          
      }
    } while (millis() - start < 5000); // explanation:
       // keep xxxx millis focussed on reading the ss. the datablurp will be less than one second
       // a 3 second focus also works great if gps is in power saving mode
       // if too low a value the gps blurp of data will be interrupted and incomplete due conflicting system interrupts
       // if too high a value then system wil delay scheduled jobs and the LMIC send sequence will take too long
     
    os_runloop_once();  // system picks up just the fisrst job from all scheduled jobs

    // can we go into energy saving mode yet?
    if(gpsEnergySavingWantsToActivate && !gpsEnergySavingActivated ) {
      if(millis() - gpsEnergySavingWantsToActivateStartTime > gpsEnergySavingStartDelayMillis) {
        Serial.println("\nGwitching GPS to Energy Saving. ");
        uint8_t data[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x10, 0x27, 0x01, 0x00, 0x01, 0x00, 0x4D, 0xDD}; // from u-center software - the changes the usb interval to every 10 seconds instead of every 1 second
        ss.write(data, sizeof(data));
        
        gpsEnergySavingWantsToActivate = false;
        gpsEnergySavingActivated = true;
      }
    }
}

