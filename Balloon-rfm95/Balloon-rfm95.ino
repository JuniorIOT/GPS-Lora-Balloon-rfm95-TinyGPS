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
 
 // TODO: if no good read from GPS in last 2 minutes, then reset arduino as a manual reset fixes the problem?? Or auto-reset every 120 minutes. investigate

 // TODO: Add arduino sleep mode: need to change the radio scheduler to inline mode? see https://github.com/tijnonlijn/RFM-node/blob/master/template%20ttnmapper%20node%20-%20scheduling%20removed.ino
 
 // TODO: We will also create a new account 'Kaasfabriek' at some later day where our teams wil be building their stuff.
 // TODO: Create and apply new device ID

//#define DEBUG     // if DEBUG is defined, some code is added to display some basic debug info

//#include "LowPower.h"   // help to do power save on the arduino  https://github.com/rocketscream/Low-Power

 //#include <JeeLib.h>  // Include library containing low power functions
 //ISR(WDT_vect) { Sleepy::watchdogEvent(); } // Setup for low power waiting

//////////////////////////////////////////////
// GPS libraries, mappings and things
//////////////////////////////////////////////
#include <SoftwareSerial.h> 
#include <TinyGPS.h>

SoftwareSerial ss(3, 2);  // ss RX, TX --> GPS TXD, RXD
TinyGPS gps;
long gps_fix_count = 0;
long gps_nofix_count = 0;

//////////////////////////////////////////////
// LMIC and RFM95 mapping and things
//////////////////////////////////////////////

// // do not keep radio active to listen to return message in RX2. see https://github.com/matthijskooijman/arduino-lmic/blob/master/src/lmic/config.h
// #define DISABLE_JOIN     // Uncomment this to disable all code related to joining
#define DISABLE_PING     // Uncomment this to disable all code related to ping
#define DISABLE_BEACONS  // Uncomment this to disable all code related to beacon tracking.// Requires ping to be disabled too 

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>  //MISO MOSI SCK stuff
#include "keys.h"  // the personal keys to identify our own nodes

const unsigned  TX_INTERVAL = 60; //  250;  // transmit interval, 5minutes is healthy according to TTN rules
const dr_t LMIC_DR_sequence[] = {DR_SF10, DR_SF7, DR_SF7, DR_SF7, DR_SF7, DR_SF7, DR_SF9, DR_SF7, DR_SF7, DR_SF7, DR_SF7, DR_SF7 };      //void LMIC_setDrTxpow (dr_t dr, s1_t txpow)
const int  LMIC_DR_sequence_count = 12;
int  LMIC_DR_sequence_index = 0;
const  lmic_pinmap lmic_pins = { .nss = 14,    .rxtx = LMIC_UNUSED_PIN,    .rst = 10,    .dio = {17, 16, 15}, };

uint8_t  mydata[14];  // mydata[9] allows for GPS location. a few bytes added to the memory buffer to play with
const unsigned message_size = 11;  // 9 bytes are needed into the ttn tracker service
// byte 0, 1, 2      Latitude     -90 to +90 rescaled to 0 - 16777215
// byte 3, 4, 5      Longitude    -180 to + 180 rescaled to 0 - 16777215
// byte 6, 7         Altitude     2 bytes in meters
// byte 8            GPS DoP      1 byte
// byte 9            Arduino VCC  1 byte in 50ths Volt
// byte 10           cpu temp     1 byte -100 to 155 scaled to 0 - 255

static  osjob_t sendjob;

// os_ interfaces for callbacks only used in over-the-air activation, so functions can be left empty here
void  os_getArtEui (u1_t* buf) { }
void  os_getDevEui (u1_t* buf) { }
void  os_getDevKey (u1_t* buf) { } 

int TX_COMPLETE_was_triggered = 0;  // 20170220 added to allow full controll in main Loop

//////////////////////////////////////////////////////////
//// Kaasfabriek routines for gps
////////////////////////////////////////////

void put_gpsvalues_into_sendbuffer(long l_lat, long l_lon, long l_alt, int hdopNumber) {    

  const double shift_lat =    90 * 1000000;        // range shift from -90M..90M into 0..180M
  const double max_old_lat = 180 * 1000000;        // max value for lat is now 180M
  const double max_3byte =        16777215;        // max value that fits in 3 bytes
  double lat_float = l_lat;                        // put the 4byte LONG into a more precise floating point to prevent round-off effect in calculation
  lat_float = (lat_float + shift_lat) * max_3byte / max_old_lat; // rescale into 3 byte integer range
  uint32_t LatitudeBinary = lat_float;             // clips off anything after the decimal point
  
  Serial.println(F("\n\nInto send buffer "));
  
  #ifdef DEBUG
  Serial.print(F(" LAT=")); Serial.print( l_lat); 
  Serial.print(F(" => ( ")); Serial.print( lat_float);    
  Serial.print(F(" + ")); Serial.print( shift_lat);    
  Serial.print(F(" ) * ")); Serial.print( max_3byte);    
  Serial.print(F(" / ")); Serial.print( max_old_lat);   
  Serial.print(F(" = ")); Serial.print( LatitudeBinary);  
  Serial.print(F(" = HEX ")); Serial.println( LatitudeBinary, HEX); 
  #endif 
  
  const double shift_lon =   180 * 1000000;        // range shift from -180M..180M into 0..360M
  const double max_old_lon = 360 * 1000000;        // max value longitude is now 360M
  double lon_float = l_lon;                        // put the 4byte LONG into a precise floating point memory space
  lon_float = (lon_float + shift_lon) * max_3byte / max_old_lon; // rescale into 3 byte integer range
  uint32_t LongitudeBinary = lon_float;             // clips off anything after the decimal point
  
  #ifdef DEBUG
  Serial.print(F(" LON =")); Serial.print( l_lon); 
  Serial.print(F(" => ( ")); Serial.print( lon_float);    
  Serial.print(F(" + ")); Serial.print( shift_lon);    
  Serial.print(F(" ) * ")); Serial.print( max_3byte);    
  Serial.print(F(" / ")); Serial.print( max_old_lon);   
  Serial.print(F(" = ")); Serial.print( LongitudeBinary);  
  Serial.print(F(" = HEX ")); Serial.println( LongitudeBinary, HEX);  
  #endif
  
  uint16_t altitudeGps = l_alt/100;         // altitudeGps in meters, l_alt from tinyGPS is integer in centimeters
  if (l_alt<0) altitudeGps=0;               // unsigned int wil not allow negative values and warps them to huge number, needs to be zero'ed
  
  #ifdef DEBUG    
  Serial.print(F(" alt=")); Serial.print( l_alt );
  Serial.print(F(" => ")); Serial.print( altitudeGps);
  Serial.print(F(" = HEX ")); Serial.println( altitudeGps, HEX);  
  #endif
  
  uint8_t accuracy = hdopNumber/10;   // from TinyGPS horizontal dilution of precision in 100ths, TinyGPSplus seems the same in 100ths as per MNEMA string
  
  #ifdef DEBUG
  Serial.print(F(" hdop=")); Serial.print( hdopNumber);
  Serial.print(F(" => ")); Serial.print( accuracy);
  Serial.print(F(" = HEX ")); Serial.println( accuracy, HEX);  
  #endif
  
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
  
  #ifdef DEBUG
  Serial.println(F("  Alkmaar = CA DA F. 83 5E 9. 0 .. .. " ));     
  Serial.println(F("  Zero    = 7F FF FF 7F FF FF 0 0 0 " ));
  #endif
  Serial.print(F(" Mydata[]=[ "));
  for(int i=0; i<message_size; i++) {
    Serial.print(mydata[i], HEX); Serial.print(F(" "));
  }
  Serial.println(F("]"));
}

void process_gps_values() { 
  // retrieve values from GPS library, and if valid put them into send buffer
  // NOT USE the float numbers from library to keep memory as low as possible
  long l_lat, l_lon, l_alt;
  unsigned long age; 
  int hdopNumber;  
  bool GPS_values_are_valid = true;
  
  gps.get_position(&l_lat, &l_lon, &age);  // lat -90.0 .. 90.0 as a 4 byte float, lon -180 .. 180 as a 4 byte float, age in 1/1000 seconds as a 4 byte unsigned long
  l_alt = gps.altitude();    // signed float altitude in meters
  hdopNumber = gps.hdop();   // int 100ths of a meter

  // check if possibly invalid
  if (l_lat == TinyGPS::GPS_INVALID_ANGLE)    GPS_values_are_valid = false;
  if (l_lon == TinyGPS::GPS_INVALID_ANGLE)    GPS_values_are_valid = false;
  if (hdopNumber == TinyGPS::GPS_INVALID_HDOP) GPS_values_are_valid = false;
  if (age == TinyGPS::GPS_INVALID_AGE)         GPS_values_are_valid = false;
  
  if (l_alt == TinyGPS::GPS_INVALID_ALTITUDE)  GPS_values_are_valid = false;   // if alt, hdop remain giving errors, possibly the GPS character read misses every start few characters of every feed. Solution: make the code lighter so it returns quicker to character read. Or process a bit of buffer while doing other actions, see TinyGPS example.

  // if valid, put into buffer
  if (GPS_values_are_valid) put_gpsvalues_into_sendbuffer( l_lat, l_lon, l_alt, hdopNumber);
   
  #ifdef DEBUG
  Serial.println();
  Serial.print(F("Data: "));
  if (GPS_values_are_valid) Serial.print(F("(valid) "));
  if (!GPS_values_are_valid) Serial.print(F("(** INVALID"));
  if (l_lat == TinyGPS::GPS_INVALID_ANGLE)    {Serial.print(F(" lat=")); Serial.print(l_lat);}
  if (l_lon == TinyGPS::GPS_INVALID_ANGLE)    {Serial.print(F(" lon=")); Serial.print(l_lon);}
  if (hdopNumber == TinyGPS::GPS_INVALID_HDOP) {Serial.print(F(" hdop=")); Serial.print(hdopNumber);}
  if (age == TinyGPS::GPS_INVALID_AGE)         {Serial.print(F(" age=")); Serial.print(age);}
  if (l_alt == TinyGPS::GPS_INVALID_ALTITUDE)  {Serial.print(F(" alt=")); Serial.print(l_alt);}
  if (!GPS_values_are_valid) Serial.print(F(" **) "));
  Serial.print(F(" LAT, LON="));
  Serial.print( l_lat);   
  Serial.print(F(", "));
  Serial.print( l_lon); 
  Serial.print(F(" hdop="));
  Serial.print( hdopNumber);
  Serial.print(F(" alt="));
  Serial.print( l_alt );
  Serial.print(F(" AGE="));
  Serial.print(age);
  Serial.println();
  #endif  // debug
  
}

void gps_calcChecksum(byte *checksumPayload, byte payloadSize) {
  byte CK_A = 0, CK_B = 0;
  for (int i = 0; i < payloadSize ;i++) {
    CK_A = CK_A + *checksumPayload;
    CK_B = CK_B + CK_A;
    checksumPayload++;
  }
  *checksumPayload = CK_A;
  checksumPayload++;
  *checksumPayload = CK_B;
}

// Send a byte array of UBX protocol to the GPS  https://ukhas.org.uk/guides:ublox6
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    ss.write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
  ss.println();
}

// Calculate expected UBX ACK packet and parse UBX response from GPS  https://ukhas.org.uk/guides:ublox6
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(F(" Reading ACK "));
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;   // CK_A
  ackPacket[9] = 0;   // CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  } 
  while (1) { 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(F("\n(GPS cmd okay)"));
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println(F("\n(GPS cmd timeout)"));
      //ss.flush();  // try to fix
      return false;
    }
 
    // Make sure data is available to read
    if (ss.available()) {
      b = ss.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }
 
    }
  }
}

void gps_init() {
  // load the send buffer with dummy location 0,0. This location 0,0 is recognized as dummy by TTN Mapper and will be ignored
  put_gpsvalues_into_sendbuffer( 0, 0, 0, 0);
  
  // GPS serial starting
  ss.begin(9600);         // software serial with GPS module. Reviews tell us software serial is not best choice; 
                          // https://www.pjrc.com/teensy/td_libs_TinyGPS.html explains to use UART Serial or NewSoftSerial 

  //   https://ukhas.org.uk/guides:ublox6
  // THE FOLLOWING COMMAND SWITCHES MODULE TO 4800 BAUD
  //ss.print(F("$PUBX,41,1,0007,0003,4800,0*13\r\n")); 
  //ss.begin(4800);
  //ss.flush();

  //gps_requestColdStart();  // DO NOT USE: it seems this does a FACTORY RESET and delays getting a solid fix
  gps_SetMode_gpsRfOn();
  gps_setStrings();
  gps_setNavMode(4); // 2=stationary, 3=pedestrian, 4=auto, 5=Sea, 6=airborne 1g, 7=air 2g, 8=air 4g
  
  gps_setPowerMode(1);  // 1=max power, 2=eco, 3=cyclic power save
  gps_read_until_fix_or_timeout(60 * 60);  // time to first fix can be 15 minutes (or multiple). after factory reset, gps needs to acquire full data which is sent out once every 15 minutes; sat data sent out once every 5 minutes
  //gps_setPowerMode(2);
}

void gps_read_until_fix_or_timeout(unsigned long timeOut) {
  Serial.print(F("\nRead GPS until fix is found or time-out at "));
  Serial.print(timeOut);
  Serial.println(F(" sec"));
  
  unsigned long timeoutTime = millis() + timeOut * 1000;
  long gps_fix_count_old = gps_fix_count;
  
  while(gps_fix_count==0 && millis() < timeoutTime) {
    gps_read_5sec();
  }
  
  if (gps_fix_count == gps_fix_count_old) {
    Serial.println(F("\nNO FIX FOUND"));
    gps_nofix_count++; // we want to know how many times no fix was found
  } else {
    Serial.println(F("\nFix was found"));
  }
}
    
void gps_read_chars(int countdown) {
  while (countdown > 0) {
    if(ss.available()) {       
      countdown--;
      char c = ss.read();
      Serial.write(c); 
      if (gps.encode(c)) {   // Did a new valid sentence come in?
        gps_fix_count++;
        process_gps_values();
        Serial.print(F(" [fix] ")); 
      }
    }
  }
}

void gps_read_5sec() {
  Serial.print(F("\nGPS fixes: "));
  Serial.print(gps_fix_count);
  Serial.print(F(" non-fixes: "));
  Serial.print(gps_nofix_count);
  Serial.println(F(". Reading GPS. "));

  char c;
  long gps_fix_count_old = gps_fix_count;
  unsigned long startTime = millis();
  do {   
    while (ss.available()) {
      char c = ss.read();
      //#ifdef DEBUG
      Serial.write(c); 
      //#endif
      
      if (gps.encode(c)) { // Did a new valid sentence come in?
          gps_fix_count++;
          process_gps_values();
          Serial.print(F(" [fix] "));            
       }          
    }
  } while (millis() - startTime < 5000); // reading for 5 seconds

  if (gps_fix_count == gps_fix_count_old) gps_nofix_count++; // we want to know how many times no fix was found
}

void gps_setNavMode(int mode) { // 2=stationary, 3=pedestrian, 4=auto, 5=Sea, 6=airborne 1g, 7=air 2g, 8=air 4g
  Serial.print(F("\n\nGPS Set Nav mode "));    
  Serial.println(mode);

  //Generate the configuration string for Navigation Mode
  //  [0]NavMode, https://wiki.paparazziuav.org/wiki/Module/GPS_UBlox_UCenter
  //    xx 0x00 = Portable, deviation=medium. max 12km height, maz speed 310 m/s, vert 50 m/s  
  //    xx 0x02 = Stationary timing applications, deviation=small. max 9km heighth, max speed 10 m/s, vert 6 m/s 
  //    0x03 = Pedestrian Mode, deviation=small. max 9km height, max speed 30m/s, vert 20m/s 
  //    0x04 = Automotive Mode, deviation=medium. max 5km heightm max speed 62 m/s, vert 15 m/s 
  //    0x05 = Sea Mode, deviation=medium. max alt 500 meters, max speed 25 m/s, vert 5 m/s
  //    0x06 = Airborne < 1G Mode, deviation=large. no 2d fix supported, max 50km height, max speed 250 m/s, vert 100 m/s; high altitude ballooning, less accurate
  //    0x07 = Airborne < 2G Mode, deviation=large. recommended for typical airborne, no 2d fix supported, max 50km height, max speed 250 m/s, vert 100 m/s;
  //    0x08 = Airborne < 4G Mode, deviation=large. only for extremely dynamic, no 2d fix supported, max 50km height, max speed 500 m/s, vert 100 m/s;
  //    0x09 = wrist worn watch (not supported in protocol versions less than 18)
  
  byte arrCommand[] = {
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 
          mode, 
        0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 
        0x2C, 0x01, 0x00,  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  gps_calcChecksum(&arrCommand[2], sizeof(arrCommand) - 4);
  
  byte gps_okay = 0;
  while(!gps_okay) {
    sendUBX(arrCommand, sizeof(arrCommand)/sizeof(uint8_t));
    gps_okay=getUBX_ACK(arrCommand);
  }
  gps_read_chars(300);

  //  ----- the internet has told us:
  //  #define UBLOX_STATIONARY 0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF, 0x03,0x02,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x54
  //  #define UBLOX_PEDESTRIAN 0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF, 0x03,0x03,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0x76
  //  #define UBLOX_AUTOMOTIVE 0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF, 0x04,0x03,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x14,0x98
  //  #define UBLOX_SEA        0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF, 0x05,0x03,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x15,0xBA
  //  #define UBLOX_AIRBORN    0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF, 0x06,0x03,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x16,0xDC

  //Dynamic Model '6' Aiborne < 1g Large Deviation
  //0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
  //Dynamic Model '3' Pedestrian Small Deviation
  //0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x76         

}

void x_gps_setDataRate(int rate) { // 1=1Hz, 2=2Hz, 3=3Hz, 4=4Hz, 5=5Hz
//  Serial.println(F("\n\nGPS Set data rate ");
//  Serial.println(rate);
//
////    //DataRate:  http://playground.arduino.cc/UBlox/GPS
////    //1Hz     = 0xE8 0x03
////    //2Hz     = 0xF4 0x01
////    //3Hz     = 0x4D 0x01
////    //3.33Hz  = 0x2C 0x01
////    //4Hz     = 0xFA 0x00
////    //5Hz     = 0xC8 0x00
//
//  byte datarate01;
//  byte datarate02;
//  switch (rate) {
//    case 1:
//      datarate01 = 0xE8;
//      datarate02 = 0x03;
//      break;
//    case 2:
//      datarate01 = 0xF4;
//      datarate02 = 0x01;
//      break;
//    case 3:
//      datarate01 = 0x4D;
//      datarate02 = 0x01;
//      break;
//    case 4:
//      datarate01 = 0xFA;
//      datarate02 = 0x00;
//      break;
//    default: 
//      // assume 5
//      datarate01 = 0xC8;
//      datarate02 = 0x00;
//      break;
//    break;
//  }
//  
//  byte arrCommand[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, datarate01, datarate02, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00};
//  gps_calcChecksum(&arrCommand[2], sizeof(arrCommand) - 4);
//  byte gps_okay=0;
//  while(!gps_okay) {
//    sendUBX(arrCommand, sizeof(arrCommand)/sizeof(uint8_t));
//    gps_okay=getUBX_ACK(arrCommand);
//  }
//  gps_read_chars(100);
//    
    //#define UBLOX_1HZ        0xB5,0x62,0x06,0x08,0x06,0x00,   0xE8,0x03,  0x01,0x00,0x01,0x00,0x01,0x39       // set rate to 1Hz
    //#define UBLOX_2HZ        0xB5,0x62,0x06,0x08,0x06,0x00,   0xF4,0x01,  0x01,0x00,0x01,0x00,0x0B,0x77       // set rate to 2Hz
    //#define UBLOX_3HZ        0xB5,0x62,0x06,0x08,0x06,0x00,   0x4D,0x01,  0x01,0x00,0x01,0x00,0x64,0x8D       // set rate to 3Hz
    //#define UBLOX_4HZ        0xB5,0x62,0x06,0x08,0x06,0x00,   0xFA,0x00,  0x01,0x00,0x01,0x00,0x10,0x96       // set rate to 4Hz
    //#define UBLOX_5HZ        0xB5,0x62,0x06,0x08,0x06,0x00,   0xC8,0x00,  0x01,0x00,0x01,0x00,0xDE,0x6A       // set rate to 5Hz
}

void gps_setStrings() {
  Serial.println(F("\n\nGPS some strings"));

  // Turning off all GPS NMEA strings apart from GPGGA (fix information) on the uBlox modules
  // we need lat, lon, alt, HDOP  --> keep GGA, GSA
  ss.print(F("$PUBX,40,GLL,0,0,0,0*5C\r\n"));  // GLL = Lat/Lon
  ss.print(F("$PUBX,40,ZDA,0,0,0,0*44\r\n"));  // ZDA = date, time
  ss.print(F("$PUBX,40,VTG,0,0,0,0*5E\r\n"));  // VTG = Vector Track and speed over ground
  ss.print(F("$PUBX,40,GSV,0,0,0,0*59\r\n"));  //GSV = Detailed satellite data
  ss.print(F("$PUBX,40,RMC,0,0,0,0*47\r\n"));    // RMC = recommended minimum data for GPS, no Alt
  // ss.print(F("$PUBX,40,GSA,0,0,0,0*4E\r\n"));  // GSA = Overall Satelite data
  // ss.println(F("$PUBX,40,GGA,0,0,0,0*5A"));   // GGA = Fix information

  gps_read_chars(300);

  //    // manual polling is possible to instruct gps output one specific string:
  //    ss.println(F("$PUBX,00*33");
  
  //  #define GGA_OFF          0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x00,0xFA,0x0F                            // switch GGA off
  //  #define GLL_OFF          0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x00,0xFB,0x11                            // switch GLL off
  //  #define GSA_OFF          0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x00,0xFC,0x13                            // switch GSA off
  //  #define GSV_OFF          0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x00,0xFD,0x15                            // switch GSV off
  //  #define RMC_OFF          0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x04,0x00,0xFE,0x17                            // switch RMC off
  //  #define VTG_OFF          0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x00,0xFF,0x19                            // switch VTG off
  //  #define POSLLH_ON        0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x02,0x01,0x0E,0x47                            // set POSLLH MSG rate
  //  #define STATUS_ON        0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x03,0x01,0x0F,0x49                            // set STATUS MSG rate
  //  #define SOL_ON           0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x06,0x01,0x12,0x4F                            // set SOL MSG rate
  //  #define VELNED_ON        0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x12,0x01,0x1E,0x67                            // set VELNED MSG rate
  //  #define SVINFO_ON        0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x30,0x01,0x3C,0xA3                            // set SVINFO MSG rate
  //  #define TIMEUTC_ON       0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x21,0x01,0x2D,0x85                            // set TIMEUTC MSG rate
  //  #define SBAS_ON          0xB5,0x62,0x06,0x16,0x08,0x00,0x03,0x07,0x03,0x00,0x51,0x08,0x00,0x00,0x8A,0x41   // set WAAS to EGNOS
}

void x_gps_setBaud() {
// Serial.println(F("\n\nGPS set baud rate ");

//  #define UBLOX_115200     0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xBE,0x72 //set speed to 115200
//  #define UBLOX_57600      0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xE1,0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xDC,0xBD //set speed to 57600
//  #define UBLOX_38400      0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x96,0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x91,0x84 //set speed to 38400
//  #define UBLOX_19200      0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x4B,0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x46,0x4B //set speed to 19200
//  #define UBLOX_9600       0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x80,0x25,0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xA0,0xA9 //set speed to 9600

}

void gps_setPowerMode(int mode) {  // 1=max power, 2=eco, 3=cyclic power save
  Serial.print(F("\n\nGPS set power mode "));
  Serial.println(mode);

  byte powerMode;
  // really nice description of power saving modes https://ukhas.org.uk/guides:ublox_psm#ublox_6_power_saving_modes
    
  switch (mode) {
    case 1:
      //Max Performance Mode (default)
      powerMode = 0x00;
      break;
    case 2:
      //Eco Mode Don't want this one but here for reference.
      powerMode = 0x04;
      break;
    default: 
      // assume 3
      //Power Save Mode
      powerMode = 0x01;
      break;
    break;
  }
  byte arrCommand[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, powerMode, 0x22, 0x92};
  gps_calcChecksum(&arrCommand[2], sizeof(arrCommand) - 4);
  byte gps_okay=0;
  while(!gps_okay) {
    sendUBX(arrCommand, sizeof(arrCommand)/sizeof(uint8_t));
    gps_okay=getUBX_ACK(arrCommand);
  }
  gps_read_chars(100);
}

void gps_setPowerMode2() {
  Serial.println(F("\n\nGPS set power mode2"));

    //CFG-PM2
    //Not sure what the implication of "do not enter 'inactive for search' state when no fix" is.
    //Update Period 1 second do not enter 'inactive for search' state when no fix unchecked (default setting)
    //0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x94, 0x02, 0x00, 0xE8, 0x03, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x87, 0x02, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00, 0x9B, 0x75 
    //Update Period 1 second do not enter 'inactive for search' state when no fix checked
    //0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x90, 0x03, 0x00, 0xE8, 0x03, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x86, 0x02, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00, 0x96, 0xEB
    //Update Period 10 seconds , do not enter 'inactive for search' state when no fix checked
    //0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x90, 0x03, 0x00, 0x10, 0x27, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x87, 0x02, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00, 0xE4, 0x8B
    //Update Period 10 seconds , do not enter 'inactive for search' state when no fix unchecked
    //0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x90, 0x02, 0x00, 0x10, 0x27, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x86, 0x02, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00, 0xE1, 0x51
    
}

void gps_SetMode_gpsOff() {
  Serial.println(F("\n\nGPS off "));
  
  ////Set GPS to backup mode (sets it to never wake up on its own) minimal current draw <5mA, loses all settings
  //uint8_t GPSoff[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
  ////Restart GPS
  //uint8_t GPSon[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37};
  
  byte arrCommand[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
  gps_calcChecksum(&arrCommand[2], sizeof(arrCommand) - 4);
  byte gps_okay=0;
  while(!gps_okay) {
    sendUBX(arrCommand, sizeof(arrCommand)/sizeof(uint8_t));
    gps_okay=getUBX_ACK(arrCommand);
  }
  // after this command no gps output is available
}

void gps_SetMode_gpsOn() {
  Serial.println(F("\n\nGPS on"));

  byte arrCommand[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37};
  gps_calcChecksum(&arrCommand[2], sizeof(arrCommand) - 4);
  byte gps_okay=0;
  while(!gps_okay) {
    sendUBX(arrCommand, sizeof(arrCommand)/sizeof(uint8_t));
    gps_okay=getUBX_ACK(arrCommand);
  }
  gps_read_chars(300);
}

void gps_SetMode_gpsRfOff() {
  Serial.println(F("\n\nGPS RF off"));

  ////Switch the RF GPS section off, draws about 5mA, retains its settings, wakes on serial command.
  //uint8_t GPSoff[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00,0x08, 0x00, 0x16, 0x74}
  //uint8_t GPSon[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00,0x09, 0x00, 0x17, 0x76};

  byte arrCommand[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00,0x08, 0x00, 0x16, 0x74};
  gps_calcChecksum(&arrCommand[2], sizeof(arrCommand) - 4);
  byte gps_okay=0;
  while(!gps_okay) {
    sendUBX(arrCommand, sizeof(arrCommand)/sizeof(uint8_t));
    gps_okay=getUBX_ACK(arrCommand);
  }
  // after this command no further gps output is available
}

void gps_SetMode_gpsRfOn() {
  Serial.println(F("\n\nGPS RF on"));

  byte arrCommand[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00,0x09, 0x00, 0x17, 0x76};
  gps_calcChecksum(&arrCommand[2], sizeof(arrCommand) - 4);
  byte gps_okay=0;
  while(!gps_okay) {
    sendUBX(arrCommand, sizeof(arrCommand)/sizeof(uint8_t));
    gps_okay=getUBX_ACK(arrCommand);
  }
  gps_read_chars(100);
}

void x_gps_requestColdStart() {  // this erases all and wipes usefull info
//  Serial.println(F("\n\nGPS cold start ");
//
//  //GPS Cold Start (Forced Watchdog)  cold start, clear all data, or cold start, factory reset
//  //0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5
//
//  byte arrCommand[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5};
//  //gps_calcChecksum(&arrCommand[2], sizeof(arrCommand) - 4);
//  byte gps_okay=0;
//  while(!gps_okay) {
//    sendUBX(arrCommand, sizeof(arrCommand)/sizeof(uint8_t));
//    gps_okay=getUBX_ACK(arrCommand);
//  }
//  gps_read_chars(300);
}

//////////////////////////////////////////////////
// Kaasfabriek routines for rfm95
///////////////////////////////////////////////

void do_send(){  
  // starting vesion was same as https://github.com/tijnonlijn/RFM-node/blob/master/template%20ttnmapper%20node%20-%20scheduling%20removed.ino
    
    Serial.println(F("\ndo_send "));
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.

        #ifdef DEBUG
        Serial.println(F("  expected CA DA F? 83 5E 9? 0 ?? ??" ));  
        Serial.println(F("    dummy 7F FF FF 7F FF FF 0 0 0" ));    
        #endif  
        Serial.print(F(" Mydata[]=[ "));
        for(int i=0; i<message_size; i++) {
          Serial.print(mydata[i], HEX);  Serial.print(F(" "));
        }
        Serial.print(F("]"));
        
        Serial.print(F(" DR="));
        if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF7) Serial.print(F("DR_SF7")); 
        if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF8) Serial.print(F("DR_SF8")); 
        if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF9) Serial.print(F("DR_SF9")); 
        if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF10) Serial.print(F("DR_SF10")); 
        if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF11) Serial.print(F("DR_SF11")); 
        if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF12) Serial.print(F("DR_SF12")); 
        
        // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
        // for the ttn mapper always use SF7. For Balloon, up to SF12 can be used, however that will require 60 minutes quiet time
        LMIC_setDrTxpow(LMIC_DR_sequence[LMIC_DR_sequence_index],14);   // void LMIC_setDrTxpow (dr_t dr, s1_t txpow)... Set data rate and transmit power. Should only be used if data rate adaptation is disabled.
        
        LMIC_DR_sequence_index = LMIC_DR_sequence_index + 1;
        if (LMIC_DR_sequence_index >= LMIC_DR_sequence_count) LMIC_DR_sequence_index=0;

        // NOW SEND SOME DATA OUT
        //  LMIC_setTxData2( LORAWAN_APP_PORT, LMIC.frame, LORAWAN_APP_DATA_SIZE, LORAWAN_CONFIRMED_MSG_ON );
        LMIC_setTxData2(1, mydata, message_size, 0);   
        Serial.println(F(" - Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

// event gets hooked into the system
void onEvent (ev_t ev) {
    Serial.print(F("\n\nonEvent: "));
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
            Serial.println(F("EV_BEACON_TRACK"));
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
            Serial.println(F("EV_JOIN_FAIL"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAIL"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            TX_COMPLETE_was_triggered = 1;  // 20170220 our custom code see https://github.com/tijnonlijn/RFM-node/blob/master/template%20ttnmapper%20node%20-%20scheduling%20removed.ino
            
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes"));
            }
       //     // Schedule next transmission   20170220 disabled the interrupt chain, now all controll in main Loop
       //     os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPL"));
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

void lmic_init() {
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


///////////////////////////////////////////////
//  some other measurements
///////////////////////////////////////////

double GetTemp(void) { //http://playground.arduino.cc/Main/InternalTemperatureSensor

  unsigned int wADC;
  double t;
  
  // The internal temperature has to be used with the internal reference of 1.1V.
  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC
  delay(20);            // wait for voltages to become stable.
  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));  
  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;
  
  t = (wADC - 324.31 ) / 1.22;  // The offset of 324.31 could be wrong. It is just an indication.
  // The returned temperature is in degrees Celsius.
  t = t - 10; // before this correction we had readings of 30 in a room which was 20 deg C, readings of 20 outdoors when it was 6 deg C

  return (t);
}

long readVcc() {  //http://dumbpcs.blogspot.nl/2013/07/arduino-secret-built-in-thermometer.html
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

void put_other_values_into_sendbuffer() {
  long vcc = readVcc();
  uint8_t vcc_bin = vcc /20 ;  // rescale 0-5100 milli volt into 0 - 255 values
  mydata[9] = vcc_bin;
  #ifdef DEBUG
  Serial.print(F("Vcc="));
  Serial.print(vcc);
  Serial.print(F(" mVolt. vcc_bin="));
  Serial.print(vcc_bin);
  #endif

  double temperature = GetTemp();
  uint8_t temperature_bin = temperature + 100;   // rescale -100 to 155 into 0 - 255 values
  mydata[10] = temperature_bin;
  #ifdef DEBUG
  Serial.print(F(" Temperature="));
  Serial.print(temperature);
  Serial.print(F(" temperature_bin="));
  Serial.println(temperature_bin);
  #endif
}


///////////////////////////////////////////////
//  arduino init and main
///////////////////////////////////////////

void setup() {
    Serial.begin(115200);   // whether 9600 or 115200; the gps feed shows repeated char and cannot be interpreted, setting high value to release system time
    
    Serial.print(F("\n\n*** Starting ***\ndevice:")); Serial.println(myDeviceName); 
    Serial.println();


    gps_init();
    lmic_init();  
    
}

void loop() {
  unsigned long startTime = millis();
  
  Serial.println(F("\n\nRead GPS"));

  //gps_wakeup();
  gps_read_until_fix_or_timeout(5*60); // try up to 5 minutes to get a fix
  gps_read_5sec();  // 5 sec extra to get stronger fix
  //gps_Snooze();

  Serial.println(F("\nRead values"));
  put_other_values_into_sendbuffer();
  
  Serial.println(F("\nSending"));
  do_send();
  Serial.println(F("Waiting.."));  
  while (TX_COMPLETE_was_triggered == 0) {
    os_runloop_once();     // system picks up just the first job from all scheduled jobs, needed for the scheduled and interrupt tasks
  }
  TX_COMPLETE_was_triggered = 0;
  Serial.println(F("TX_COMPL"));
  
  
  Serial.print(F("\nSleep GPS "));
  gps_SetMode_gpsRfOff();

  //=--=-=---=--=-=--=-=--=  START SLEEP HERE -=-=--=-=-=-=-==-=-=-

  unsigned long processedTime = millis() - startTime;
  long sleeptime = TX_INTERVAL * 1000 - processedTime;
  if ( sleeptime < 0 ) sleeptime = 0;
  Serial.print(sleeptime / 1000);
  Serial.println(F(" sec"));
  
  //LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);    // this kind of sleep does not work
  //Serial.println(F("sleep2 "));
  //Sleepy::loseSomeTime(8000);  // max 60.000 (60 sec)  // this kind of sleep does not work
  //Serial.println(F("delay "));
  
  delay(sleeptime);
  
  //=--=-=---=--=-=--=-=--=  SLEEP IS COMPLETED HERE -=-=--=-=-=-=-==-=-=-
  
//    gps_SetMode_gpsOn();
  gps_SetMode_gpsRfOn();
  Serial.println(F("Sleep done"));
}


// without float
// Sketch uses 13794 bytes (44%) of program storage space. Maximum is 30720 bytes.
// Global variables use 1669 bytes (81%) of dynamic memory, leaving 379 bytes for local variables. Maximum is 2048 bytes.
// Low memory available, stability problems may occur.

// with float
// Sketch uses 13996 bytes (45%) of program storage space. Maximum is 30720 bytes.
// Global variables use 1669 bytes (81%) of dynamic memory, leaving 379 bytes for local variables. Maximum is 2048 bytes.
// Low memory available, stability problems may occur.

// with debug
// Sketch uses 15944 bytes (51%) of program storage space. Maximum is 30720 bytes.
// Global variables use 1967 bytes (96%) of dynamic memory, leaving 81 bytes for local variables. Maximum is 2048 bytes.
// Low memory available, stability problems may occur.
//    ++> ERROR in radio.c:815

// after F(..) in all print calls:
// Sketch uses 28678 bytes (93%) of program storage space. Maximum is 30720 bytes.
// Global variables use 1335 bytes (65%) of dynamic memory, leaving 713 bytes for local variables. Maximum is 2048 bytes.

// met DEBUG aan:
// Sketch uses 30626 bytes (99%) of program storage space. Maximum is 30720 bytes.
// Global variables use 1331 bytes (64%) of dynamic memory, leaving 717 bytes for local variables. Maximum is 2048 bytes.
