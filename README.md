# GPS-Lora-Balloon-rfm95-TinyGPS
Lora location transmittor with TinyGPS and rfm95 for fablab Kaasfabriek Junior IoT Baloon Challenge
  See https://www.thethingsnetwork.org/labs/story/junior-iot-ballonnen-challenge

Libraries required:
   TinyGPS       https://github.com/mikalhart/TinyGPS
   arduino-lmic  https://github.com/matthijskooijman/arduino-lmic

//
//    Deze software is voor het Junior Internet of Things project in februari 2017
//                 uitgevoerd met leerling teams bij fablab de Kaasfabriek in Alkmaar
//                         de locatie van deze software op het internet is natuurlijk geheim... 
//                                         ...anders zouden ze makkelijk kunen afkijken!
//    Wil je meer weten?                              
//        Software plakker: Dennis --> dennis at didnotreveal dot nope
//        Educatie kletser: Marco --> marco@kaasfabriek.nl
//        Regie en inspiratie: Kaasfabriek --> info at kaasfabriek punt nl
//

Practical tips for soldering quality and very nice pictures on how to build your first node
   https://www.thethingsnetwork.org/labs/story/build-the-cheapest-possible-node-yourself
    --> you will need to use our port numbers instead


Suggested pin mapping:
  this is a new mapping where the boards can be positioned next to each other as shown in schema below, 
    also can be folded over for compact stacked effect, with the 868 antenna outside the stack

                              USB to serial programmer
                                    RX TX 3.3 GND
                                  x  │  │  │  │  x
                               ╔══╬══╬══╬══╬══╬══╬══╗
                               ║ GRN   RXI   GND BLK║              * ports marked are non-mappable
                               ║    TXD   VCC       ║               all others can be adjusted 
                               ╬ TXD            RAW ╬x              in software
  ╔═════════════════╗          ╬ RXD  ARDUINO   GND ╬──gnd         
  ║                 ║          ╬ RST  pro mini  RST ╬x             x ports must remain unconnected 
  ║  GPS      (PPS) ╬x         ╬ GND  (2 euro)  VCC ╬──3.3v         
  ║  NEO-6M     RXD ╬───────tx─╬ 2               A3 ╬────────────────────────────────────────┐
  ║  (10 euro)  TXD ╬───────rx─╬ 3    8Mhz 32kb  A2 ╬──────────────────────────────────────┐ │
  ║             GND ╬─gnd      ╬ 4    3.3 volt   A1 ╬────────────────────────────────────┐ │ │
 ┌╬  3.3 volt   VCC ╬─3.3v     ╬ 5               A0 ╬──────┐    ╔════════════════════╗   │ │ │
 │║                 ║          ╬ 6         (led) 13*╬*─────│─┐  ╬ GND           DIO2 ╬───│─│─┘
 │╚═════════════════╝          ╬ 7          MISO 12*╬─*────│─│──╬*MISO  RFM95   DIO1 ╬───│─┘
 │                             ╬ 8          MOSI 11*╬──*───│─│──╬*MOSI (10 eur) DIO0 ╬───┘
 │  ┌─────────────┐            ╬ 9               10 ╬────┐ │ └──╬*SCK           3.3V ╬──3.3v
 └──┤ gps antenna │            ║  (GND A4 A5 A6 A7) ║    │ └────╬ NSS  3.3 volt DIO4 ╬x
    └─────────────┘            ╚════╬══╬══╬══╬══╬═══╝    └──────╬ RESET         DIO3 ╬x
                                                               x╬ DIO5           GND ╬gnd
                                                             gnd╬ GND            ANT ╬──┐
                               |         |         |            ╚════════════════════╝  │
                               └─|───────┴─|───────┴─|───────┐                          │
                                 └─────────┴─────────┴─────┐ │                 ///////////
                                                           - +                 / helical /
 choice:                                        JST male connector             / 868Mhz  /
  2x AA battery -> JST female connector                                        / antenna /
  1S lipo 3.7v -> lipo protect -> JST female connector                         /         /
  2s lipo 7.4V -> lipo protect -> step down to 3.3 v -> JST female connector   /   0.15  /
  5 volt from usb -> step down to 3.3 volt -> JST female connector             /   euro  /
  9 volt or 12 volt battery -> step down to 3.3 v -> JST female connector      ///////////


