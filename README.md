# GPS-Lora-Balloon-rfm95-TinyGPS

##Libraries required
- TinyGPS       https://github.com/mikalhart/TinyGPS
- arduino-lmic  https://github.com/matthijskooijman/arduino-lmic

##Lora location transmittor with TinyGPS and rfm95 for Kaasfabriek project
```
  adjustment for fablab Kaasfabriek Junior IoT Baloon Challenge february 2017
             met leerling teams bij fablab de Kaasfabriek in Alkmaar
                     deze software op het internet is natuurlijk geheim...
                                  ...anders kunnen ze makkelijk afkijken!
```
See https://www.thethingsnetwork.org/labs/story/junior-iot-ballonnen-challenge

- Software plakker: Dennis --> dennis.ruigrok@gmail.com
- Educatie kletser: Marco --> marco@kaasfabriek.nl
- Regie en inspiratie: Kaasfabriek --> info at kaasfabriek punt nl

##Important
how to build your first node, practical soldering tips in nice pictures,
https://www.thethingsnetwork.org/labs/story/build-the-cheapest-possible-node-yourself
 --> you will need to use our pin mapping instead

##Pin mapping
    Suggested pin mapping:
    this new mapping puts the boards next to each other as shown in schema below,
          can be folded into compact stack or embedded in a business card size
    -----------------------------------------------------------------------------
                            ║                 ║ USB to serial programmer
                            ║3.3 TX RX GND 5V ║ while programming, power comes
                            ╚══╬══╬══╬══╬══╬══╝ from external supply to get
                               x  │  │  │  x    enough current for GPS
                               ┌──│──┘  │
                               │  │  x  │      ARDUINO pro mini (2 euro)
      GPS unit            ╔═╬══╬══╬══╬══╬══╬═╗ 8Mhz 32kb 3.3 volt
      NEO-6M              ║ G TX RX VCC GND B║
      3.3 volt            ║                  ║                           ///////
      (10 euro)           ╬ TXD          RAW ╬x                          helical
     ╔═══════════╗        ╬ RXD          GND ╬───gnd                      868Mhz
     ║           ║        ╬ RST          RST ╬x                          antenna
     ║     (PPS) ╬x       ╬ GND          VCC ╬───3.3v                    ///////
     ║       RXD ╬─────tx─╬ 2             A3 ╬────────────────┐              │
     ║  ro)  TXD ╬─────rx─╬ 3             A2 ╬─────────────┐  │ 3.3V     GND │
     ║       GND ╬─gnd    ╬ 4             A1 ╬──────────┐  │  │  │        │  │
    ┌╬  lt   VCC ╬─3.3v   ╬ 5             A0 ╬──────┐ ╔═╬══╬══╬══╬══╬══╬══╬══╬═╗
    │║           ║        ╬ 6       (led) 13*╬─────┐│ ║D2 D1 D0 3V D4 D3 GND ANT
    │╚═══════════╝        ╬ 7        MISO 12*╬────┐│└───────────────┐          ║
    │                     ╬ 8        MOSI 11*╬───┐│└─────────────┐  │ RFM95    ║
    │ ┌─────────────┐     ╬ 9             10 ╬─┐ └│───────────┐  │  │ 3.3 volt ║
    └─┤ gps antenna │     ║                  ║ │  └────────┐  │  │  │ (10 euro)║
      └─────────────┘     ╚═══╬══╬══╬══╬══╬══╝ └───────────│──│──│──│──┐       ║
                                                      ║    │  │  │  │  │       ║
                                                      ╚═╬══╬══╬══╬══╬══╬══╬══╬═╝
                                                      GND MI MO SCK NS RE D5 GND
          GPS       arduino     RFM95
           | |       | |       | |  
           └─|───────┴─|───────┴─|───────┐  
             └─────────┴─────────┴─────┐ │          
                                       - +           
                                JST female connector
     power supply suggestions:           
<<<<<<< HEAD
     - 1S lipo 3.7v with lipo protect and JST male connector
     - 2x AA battery -> JST male connector
     - 1S lipo 3.7v -> lipo protect -> JST male connector
     - 2s lipo 7.4V -> lipo protect -> step down to 3.3v -> JST male connector
     - 5 volt from usb -> step down to 3.3 volt -> JST male connector
     - 9 volt or 12 volt battery -> step down to 3.3v -> JST male connector
    -----------------------------------------------------------------------------
=======
     - 1S lipo 3.7v with lipo protect and JST male connector 
     - 2x AA battery -> JST male connector                          
     - 1S lipo 3.7v -> lipo protect -> JST male connector                     
     - 2s lipo 7.4V -> lipo protect -> step down to 3.3 v -> JST male connector 
     - 5 volt from usb -> step down to 3.3 volt -> JST male connector          
     - 9 volt or 12 volt battery -> step down to 3.3 v -> JST male connector     
    -----------------------------------------------------------------------------
>>>>>>> origin/master
