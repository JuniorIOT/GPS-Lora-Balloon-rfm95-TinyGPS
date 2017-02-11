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
                               │  │  x  │      ARDUINO pro mini
                          ╔═╬══╬══╬══╬══╬══╬═╗ 8Mhz 32kb                 ///////
      GPS unit            ║ G TX RX VCC GND B║ 3.3 volt                  helical
      NEO-6M              ║                  ║            gnd            868Mhz
      3.3 volt            ╬ TXD          RAW ╬x            │ 3v          antenna
     ╔═══════╗            ╬ RXD          GND ╬─black─60mm──┘ │           ///////
     ║       ║            ╬ RST          RST ╬x              │               │
     ║ (PPS) ╬x           ╬ GND          VCC ╬─red─48mm──────┘    3v   gnd   │
     ║   RXD ╬──pink─15mm─╬ 2 tx          A3 ╬───white─20mm───┐  red48  │    │
     ║   TXD ╬──brn─15mm──╬ 3 rx          A2 ╬──gray─16mm──┐  │  ┌─┘   blk60 │
     ║   GND ╬─blk30──┐   ╬ 4             A1 ╬─pur─14───┐  │  │  │      └─┐  │
    ┌╬   VCC ╬─red35┐ │   ╬ 5             A0 ╬──────┐ ╔═╬══╬══╬══╬══╬══╬══╬══╬═╗
    │║       ║      │ │   ╬ 6       (led) 13*╬─────┐│ ║D2 D1 D0 3V D4 D3 GND ANT
    │╚═══════╝     3v │   ╬ 7        MISO 12*╬────┐│└──pink─18mm────┐          ║
    │                gnd  ╬ 8        MOSI 11*╬───┐│└──ora─20mm───┐  │ RFM95    ║
    │                     ╬ 9             10 ╬─┐ └│──yel─23mm─┐  │  │ 3.3 volt ║
    │ ┌─────────────┐     ║                  ║ │  └─grn─27─┐  │  │  │ (10 euro)║
    └─┤ gps antenna │     ╚═══╬══╬══╬══╬══╬══╝ └──blue─22──│──│──│──│──┐       ║
      └─────────────┘                                 ║    │  │  │  │  │       ║
                                                      ╚═╬══╬══╬══╬══╬══╬══╬══╬═╝
                                                      GND MI MO SCK NS RE D5 GND
    GPS arduino RFM95       LIPO Protect
    | |   | |    | |         ╔═══════╗        switch               ╔═════════╗
    └─|───┴─|────┴─|─3x─red──╬3v   B+╬─┬─red17─╣/╠──red─lipo─30mm──╬+  1 CELL║
      └─────┴──────┴3x─black─╬gnd  B-╬─│─┬─────────black─lipo─??mm─╬-  LIPO  ║
                             ╚═══════╝ │ │                         ╚═════════╝
                                      JST female connector
                                      to LiPo charger with protect
         
