# GPS-Lora-Balloon-rfm95-TinyGPS 

##Libraries required
- TinyGPS       https://github.com/mikalhart/TinyGPS
- arduino-lmic  https://github.com/matthijskooijman/arduino-lmic

##Lora location transmittor with TinyGPS and rfm95 
```
  adjustment for fablab Kaasfabriek Junior IoT Baloon Challenge february 2017
             met leerling teams bij fablab de Kaasfabriek in Alkmaar
                     deze software op het internet is natuurlijk geheim...
                                  ...anders kunnen ze makkelijk afkijken! 
```
See https://www.thethingsnetwork.org/labs/story/junior-iot-ballonnen-challenge 

- Software plakker: Dennis --> dennis at didnotreveal dot dev0
- Educatie kletser: Marco --> marco@kaasfabriek.nl
- Regie en inspiratie: Kaasfabriek --> info at kaasfabriek punt nl

##Important
how to build your first node, practical soldering tips in nice pictures,  <BR/>
https://www.thethingsnetwork.org/labs/story/build-the-cheapest-possible-node-yourself <BR/>
 --> you will need to use our pin mapping instead <BR/>

##Pin mapping
   ---------------------------------------------------------------------------- 
   Suggested pin mapping:                                                       
   this new mapping puts the boards next to each other as shown in schema below,
         can be folded into compact stack with the 868 antenna outside the pile 
   -----------------------------------------------------------------------------
                        ║                 ║ USB to serial programmer        
                        ║3.3 TX RX GND 5V ║                              
                        ╚══╬══╬══╬══╬══╬══╝ while programming, power comes <BR/>
                           x  │  │  │  x       from external supply to get <BR/>
                                 │  │  │               enough current for GPS <BR/>
                              ┌──│──┘  │                                     <BR/>
                              │  │  x  │      ARDUINO pro mini (2 euro)      <BR/>
     GPS unit            ╔═╬══╬══╬══╬══╬══╬═╗ 8Mhz 32kb 3.3 volt             <BR/>
     NEO-6M              ║ G TX RX VCC GND B║                                   <BR/>
     3.3 volt            ║                  ║      * ports marked are non-mappable <BR/>
     (10 euro)           ╬ TXD          RAW ╬x         others can be re-configured <BR/>
    ╔═══════════╗        ╬ RXD          GND ╬─gnd                                  <BR/>
    ║           ║        ╬ RST          RST ╬x        x marked remains unconnected <BR/>
    ║     (PPS) ╬x       ╬ GND          VCC ╬─3.3v                                <BR/>
    ║       RXD ╬─────tx─╬ 2             A3 ╬────────────────────────────────┐ <BR/>
    ║  ro)  TXD ╬─────rx─╬ 3             A2 ╬──────────────────────────────┐ │ <BR/>
    ║       GND ╬─gnd    ╬ 4             A1 ╬────────────────────────────┐ │ │ <BR/>
   ┌╬  lt   VCC ╬─3.3v   ╬ 5             A0 ╬────┐    ╔═══════════════╗  │ │ │ <BR/>
   │║           ║        ╬ 6       (led) 13*╬────│─┐  ╬ GND      DIO2 ╬──│─│─┘ <BR/>
   │╚═══════════╝        ╬ 7        MISO 12*╬────│─│──╬*MISO     DIO1 ╬──│─┘   <BR/>
   │                     ╬ 8        MOSI 11*╬────│─│──╬*MOSI     DIO0 ╬──┘    <BR/>
   │ ┌─────────────┐     ╬ 9             10 ╬──┐ │ └──╬*SCK      3.3V ╬──3.3v  <BR/>
   └─┤ gps antenna │     ║(GND A4 A5 A6 A7) ║  │ └────╬ NSS      DIO4 ╬x      <BR/>
     └─────────────┘     ╚═══╬══╬══╬══╬══╬══╝  └──────╬ RESET    DIO3 ╬x     <BR/>
                                                     x╬ DIO5      GND ╬gnd   <BR/>
                                                   gnd╬ GND       ANT ╬──┐   <BR/>
        power to    power to    power to              ╚═══════════════╝  │   <BR/>
         GPS       arduino     RFM                     RFM95             │   <BR/>
          | |       | |       | |                      3.3 volt     /////////// <BR/>
          └─|───────┴─|───────┴─|───────┐              (10 euro)    / helical / <BR/>
            └─────────┴─────────┴─────┐ │                           / 868Mhz  / <BR/>
                                      - +                           / antenna / <BR/>
                               JST male connector                   /////////// <BR/>
    power supply suggestions:                                                       <BR/>
     2x AA battery -> JST female connector                                  <BR/>
    1S lipo 3.7v -> lipo protect -> JST female connector                     <BR/>
    2s lipo 7.4V -> lipo protect -> step down to 3.3 v -> JST female connector <BR/>
    5 volt from usb -> step down to 3.3 volt -> JST female connector          <BR/>
    9 volt or 12 volt battery -> step down to 3.3 v -> JST female connector     <BR/>
   ----------------------------------------------------------------------------- <BR/>
