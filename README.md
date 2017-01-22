# GPS-Lora-Balloon-rfm95-TinyGPS <br/>
 <br/>
Libraries required: <br/>
   TinyGPS       https://github.com/mikalhart/TinyGPS <br/>
   arduino-lmic  https://github.com/matthijskooijman/arduino-lmic <br/>
 <br/>
Lora location transmittor with TinyGPS and rfm95 <br/>
  adjustment for fablab Kaasfabriek Junior IoT Baloon Challenge february 2017 <br/>
             met leerling teams bij fablab de Kaasfabriek in Alkmaar <br/>
                     deze software op het internet is natuurlijk geheim... <br/>
                                  ...anders kunnen ze makkelijk afkijken! <br/>
  See https://www.thethingsnetwork.org/labs/story/junior-iot-ballonnen-challenge <br/>
 <br/>
  Software plakker: Dennis --> dennis at didnotreveal dot dev0 <br/>
  Educatie kletser: Marco --> marco@kaasfabriek.nl <br/>
  Regie en inspiratie: Kaasfabriek --> info at kaasfabriek punt nl <br/>
 <br/>
Important: <br/>
how to build your first node, practical soldering tips in nice pictures,  <br/>
https://www.thethingsnetwork.org/labs/story/build-the-cheapest-possible-node-yourself <br/>
 --> you will need to use our pin mapping instead <br/>
 <br/>
---------------------------------------------------------------------------- <br/>
Suggested pin mapping:                                                       <br/>
 this new mapping puts the boards next to each other as shown in schema below, <br/>
         can be folded into compact stack with the 868 antenna outside the pile <br/>
----------------------------------------------------------------------------- <br/>
                        ║                 ║ USB to serial programmer        <br/>
                        ║3.3 TX RX GND 5V ║                               <br/>
                        ╚══╬══╬══╬══╬══╬══╝ while programming, power comes <br/>
                           x  │  │  │  x       from external supply to get <br/>
                              │  │  │               enough current for GPS <br/>
                           ┌──│──┘  │                                     <br/>
                           │  │  x  │      ARDUINO pro mini (2 euro)      <br/>
  GPS unit            ╔═╬══╬══╬══╬══╬══╬═╗ 8Mhz 32kb 3.3 volt             <br/>
  NEO-6M              ║ G TX RX VCC GND B║                                   <br/>
  3.3 volt            ║                  ║      * ports marked are non-mappable <br/>
  (10 euro)           ╬ TXD          RAW ╬x         others can be re-configured <br/>
 ╔═══════════╗        ╬ RXD          GND ╬─gnd                                  <br/>
 ║           ║        ╬ RST          RST ╬x        x marked remains unconnected <br/>
 ║     (PPS) ╬x       ╬ GND          VCC ╬─3.3v                                <br/>
 ║       RXD ╬─────tx─╬ 2             A3 ╬────────────────────────────────┐ <br/>
 ║  ro)  TXD ╬─────rx─╬ 3             A2 ╬──────────────────────────────┐ │ <br/>
 ║       GND ╬─gnd    ╬ 4             A1 ╬────────────────────────────┐ │ │ <br/>
┌╬  lt   VCC ╬─3.3v   ╬ 5             A0 ╬────┐    ╔═══════════════╗  │ │ │ <br/>
│║           ║        ╬ 6       (led) 13*╬────│─┐  ╬ GND      DIO2 ╬──│─│─┘ <br/>
│╚═══════════╝        ╬ 7        MISO 12*╬────│─│──╬*MISO     DIO1 ╬──│─┘   <br/>
│                     ╬ 8        MOSI 11*╬────│─│──╬*MOSI     DIO0 ╬──┘    <br/>
│ ┌─────────────┐     ╬ 9             10 ╬──┐ │ └──╬*SCK      3.3V ╬──3.3v  <br/>
└─┤ gps antenna │     ║(GND A4 A5 A6 A7) ║  │ └────╬ NSS      DIO4 ╬x      <br/>
  └─────────────┘     ╚═══╬══╬══╬══╬══╬══╝  └──────╬ RESET    DIO3 ╬x     <br/>
                                                  x╬ DIO5      GND ╬gnd   <br/>
                                                gnd╬ GND       ANT ╬──┐   <br/>
      power to    power to    power to             ╚═══════════════╝  │   <br/>
        GPS       arduino     RFM                   RFM95             │   <br/>
         | |       | |       | |                    3.3 volt     /////////// <br/>
         └─|───────┴─|───────┴─|───────┐            (10 euro)    / helical / <br/>
           └─────────┴─────────┴─────┐ │                         / 868Mhz  / <br/>
                                     - +                         / antenna / <br/>
                            JST male connector                   /////////// <br/>
 choice:                                                                  <br/>
  2x AA battery -> JST female connector                                  <br/>
 1S lipo 3.7v -> lipo protect -> JST female connector                     <br/>
 2s lipo 7.4V -> lipo protect -> step down to 3.3 v -> JST female connector <br/>
 5 volt from usb -> step down to 3.3 volt -> JST female connector          <br/>
 9 volt or 12 volt battery -> step down to 3.3 v -> JST female connector     <br/>
----------------------------------------------------------------------------- <br/>