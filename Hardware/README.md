# Hardware Section and Setup

Welcome to the mechanical and electrical parts section of the project
here you can find the entire process of building a custom drone

### Parts List

#### Electrical Components

| Item             | Part                    | Datasheet                                             | Link                                                                                                                                                                                                   |
|------------------|------------------------:| -----------------------------------------------------:| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| MCU/FC           | Teensy 4.0              | [Datasheet](https://www.pjrc.com/store/teensy40.html) | [Link](https://www.electromaker.io/shop/product/teensy-40?gad_source=1&gad_campaignid=17338710367&gclid=CjwKCAjw4K3DBhBqEiwAYtG_9AoOmjvFSXFEkkdpQb7JbPI9Wz47hlxhBNEYIXRS0C-5BRerjckGFBoCwNUQAvD_BwE)
| IMU              | HiLetgo GY-521          | [Datasheet](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/2204/SEN0142_Web.pdf?_gl=1*w6n5j7*_up*MQ..*_gs*MQ..&gclid=Cj0KCQjwuvrBBhDcARIsAKRrkjcRADOJV8dn-wtU01sinVUfGwrihMSZQWBl3n9ofJSYGxhJTd8fDG4aAldAEALw_wcB&gclsrc=aw.ds) | [Link](https://www.amazon.com/HiLetgo-MPU-6050-Accelerometer-Gyroscope-Converter/dp/B078SS8NQV)
| Radio Rx         | ELRS Lite RX 2.4 GHz    | [Datasheet](https://support.betafpv.com/hc/en-us/articles/4402596816409-Manual-for-Nano-Receiver) | [Link](https://betafpv.com/products/elrs-lite-receiver?variant=39582961664134)
| ESC (4in1)       | Mamba F30 Mini          | [Datasheet](https://cdn.shopifycdn.net/s/files/1/0027/2708/4144/files/MK4_F405_MINI_ICM42688P.jpg?v=1658891020) | [Link](https://speedyfpv.com/products/diatone-mamba-f30mini-2-5s-30a-brushless-4-in-1-esc-blheli_s-dshot600?variant=43539432702166&country=US&currency=USD&utm_medium=product_sync&utm_source=google&utm_content=sag_organic&utm_campaign=sag_organic&gad_source=1&gad_campaignid=21376024065&gclid=Cj0KCQjw1JjDBhDjARIsABlM2Ss1GEUBidOX0kPm21NHFVc2s-RZn4kGX1sH1SGZCS8viviWAjkMLesaAoZAEALw_wcB)
| BEC (5V)         | MP1584EN Mini           | [Datasheet](https://www.makerfabs.com/desfile/files/MP1584.pdf) | [Link](https://www.amazon.com/MP1584EN-DC-DC-Converter-Adjustable-Module/dp/B01MQGMOKI)
| Battery (2S)     | GNB Li-ion 2S           | None                                                            | [Link](https://www.gaoneng.shop/products/gaoneng-gnb-2s-7.4v-3000mah-10c-xt60-li-ion-battery-made-with-sony-18650-vtc6)
| Motors (x4)      | HGLRC 11000KV Brushless | [Datasheet](https://www.hglrc.com/products/specter-1202-5-11000kv-brushless-motor?srsltid=AfmBOoqW-ta1qVEoeQz9RS-2Xiud78sCV2YQTAbIZQbJYDzdP6lwds6F) | [Link](https://www.hglrc.com/products/specter-1202-5-11000kv-brushless-motor?srsltid=AfmBOoqW-ta1qVEoeQz9RS-2Xiud78sCV2YQTAbIZQbJYDzdP6lwds6F)
| Optical Flow     | Matek 3901-L0X          | [Datasheet](https://www.mateksys.com/?portfolio=3901-l0x#tab-id-2) | [Link](https://www.getfpv.com/mateksys-optical-flow-lidar-sensor-3901-l0x.html?utm_source=google&utm_medium=cpc&utm_campaign=DM+-+NB+-+PMax+-+Shop+-+No-index+-+SM+-+ALL+%7C+Full+Funnel&utm_content=pmax_x&utm_keyword=&utm_matchtype=&campaign_id=20799936859&network=x&device=c&gc_id=20799936859&gad_source=1&gad_campaignid=20796067361&gclid=CjwKCAjwg7PDBhBxEiwAf1CVu5J9NG6OtZcLzF9Gt-jSDHbckIvNjY7H0FQzeP1tNwYG2YqhNJe9JxoCZTMQAvD_BwE)
| Wires            | 28awg Wires + Connectors| [Datasheet](https://www.molex.com/en-us/products/part-detail/151340402?display=pdf)                                                            | [Link](https://www.amazon.com/Keszoox-Pre-Crimped-Compatible-Controller-Connectors/dp/B09F3TQS9V?th=1)
| Capacitor        | 25V 330uF               | None          | [Link](https://www.digikey.com/en/products/detail/rubycon/25YXF330MEFCT810X12.5/9553865?gclsrc=aw.ds&gad_source=1&gad_campaignid=17336967819&gclid=CjwKCAjw1ozEBhAdEiwAn9qbzVpkC7tEQoLxKylyjwLx9Ek22v4CTqct2tGUgR2duNCr6zZ4ja4pIhoCTzIQAvD_BwE)

#### Mechanical Parts

| Item             | Part                    | Datasheet                                             | Link                                                                                                                                                                                                   |
|------------------|------------------------:| -----------------------------------------------------:| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| Drone Frame      | Carbon Fiber Micro UAS  | None                                                            | None
| Screws(x7)       | M2 12mm Metal Screw     | None                                                            | None
| Nuts(x7)         | M2 Metal Nut            | None                                                            | None
| Screws(x2)       | M3 12mm Nylon Screw     | None                                                            | None
| Nuts(x2)         | M3 Nylon Nut            | None                                                            | None
| Custom PCB       | Copper Board            | The PCB in the Hardware Section                                 | None
| Mount            | Battery Holder          | None                                                            | None
| JST SH           | 8-Pin Vertical Shrouded | [Datasheet](https://www.jst-mfg.com/product/pdf/eng/eSH.pdf)                                                            | [Link](https://www.digikey.com/en/products/detail/jst-sales-america-inc/BM08B-SRSS-TB/926700)
| MX PicoBlade (x2)| 4-Pin Vertical Picoblade| [Datasheet](https://www.molex.com/en-us/products/part-detail/533980471?display=pdf)                                                            | [Link](https://www.digikey.com/en/products/detail/molex/0533980471/699068?_gl=1*6zaf8a*_up*MQ..*_gs*MQ..&gclid=CjwKCAjw4efDBhATEiwAaDBpbmVd34wrg8y2s5RDRtP2scQ2MDIMdRvgA0yoEv-UR2EZkolO0P8Q8BoCRToQAvD_BwE&gclsrc=aw.ds)
| MX PicoBlade (x4)| 3-Pin Male Connector    | [Datasheet](https://www.molex.com/en-us/products/part-detail/2181100301)                                                            | [Link](www.digikey.com/en/products/detail/molex/2181100301/14309272?utm_source=oemsecrets&utm_medium=aggregator&utm_campaign=buynow)


### Assembly

1) Grab your drone Frame, electrical components, and turn on your soldering iron
2) Before you do anything, ensure your PCB's PWR/GND doesn't have any shorts
by testing with a multimeter, there should be 1 big GND plane
3) Use helping hands to hold the PCB still, start by soldering pinouts to tracks
    1) Start with the BEC, ensure it's in the correct orientation, arrow points left
    and solder pins 3 & 7, using the PCB editor as a guide. I placed 2 pin
    headers in each section, soldered those into the board, then sniped them flush
    2) Next we solder the Teensy 4.0 going in the clockwise direction, starting
    with Vin (the top right pin), the USB port should face the BEC, skip 4pins,
    then solder pins 21, 20, (skip 2), 17, 16, (skip 6), 3.3V, (skip 4), 9,
    8, 7, 6, (skip 4), 1, 0. You may want to solder header pins in the vertical
    pinouts of the Teensy itself, then solder matching sockets (14 pin) into
    the board itself, for easy removal of the Teensy.
    3) Solder all 4 JST headers, the 8-pin on the left, two 4-pins on the right
    and another 4-pin on the lower on the right
4) Mount the PCB to the drone frame itself, using three M2 (2") screws, they
   should lie underneath the ESC, so taking it off must be done in reverse
5) The next step is soldering 28-awg wire(Or whatever you have) with
   connectors to the components
   There are 4 components which are wired in this way:
     - ESC Motors (3 connectors each, 4 connectors) Goes RBY for colors,
     string and solder those wires to the edge power pads, then we can
     run the other end of the connector into the corresponding motor
     connectors, so they should click into each other (MX Picoblade)
     male -> female and female -> male
     - ESC (8 pin connector, using 6, top left)
     - IMU (4 pin connector, top right)
     - Optical Flow (4 pin connector, middle right)
     - ELRS (4 pin connector, bottom right)
6) *Optional* : Put a layer of Kapton tape on the bottom of the PCB
7) After we've solder all the components onto the board, we can mount things
   Things have to mounted in a weird order so that we save space
    1) Run the 3 mounting screws for the PCB, through the PCB, but not onto the
    frame
    2) Mount the ESC to the PCB using 4 M2 (2") screws with the custom spacers
    on the opposite side of the ESC, finishing with a M2 nut on the other side
    of the board
    3) The other component being mounted to the PCB is the IMU, with two
    3" screws 12mm Nylon screws. Run the screws through the IMU into rubber
    spacers (gummies) which are finished up by some tight fitting nuts on the
    other side of the PCB
    4) Flip the PCB + Drone over, slotting the mounting screws through and as
    always, finish on the otherside with M2 nuts for each of the 3 screws
8) We must now screw in the battery pack, to the bottom with the holder
9) **Before** plugging in your external connectors or the Teensy, you need to
test the BEC, since it's controlled with a potentiometer. **Ensure no component
can be fried by 7.4V you'll get from the battery**. Do this in a controlled
environment and set up ahead of time, plug in your battery, take a multimeter
and the easy spot to test will be the back of the board where lies the
Teensy's 5V and GND pins, so being careful as to not short the two, put the
meter on each respective side. You should get about 4.95 - 5.15V, if not,
disasmbled. Alternatively you could put temporary header pins into the soldered
socket and test that way. If need be, tune that potentiometer
10) After ensuring you won't fry your connectors, plug in the components.
11) You should use Waxed Nylon String or similar, with a clove hitch knot
followed by a square knot, to tie excess table bundles down to the frame
12) Place your props (not too far down) onto your motors, ensure you have
your props in the correct direction for props out. *Tip* : think about how the
air would flow if it made contact with the prop
13) The drone is now assembled!

### General Board Setup Requirements
- 1.0mm Power/GND width
- 0.5mm Default Clearance
- 0.5mm Default Track Width + Connection Width
- No need for other trace sizes b/c not very intense
- Min Annular Width = 0.5mm
- Min Via Diameter = 1.0mm
- Copper to Hole/Edge Clearance: 0.5mm
- Min Through Hole: 0.25mm
- Hole -> Hole Clearance: 1.0mm

### Considerations for the future
- [] Change Header footprints to corresponding connectors
- [] FlipFlop the Servo connector so on the board it's 1 -> 1
- [] Since we have access to PCBWay, should probably route more concisely
- [] Check the spacing for our headers to ensure the physical part fits
- [] Check the spacing from connector to other components like ESC conn. to BEC
- [] Reverse order pins for IMU since not enough space
- [] Smaller Pitch Surface MNT connectors
- [] Enable Clearance for Props (maybe create design of frame and all)
