# Hardware Section and Setup

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

### Parts List

| Item        | Part                    | Datasheet                                             | Link                                                                                                                                                                                                   |
|-------------|------------------------:| -----------------------------------------------------:| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| MCU/FC      | Teensy 4.0              | [Datasheet](https://www.pjrc.com/store/teensy40.html) | [Link](https://www.electromaker.io/shop/product/teensy-40?gad_source=1&gad_campaignid=17338710367&gclid=CjwKCAjw4K3DBhBqEiwAYtG_9AoOmjvFSXFEkkdpQb7JbPI9Wz47hlxhBNEYIXRS0C-5BRerjckGFBoCwNUQAvD_BwE)
| IMU         | HiLetgo GY-521          | [Datasheet](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/2204/SEN0142_Web.pdf?_gl=1*w6n5j7*_up*MQ..*_gs*MQ..&gclid=Cj0KCQjwuvrBBhDcARIsAKRrkjcRADOJV8dn-wtU01sinVUfGwrihMSZQWBl3n9ofJSYGxhJTd8fDG4aAldAEALw_wcB&gclsrc=aw.ds) | [Link](https://www.amazon.com/HiLetgo-MPU-6050-Accelerometer-Gyroscope-Converter/dp/B078SS8NQV)
| Radio Rx    | ELRS Lite RX 2.4 GHz    | [Datasheet](https://support.betafpv.com/hc/en-us/articles/4402596816409-Manual-for-Nano-Receiver) | [Link](https://betafpv.com/products/elrs-lite-receiver?variant=39582961664134)
| ESC (4in1)  | Mamba F30 Mini          | [Datasheet](https://cdn.shopifycdn.net/s/files/1/0027/2708/4144/files/MK4_F405_MINI_ICM42688P.jpg?v=1658891020) | [Link](https://speedyfpv.com/products/diatone-mamba-f30mini-2-5s-30a-brushless-4-in-1-esc-blheli_s-dshot600?variant=43539432702166&country=US&currency=USD&utm_medium=product_sync&utm_source=google&utm_content=sag_organic&utm_campaign=sag_organic&gad_source=1&gad_campaignid=21376024065&gclid=Cj0KCQjw1JjDBhDjARIsABlM2Ss1GEUBidOX0kPm21NHFVc2s-RZn4kGX1sH1SGZCS8viviWAjkMLesaAoZAEALw_wcB)
| BEC (5V)    | MP1584EN Mini           | [Datasheet](https://www.makerfabs.com/desfile/files/MP1584.pdf) | [Link](https://www.amazon.com/MP1584EN-DC-DC-Converter-Adjustable-Module/dp/B01MQGMOKI)
| Battery (2S)| GNB Li-ion 2S           | None                                                            | [Link](https://www.gaoneng.shop/products/gaoneng-gnb-2s-7.4v-3000mah-10c-xt60-li-ion-battery-made-with-sony-18650-vtc6)
| Motors (x4) | HGLRC 11000KV Brushless | [Datasheet](https://www.hglrc.com/products/specter-1202-5-11000kv-brushless-motor?srsltid=AfmBOoqW-ta1qVEoeQz9RS-2Xiud78sCV2YQTAbIZQbJYDzdP6lwds6F) | [Link](https://www.hglrc.com/products/specter-1202-5-11000kv-brushless-motor?srsltid=AfmBOoqW-ta1qVEoeQz9RS-2Xiud78sCV2YQTAbIZQbJYDzdP6lwds6F)
| Optical Flow| Matek 3901-L0X          | [Datasheet](https://www.mateksys.com/?portfolio=3901-l0x#tab-id-2) | [Link](https://www.getfpv.com/mateksys-optical-flow-lidar-sensor-3901-l0x.html?utm_source=google&utm_medium=cpc&utm_campaign=DM+-+NB+-+PMax+-+Shop+-+No-index+-+SM+-+ALL+%7C+Full+Funnel&utm_content=pmax_x&utm_keyword=&utm_matchtype=&campaign_id=20799936859&network=x&device=c&gc_id=20799936859&gad_source=1&gad_campaignid=20796067361&gclid=CjwKCAjwg7PDBhBxEiwAf1CVu5J9NG6OtZcLzF9Gt-jSDHbckIvNjY7H0FQzeP1tNwYG2YqhNJe9JxoCZTMQAvD_BwE)
| Mechanical  | Parts                   | None                                                            | None
| Screws(x10) | M3 Hex Top              | None                                                            | None
| Drone Frame | Carbon Fiber Micro UAS  | None                                                            | None
| Custom PCB  | Copper Board            | The PCB in the Hardware Section                                 | None
| Mount       | Battery Holder          | None                                                            | None
| JST SH      | 8-Pin Vertical Shrouded | [Datasheet](https://www.jst-mfg.com/product/pdf/eng/eSH.pdf)                                                            | [Link](https://www.digikey.com/en/products/detail/jst-sales-america-inc/BM08B-SRSS-TB/926700)
| JST Connect | 4-Pin Vertical Shrouded | None                                                            | None

### Assembly

1) Grab your drone Frame, electrical components, and turn on your soldering iron
2) Use helping hands to hold the PCB still, start by soldering pinouts to tracks
    1) Start with the BEC, ensure it's in the correct orientation, arrow points left
    and solder pins 3 & 7, using the PCB editor as a guide
    2) Next we solder the Teensy 4.0 going in the clockwise direction, starting
    with Vin (the top right pin), the USB port should face the BEC, skip 4pins,
    then solder pins 21, 20, (skip 2), 17, 16, (skip 6), 3.3V, (skip 4), 9,
    8, 7, 6, (skip 4), 1, 0
    3) Solder all 3 JST headers, the 8-pin on the left, two 4 pins in the middle
    4) We can now solder the ELRS Reciever which doesn't have large pads, so
    be careful
3) Mount the PCB to the drone frame itself, using two M2 (2") screws, they
   should lie underneath the ESC, so taking it off must be done in reverse
4) After we've solder all the components onto the board, we can mount things to the PCB
    1) Mount the ESC to the PCB using 4 M2 (2") screws with nylon for bending
    2) The other component being mounted to the PCB is the IMU, with two 3" screws
5) The last step is soldering x-awg wire with connectors to the components
   There are 3 components which are wired in this way:
     - ESC (8 pin connector, using 6, left side)
     - Optical Flow (4 pin connector, top right)
     - IMU (4 pin connector, middle)
6) We must now screw in the battery pack, to the bottom with the holder
7) The drone is now assembled!
