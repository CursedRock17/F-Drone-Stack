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

### Board Future Checklist
- [] Update from pads to JST-SH 01x08 shrouded male Connector
- [] Try to reduce length from 97.5mm to 90mm to fit drone frame perfectly
- [] Try to reduce width from 42mm to 30mm to fit drone frame perfectly
- [] Swap from Teensy 4.1 -> Teensy 4.0 to reduce length of MCU

### Parts List

| Item           | Part                    | Datasheet                                             | Link                                                                                                                                                                                                   |
|----------------|------------------------:| -----------------------------------------------------:| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| MCU/FC         | Teensy 4.1              | [Datasheet](https://www.pjrc.com/store/teensy41.html) | [Link](https://www.amazon.com/PJRC-Teensy-4-1-Without-Pins/dp/B088D3FWR7/ref=asc_df_B088D3FWR7?mcid=d246140623733dac86bd615b7e876da5&hvocijid=1131540087249221467-B088D3FWR7-&hvexpln=73&tag=hyprod-20)
| IMU            | Sen0142                 | [Datasheet](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/2204/SEN0142_Web.pdf?_gl=1*w6n5j7*_up*MQ..*_gs*MQ..&gclid=Cj0KCQjwuvrBBhDcARIsAKRrkjcRADOJV8dn-wtU01sinVUfGwrihMSZQWBl3n9ofJSYGxhJTd8fDG4aAldAEALw_wcB&gclsrc=aw.ds) | [Link](https://www.amazon.com/HiLetgo-MPU-6050-Accelerometer-Gyroscope-Converter/dp/B078SS8NQV)
| Radio Rx       | ELRS Lite RX            | [Datasheet](https://support.betafpv.com/hc/en-us/articles/4402596816409-Manual-for-Nano-Receiver) | [Link](https://betafpv.com/products/elrs-lite-receiver?variant=39582961664134)
| ESC (4in1)     | Mamba F30 Mini          | [Datasheet](https://cdn.shopifycdn.net/s/files/1/0027/2708/4144/files/MK4_F405_MINI_ICM42688P.jpg?v=1658891020) | [Link](https://speedyfpv.com/products/diatone-mamba-f30mini-2-5s-30a-brushless-4-in-1-esc-blheli_s-dshot600?variant=43539432702166&country=US&currency=USD&utm_medium=product_sync&utm_source=google&utm_content=sag_organic&utm_campaign=sag_organic&gad_source=1&gad_campaignid=21376024065&gclid=Cj0KCQjw1JjDBhDjARIsABlM2Ss1GEUBidOX0kPm21NHFVc2s-RZn4kGX1sH1SGZCS8viviWAjkMLesaAoZAEALw_wcB)
| BEC (5V)       | MP1584EN Mini           | [Datasheet](https://www.makerfabs.com/desfile/files/MP1584.pdf) | [Link](https://www.amazon.com/MP1584EN-DC-DC-Converter-Adjustable-Module/dp/B01MQGMOKI)
| Battery (2S)   | GNB Li-ion 2S           | None                                                            | [Link](https://www.gaoneng.shop/products/gaoneng-gnb-2s-7.4v-3000mah-10c-xt60-li-ion-battery-made-with-sony-18650-vtc6)
| Motors (x4)    | HGLRC 11000KV Brushless | [Datasheet](https://www.hglrc.com/products/specter-1202-5-11000kv-brushless-motor?srsltid=AfmBOoqW-ta1qVEoeQz9RS-2Xiud78sCV2YQTAbIZQbJYDzdP6lwds6F) | [Link](https://www.hglrc.com/products/specter-1202-5-11000kv-brushless-motor?srsltid=AfmBOoqW-ta1qVEoeQz9RS-2Xiud78sCV2YQTAbIZQbJYDzdP6lwds6F)
| M3 Screw (x10) | Hex Screw 0.5mm pitch   | None | [Link](https://us.misumi-ec.com/vona2/detail/221000551376/?HissuCode=CSHCS-316L-M3-16&gad_source=1&gad_campaignid=19817031019&gclid=CjwKCAjwg7PDBhBxEiwAf1CVu5zaJW_kZJtc979KgaLEIrtVIYL096AOgAkgUkWIPzicWDwsFN0UiBoCMWEQAvD_BwE)
| Frame          | Carbon Fiber Micro UAV  | None | None Yet
