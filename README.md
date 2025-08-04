# Welcome to the F-Drone-Stack

## Introduction
This is project for the MATRIX Lab at the University of Maryland's SMART
building. It utilizes dRehmFlight (by nickrehm) as a flight controller.

The intent of the project is to create a stack for replicable, autonmous
drones. These drones utilize a soft-core ARM Cortex M3 on an FPGA to
allow the software to not be restricted to any certain chip which can
be used on quadrotors of any size or scale.

Clone this repository with the version you'd like: `git clone https://github.com/CursedRock17/F-Drone-Stack.git -b humble`

## Software Setup
--------------------------------------------
### Required External Resources (Libraries)
 - Vivavdo [Download](https://www.amd.com/en/products/software/adaptive-socs-and-fpgas/vivado/vivado-buy.html)
 - ARM [Cortex M3](https://developer.arm.com/documentation/101483/0000/introduction/directory-structure) replicable architecture for Artix A7 FPGA

### CRSF Protocol Overview
 - CRSF Telemetry [PX4 Guide](https://docs.px4.io/main/en/telemetry/crsf_telemetry.html)
 - Telemtry [Messages](https://docs.px4.io/main/en/telemetry/crsf_telemetry.html#telemetry-messages)

<details>
<summary> Arduino Setup Steps </summary>

Prerequiste Installs

1. Install [Arduino](https://www.arduino.cc/en/software/) for your respective system
2. Install the [Teensyduino](https://www.pjrc.com/teensy/td_download.html) add-on for your respective system
3. Install the [CRSFforArduino](https://github.com/ZZ-Cat/CRSFforArduino) library
4. Navigate to the directory in which you cloned the repo: `cd ~/user/F-Drone-Stack`
5. You have successfully installed the software stack!

</details>

<details>
<summary> Vivado Setup Steps </summary>

** Be Warned Vivado is a **FAT** install **
1. Install [Vivado](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vivado-design-tools.html) for your OS
    1. For Linux: Download the `.bin` file, open up a terminal, `cd ~/Downloads`
    2. Make the file installable: `chmod +x <installer_name>.bin`
    3. Run install: `sudo <installer_name>.bin`
2. Go through the setup manager, you only need the free version for this project
    1. In Detail: Sign-in with your AMD account
    2. Install Vivado ML Standard
    3. In Devices, add "7-Series > Artix-7 FPGAs"
    4. Agree to all the licenses, scrolling to the bottom
    5. Install in the "/tools/Xilinx" directory, or wherever's best for you
    6. Cry as you watch your RAM wither away
        1. If you're on Linux, install the cable drivers, so we can program it: `cd /tools/Xilinx/data/xicom/cable_drivers/lin64/install_script/install_drivers`
        2. Make sure you're added, sudo adduser $USER dialout
    7. Source the install: "source /tools/Xilinx/<version>/settings64.sh"
    8. Run the program, (Linux): `vivado` (Windows): double click app
3. Install the [ARM Cortex IP](https://developer.arm.com/documentation/101483/0000/?lang=en)
    1. See if you're respective university/company already has it (UMD does!), if not apply for the [DesignStart program](https://www.arm.com/resources/program-reg/flexible-access-designstart-tier)
    2. Once you gain access go to [ARM Developer's Product Hub](https://developer.arm.com/downloads/)
    3. Go to [DesignStart Physical IP](https://developer.arm.com/downloads/search?programme=DesignStart&term=Cortex-M3+Processor&entitled=false) and grab the `Cortex M3`
4. Boot up Vivado, we can do so by sourcing the shell script:

</details>

### Running the Software
1. Open the Arduino IDE
2. We now need to open the correct version of the software to make adjustments: In the top left corner of the IDE go to “File > Open > ~/user/F-Drone-Stack/dRehmFlight/Versions/dRehmFlight_Teensy_BETA_1.3”
3. We now need to select the correct board, which in our case is the Teensy 4.1 (but 4.0 works just fine). Go to “Tools > Board > Teensy > Teensy 4.1”. Make sure CPU speed is “600 mHz” and USB type is “Serial”
4. At this point in time acquire a data transfer USB and plug it into the Teensy. We can hit “Sketch > Upload” and the board should flash the Arduino file
    1. Note an error I came across was not having the rules for the Teensy board already installed. So if you come across this error do the following:
    2. Open up a terminal window and go to the rules directory, on Linux: “cd /etc/udev/rules.d” or make the directory if it doesn’t exist “mkdir -p /etc/udev/rules.d”
    3. We need to add the Teensy Rules File, now “touch 00-teensy.rules”
    4. In the text editor of your choice, copy the entire file linked above and paste it into the file. Reflash dRehmFlight and the upload should complete
5. Enjoy!

### Flight Computer Setup Steps (Intelligence Stack)

Provides to sort of intelligence like waypoint provision, pose estimation,
and path planning access.

[In Flight Computer Section](./FlightComputer).

### Hardware Setup

[In Hardware Section](./Hardware), steps to find needed parts, assemble,
and troubleshoot the mechanical and electrical parts to a sample drone

You can also find any CAD Files and eCAD files pertaining to the drone

### Handset Setup 

While this code will work autonomously, it's a good idea as always to 
allow manual control via a human pilot, for the testing I used to the 
[Radiomaster Zorro](https://radiomasterrc.com/collections/zorro-radio-control-series/products/zorro-radio-controller).
We will flashing everything through UART and over Wifi.

1) Download the [ExpressLRS Configurator](https://github.com/ExpressLRS/ExpressLRS-Configurator/releases/)
it will make it easier to bind the Handset (Tx) to the ELRS Receiver (Rx)
2) You can plug your Teensy into a computer to get 5V passing through the receiver 
Once it is on, open that Configurator application. There will some options:
  - Releases : Whatever version you installed 
  - Device Category : BETAFPV 2.4 GHz 
  - Device : BETAFPV 2400 RX Lite 
  - Flashing Method : UART
Now, hit `Build and Flash`, it might take a couple minutes.
3) We can now flash the receiver over wifi, download the [ELRS Firmware](https://support.betafpv.com/hc/en-us/articles/24408930358169-Firmware-for-ELRS-Lite-Receiver-2-4GHz)
before you actually flash, since you'll lose access to the internet. 
After waiting about 30 secs there should be a new wifi network:
`ExpressLRS RX` with the password "expresslrs". Join it! 
4) It will automatically put you in the browser so that you can acccss 
the `Firmware Update` section, *Select* the firmware you just downloaded.
Now I had some trouble here where it would tell me there's a mismatch and kick me
out, this happened a couple of times, just repeat it 3-4 times and it will download for you.
5) We can now pair the handset, in the Zorro `Hardware` section (Page 6 in SYS) 
there's a subgroup : `Bind Configuration`, use the right scroll wheel, then press in
6) We will now do the same wifi style setup but with the TX instead.
7) Congrats the two items should be paired!

After you've paired the two together we should tune our controller for the drone.
TODO : Insert Handset Values of Channels
