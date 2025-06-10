# Welcome to the F-Drone-Stack

## Introduction
This is project for the MATRIX Lab at the University of Maryland's SMART
building. It utilizes dRehmFlight (by nickrehm) as a flight controller.

The intent of the project is to create a stack for replicable, autonmous
drones. These drones utilize a soft-core ARM Cortex M3 on an FPGA to
allow the software to not be restricted to any certain chip which can 
be used on quadrotors of any size or scale.

## Software Setup 

### Required External Resources (Libraries)
    BLHeli [Github](https://github.com/bitdump/BLHeli) which will control our ESCs to control the motors
    Vivavdo [Download](https://www.amd.com/en/products/software/adaptive-socs-and-fpgas/vivado/vivado-buy.html)
    ARM [Cortex M3](https://developer.arm.com/documentation/101483/0000/introduction/directory-structure) replicable architecture for Artix A7 FPGA

### Arduino Setup Steps

#### Prerequiste Installs
    1) Install [Arduino](https://www.arduino.cc/en/software/) for your respective system
    2) Install the [Teensyduino](https://www.pjrc.com/teensy/td_download.html) add-on for your respective system
    3) Create an install directory for the project, for the case of simplicity we will call ours “~/user/quadcopter/”
    4) Navigate to the directory: “cd ~/user/quadcopter”
    5) You have successfully installed the software stack!
    

#### Running the Software
    1) Clone this repository with the version you'd like: 'git clone https://github.com/CursedRock17/F-Drone-Stack.git -b kilted'
    2) Open the Arduino IDE
    3) We now need to open the correct version of the software to make adjustments: In the top left corner of the IDE go to “File > Open > ~/user/quadcopter/dRehmFlight/Versions/dRehmFlight_Teensy_BETA_1.3”
    4) We now need to select the correct board, which in our case is the Teensy 4.1 (but 4.0 works just fine). Go to “Tools > Board > Teensy > Teensy 4.1”. Make sure CPU speed is “600 mHz” and USB type is “Serial”
    5) At this point in time acquire a data transfer USB and plug it into the Teensy. We can hit “Sketch > Upload” and the board should flash the Arduino file
        5.1) Note an error I came across was not having the rules for the Teensy board already installed. So if you come across this error do the following:
        5.2) Open up a terminal window and go to the rules directory, on Linux: “cd /etc/udev/rules.d” or make the directory if it doesn’t exist “mkdir -p /etc/udev/rules.d”
        5.3) We need to add the Teensy Rules File, now “touch 00-teensy.rules”
        5.4) In the text editor of your choice, copy the entire file linked above and paste it into the file. Reflash dRehmFlight and the upload should complete
    6) Enjoy! 

### Flight Controller Setup Steps 
TODO

## Hardware Setup
