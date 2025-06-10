## Overview
dRehmFlight is the flight controller for hobbyists, hackers, and non-coders interested in stabilizing their wacky and unique flying creations. The code and supporting documentation is built to bring someone up to speed on VTOL flight stabilization concepts as quickly and painlessly as possible. The code is written and presented in a way that is intuitive, easy to follow, and most importantly: all in one place. No more digging through countless folders and confusing classes just to add an 'if' statement for your custom drone application. This flight controller uses an Arduino-compatible microcontroller, so there is no confusing flashing or compiling process necessary. If you can use Arduino, you can start expanding the capabilites of this flight controller to your liking.

dRehmFlight has been used as a teaching tool for aircraft stabilization and flight control principles in universities and tech companies around the world. It is not meant to out-perform other flight controller packages on the market, or be used in a commercial sense. It is best suited for rapid prototyping or allowing a radio control hobbyist to get their feet wet in flight control code for their VTOL project. Much more information is included in the dRehmFlight VTOL Documentation .pdf.

This code is entirely free to use and will stay that way forever. If you found this helpful for your project, donations are appreciated: [Paypal Donation](https://www.paypal.me/NicholasRehm)

### Hardware Requirements
This flight controller is based off of the Teensy 4.0 microcontroller and MPU6050 6DOF IMU. The following components (available on Amazon) are required to complete the flight controller assembly:

**Teensy 4.1**: https://amzn.to/3oFG3QN

**GY-521 MPU6050 IMU**: https://amzn.to/3edF1Vn

These (and all Amazon links contained within the supporting documentation) are Amazon Affiliate links; by purchasing from these, I receive a small portion of the revenue at no cost to you. I appreciate any and all support! [Buy all of the parts here.](https://www.amazon.com/shop/nicholasrehm/list/1NDPB7E0VMZOP?ref_=aip_sf_list_spv_ons_mixed_d)

### Software Requirments
Code is uploaded to the board using the Arduino IDE; download the latest version here: https://www.arduino.cc/en/main/software
To connect to the Teensy, you must also download and install the Teensyduino arduino add-on; download and instructions available here: https://www.pjrc.com/teensy/td_download.html

## Disclaimer
This code is a shared, open source flight controller for small micro aerial vehicles and is intended to be modified to suit your needs. It is NOT intended to be used on manned vehicles. I do not claim any responsibility for any damage or injury that may be inflicted as a result of the use of this code. Use and modify at your own risk. More specifically put:

THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
