# Introduction
This project is forked from https://github.com/sh123/esp32_loraprs to reduce it to a simple LoRa APRS beacon and tracker, but adding a GPS module.

In this configuration, it reads the GPS coordinates (latitude and longitude) and sends them to a LoRa APRS digipeater with the following settings:

- #define CFG_LORA_FREQ         433.775e6 
- #define CFG_LORA_BW           125e3
- #define CFG_LORA_SF           10
- #define CFG_LORA_CR           5
- you can change them in the file https://github.com/guido57/esp32_loraprs_gps/config.h, according to the settings of the digipeater you want to transmit to.


# Wirings
![alt text](images/ESP32-SX1278-GPS.PNG)

# Prototype

See the 432 MHz antenna connection on the left and the small GPS antenna on the right
![alt text](images/esp32_loraprs_gps.PNG)

# About the fork from the original project

Comparing with https://github.com/sh123/esp32_loraprs which has multiple functions, this esp32_loraprs_gps reduced to a simple beacon and tracker, but with GPS.
For this reason, the functions: 
- LoRa APRSDroid KISS Bluetooth modem
- LoRa APRS-IS RX/TX iGate server over WiFI
- digipeater
- Codec2 DV modem

are disabled in config.h, but can be enabled again (anyhow, I didn't try!)  

- For the original project esp32_loraprs description, please, visit the Wiki at https://github.com/sh123/esp32_loraprs/wiki
- For discussions, visit https://github.com/sh123/esp32_loraprs/discussions

# Further implementations

## This ESP32 SX1276 NEO6M module from LILYGO  
![alt text](http://www.lilygo.cn/prod_view.aspx?TypeId=50044&Id=1317&FId=t3:50044:3) 
would appear to cover perfectly the hardware needs of this software, apart from the display, not implemented at the moment.
I ordered it on ![Bangood](https://it.banggood.com/LILYGO-TTGO-Meshtastic-T-Beam-V1_1-ESP32-433-or-915-or-923Mhz-WiFi-Bluetooth-ESP32-GPS-NEO-6M-SMA-18650-Battery-Holder-With-OLED-p-1727472.html?rmmds=myorder&cur_warehouse=CN&ID=513922) and I'll update these notes when tested.

## Add a display

I'll add a display to show the incoming packet from the digipeaters that received the trasmitted packets ofr from other LoRa peers in the surroundings.



