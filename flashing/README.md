# Flashing OpenEVSE
OpenEVSE's come from factory without support for zero current while EVSE is activated, nor a bootloader. To function with Parking Energy system, support for zero current needs to be added, and a bootloader so that the boards may be updated OTA. We use basic Arduino bootloader.

## Prerequisites
- Avrdude installed
- A programmer f.ex. AVRISP mkII (https://www.microchip.com/DevelopmentTools/ProductDetails/PartNO/ATAVRISP2)
- Programmer connected to OpenEVSE's ISP1 pins (Should show green when connected righ), connection diagram found in this directory ```OpenEVSE_v5.pdf```
- OpenEVSE needs to be powered up, a propotion for future is to design a board that simply feeds 5 V to OpenEVSE via progrmamming pins while programming

## Using different programmer
- Edit ```flash.sh```, replace avrispmkII with the correct programming device

## Flashing
- Run ./flash.sh
- Verify that avrdude says everything is ok.

## Flashing OTA
- Avrdude needs to be installed on device
- Client-amy needs to be shut down to not disturb serial data
- We use a serial connection wich connects to Arduino bootloader for flashing
- Command: ```avrdude -p atmega328p -c arduino -P /dev/ttyUSB0 -b 115200 -D -U flash:w:open_evse.ino.standard.hex```, where the file name and port may have to be changed.