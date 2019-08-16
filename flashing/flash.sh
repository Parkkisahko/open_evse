#!/bin/sh
avrdude -c avrispmkII -p m328p -U lfuse:w:0xFF:m -U hfuse:w:0xDE:m -U efuse:w:0x05:m
avrdude -c avrispmkII -p m328p -U flash:w:open_evse.ino.with_bootloader.standard.hex