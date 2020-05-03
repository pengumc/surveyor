#! /bin/sh
avrdude -p m328p -c linuxspi -P /dev/spidev0.0 -U flash:w:surveyor.hex:i
