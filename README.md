# Pico TinyUSB i2s Speaker

A USB speaker firmware for the RP2040/Pico using TinyUSB and i2s.

## Updating the firmware for the board

Push the volume button in for 2 seconds and hold.

This will put the audio board into bootloader mode.

If you're using Recalbox, you may see a pop up that states a new USB device named RPI-RP2 has been discovered and asks you if you wish to initialise.  You can ignore this screen.

If you don't have any other external USB devices plugged in, you should be able to access the bootloader at:

/recalbox/share/externals/usb0

If you have more than one device plugged in, check the USB folders for two files in here - one called INDEX.HTM and one called INFO_UF2.TXT

The contents of INFO_UF2.TXT should be something like:

UF2 Bootloader v3.0
Model: Raspberry Pi RP2
Board-ID: RPI-RP2

Using scp or windows explorer, you should be able to drag/drop the uf2 file from this repo there.  The audio board will apply the update and then restart itself.

Check the releases page here for the latest link:

https://github.com/pimoroni/picade-max-audio/releases

If you want to double check, ssh to the picade and run

mount

You're looking for an entry like this one - if you have a USB drive or other devices are in bootloader mode you might see more than one listed here.

/dev/sda1 on /recalbox/share/externals/usb0 type vfat (rw,sync,nodev,noexec,noatime,nodiratime,fmask=0022,dmask=0022,codepage=437,iocharset=ascii,shortname=mixed,errors=remount-ro)
