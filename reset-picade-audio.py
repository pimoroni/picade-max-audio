import glob
import serial

picade = glob.glob("/dev/serial/by-id/usb-Pimoroni_Picade_USB_Audio_*")[0]

device = serial.Serial(picade)

device.write(b"multiverse:_usb")