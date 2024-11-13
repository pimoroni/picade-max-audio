#!/bin/bash
PRODUCT="picade-max-audio"
FIRMWARE_VERSION=$(curl -sSL https://api.github.com/repos/pimoroni/$PRODUCT/releases/latest | grep "tag_name" | cut -d \" -f4)
FIRMWARE_UF2="picade-max-audio.uf2"
FIRMWARE="https://github.com/pimoroni/picade-max-audio/releases/download/$FIRMWARE_VERSION/$PRODUCT-$FIRMWARE_VERSION.zip"

echo "Installing firmware: $FIRMWARE_VERSION."

SERIAL_DEVICE=$(find /dev/serial/by-id -name "usb-Pimoroni_Picade_USB_Audio_*")
DEVICE=$(blkid -L RPI-RP2)

if [[ -e "$SERIAL_DEVICE" ]]; then
    echo "Found Picade Max Audio at: $SERIAL_DEVICE"
    echo "multiverse:_usb" > "$SERIAL_DEVICE"
    DEVICE=""
    START=$EPOCHSECONDS
    while [[ -z "$DEVICE" ]]; do
        if (( EPOCHSECONDS-START > 10 )); then
            echo "RP2 bootloader device not found."
            exit 1
        fi
        sleep 1
        DEVICE=$(blkid -L RPI-RP2)
    done
    echo "Found RP2 on $DEVICE"
    MNT_TEMP=$(mktemp -d --suffix "$PRODUCT")
    sudo mount "$DEVICE" "$MNT_TEMP" # > /dev/null 2>&1
    FW_TEMP=$(mktemp --suffix "$PRODUCT")
    echo "Fetching $FIRMWARE..."
    wget -q "$FIRMWARE" -O "$FW_TEMP"
    echo "Installing firmware..."
    sudo unzip "$FW_TEMP" -d "$MNT_TEMP" "$FIRMWARE_UF2"
    rm "$FW_TEMP"
    sync
    sudo umount "$MNT_TEMP"
    rm -r "$MNT_TEMP"
else
    echo "Picade Max Audio not found?"
    exit 1
fi
