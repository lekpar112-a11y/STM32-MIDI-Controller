#!/bin/sh
echo "This helper will try to download required ST USB Device files into src/usb_st/"
echo "You must supply raw URLs for usbd_midi_if.c, usbd_midi_if.h, usbd_conf.c, usbd_desc.c, usbd_desc.h"
echo "Example usage:"
echo "sh fetch_usbstub.sh <url_usbd_midi_if.c> <url_usbd_midi_if.h> <url_usbd_conf.c> <url_usbd_desc.c> <url_usbd_desc.h>"
if [ "$#" -lt 5 ]; then
  echo "Not enough arguments."; exit 1
fi
mkdir -p src/usb_st
curl -L -o src/usb_st/usbd_midi_if.c "$1"
curl -L -o src/usb_st/usbd_midi_if.h "$2"
curl -L -o src/usb_st/usbd_conf.c "$3"
curl -L -o src/usb_st/usbd_desc.c "$4"
curl -L -o src/usb_st/usbd_desc.h "$5"
echo "Downloaded files to src/usb_st/. Now try: platformio run"
