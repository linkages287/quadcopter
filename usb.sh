#!/bin/sh -x
# usb.h

for X in /sys/bus/usb/devices/*; do
    echo "$X"
    cat "$X/idVendor" 2>/dev/null
    cat "$X/idProduct" 2>/dev/null
    echo done
done

for X in /sys/bus/usb/devices/*; do
    echo "$X"
    cat "$X/idVendor" 1>/dev/null
    cat "$X/idProduct" 1>/dev/null
    echo done
done

for i in /sys/bus/pci/drivers/[uoex]hci_hcd/*:*; do
  echo "${i##*/}" > "${i%/*}/unbind"
  echo "${i##*/}" > "${i%/*}/bind"
done
