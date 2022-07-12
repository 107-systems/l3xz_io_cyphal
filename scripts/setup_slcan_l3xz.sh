#!/bin/bash
cd $(dirname $(readlink -f "$0"))
sudo ./setup_slcan.sh --remove-all --basename can --speed-code 8 \
  /dev/serial/by-id/usb-Zubax_Robotics_Zubax_Babel_2300390005514E433734382000000000-if00
