#!/bin/bash

set -ex

# clean
rm -f rgatest frame.1.tga

# build
gcc -I /usr/include/libdrm -I /usr/local/include/libdrm/ -o rgatest rgatest.c -ldrm -ldrm_rockchip

# execute
./rgatest

# show result (eog is nice tool to show tga images)
#eog frame.1.tga
