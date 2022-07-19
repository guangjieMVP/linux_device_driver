#!/bin/sh

make ARCH=arm -j4 CROSS_COMPILE=arm-linux-gnueabihf- dtbs
