#!/bin/bash

apt update
apt install -y build-essential cmake git libusb-dev libusb-1.0-0-dev

cd /tmp

git clone https://github.com/janspeller/librtlsdr.git
cd librtlsdr

mkdir build && cd build
cmake ../ -DINSTALL_UDEV_RULES=ON -DDETACH_KERNEL_DRIVER=ON
make

make install
ldconfig
