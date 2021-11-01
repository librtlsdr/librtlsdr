# Description

rtl-sdr turns your Realtek RTL2832 based DVB dongle into a SDR receiver

# Build / Install (on debian/ubuntu)

## TL;DR
just execute this
```
wget -O - https://raw.githubusercontent.com/janspeller/librtlsdr/master/install.sh | bash
```

and for proxmox usb passthrough add this to the container config (eigther mount specific usb bus or everything)
```
lxc.cgroup2.devices.allow: c 189:* rwm
lxc.mount.entry: /dev/bus/usb/001 dev/bus/usb/001 none bind,optional,create=dir
```

## prerequisites
development tools have to be installed:
```
sudo apt-get install build-essential cmake git
```

install the libusb-1.0 development package::
```
sudo apt-get install libusb-dev libusb-1.0-0-dev
```

## retrieve the sources - right branch

```
git clone https://github.com/janspeller/librtlsdr.git
```

by default, you should have the *master* branch, in doubt:
```
cd librtlsdr
git status
git checkout master
```

run cmake and start compilation with the required options
```
mkdir build && cd build
cmake ../ -DINSTALL_UDEV_RULES=ON -DDETACH_KERNEL_DRIVER=ON
make
```

## install
setup into prefix, usually will require `sudo`:
```
sudo make install
sudo ldconfig
```
