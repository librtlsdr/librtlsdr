# Description

rtl-sdr turns your Realtek RTL2832 based DVB dongle into a SDR receiver

# Build / Install (on debian/ubuntu)

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
