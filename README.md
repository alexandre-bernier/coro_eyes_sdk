# coro_eyes_sdk

## Overview

SDK for a structured light 3D vision system using the TI LightCrafter 4500 EVM and Flir Flea3 camera(s) to generate point clouds.

![](doc/coro__eyes__sdk_8h__incl.png)

### License

The source code is released under a [BSD 3-Clause license](coro_eyes_sdk/LICENSE).

<b>Author: Alexandre Bernier<br />
Affiliation: [CoRo, ÉTS](http://en.etsmtl.ca/unites-de-recherche/coro/accueil?lang=en-CA)<br />
Maintainer: Alexandre Bernier, ab.alexandre.bernier@gmail.com</b>

### Compatibility

The coro_eyes_sdk has been tested on Ubuntu 20.04 with the following versions of dependencies:

| Dependencies | Version |
| ------------ | :-----: |
| OpenCv | 4.2.0 |
| hidapi | 0.9.0 |
| libusb | 1.0.23 |
| udev | 245.4 |

## Installation

### Building from Source

#### Dependencies
- [OpenCV](https://opencv.org/)

      sudo apt install libopencv-dev

- [hidapi](https://github.com/libusb/hidapi)

      sudo apt install libhidapi-dev

- [libusb-1.0](https://libusb.info/)

      sudo apt install libusb-1.0-0-dev

- [libudev](https://manpages.debian.org/testing/libudev-dev/libudev.3.en.html)

      sudo apt install libudev-dev
    
## Usage

### udev

Once udev is installed, you can run the [udev installation script](install/install_udev_rule.sh) with sudo to create the udev rules automatically.

    sudo ./install_udev_rule.sh

### Header

Simply include coro_eyes_sdk.h in your code to get access to all the APIs.

### Library

You will need to build coro_eyes_sdk library and link it in your project.
