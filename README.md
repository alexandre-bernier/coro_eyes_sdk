# coro-eyes-sdk
SDK for a structured light 3D vision system using the TI LightCrafter 4500 EVM and Flir Flea3 cameras.

## Installation

### Building from Source

#### Dependencies
- [OpenCV4](https://opencv.org/)

		sudo apt install libopencv-dev

- [hidapi](https://github.com/libusb/hidapi)

		sudo apt install libhidapi-dev

- [libusb-1.0](https://libusb.info/)

		sudo apt install libusb-1.0-0-dev

- [libudev](https://manpages.debian.org/testing/libudev-dev/libudev.3.en.html)

		sudo apt install libudev-dev

### udev
Once udev is installed, you can run the [udev installation script](install/install_udev_rule.sh) with sudo to create the udev rules automatically.

		sudo ./install_udev_rule.sh
