# ADF4351 USB Control

This is an alternative firmware for the ADF4351 boards on ebay and similar websites, that ship with a USB connector that isn't functional on the default firmware.

This firmware enables the USB port in two modes: direct register programming (ie. passthrough), and frequency command.

NB: The screen and buttons often found on these boards is not enabled, so you will lose local control of the ADF4351.

## Firmware

The firmware for the device, using [libopencm3](https://libopencm3.org/).

This firmware can be flashed onto the device via the SWD header, using an ST-LINK programmer.

The firmware currently assumes a 100MHz crystal oscillator for the onboard register calculation routines.

Compile: `cd firmware/ && ./build`

Flash: `cd firmware && make stlink-flash`

## Tools

Example applications for controlling the device over USB. These are minimal terminal applications for use on Linux.

The communication uses USB Control Transfers, implemented in these applications with libusb.

## Authors

* Phil Crump <phil@philcrump.co.uk>

The ADF4351 Register Calculation code is derived from the Analog Devices example at: [github.com/analogdevicesinc](https://github.com/analogdevicesinc/linux/blob/xcomm_zynq/drivers/iio/frequency/adf4350.c)
