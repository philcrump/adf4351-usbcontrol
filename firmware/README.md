# ADF4351-usbcontrol firmware

## Dependencies

* arm-none-eabi toolchain: https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
  * Download and extract, eg. to `~/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/`
  * Add to path, eg. append to _~/.bashrc_ : `PATH=$PATH:~/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin/`
* stlink-tools
  * `sudo apt install stlink-tools`
  * The board may require unplugging and replugging if it is already plugged in.

## Compile

```bash
# Compile
./build
# Flash board with STLINK
make stlink-flash
```

## Authors

* Phil Crump <phil@philcrump.co.uk>
