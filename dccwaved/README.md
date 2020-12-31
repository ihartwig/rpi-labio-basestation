DCC Waveform Driver
===================

Work In Progress - many features not complete

This driver for the Raspberry Pi (tested on 3B+) implements a linux driver to
generate NMRA DCC compliant output waveforms for controlling a model railroad.
The PWM serializer and DMA components are used to ensure DCC pulse timing is
stable across software and network loads on the Pi.

### Hardware Support

This driver is designed to run on RPi LabIO expansion board, but it is adaptable
to any Raspberry Pi with a motor controller board that can meet the following:

* Supports the current required for your model railroad.
* DIR pin(s) must use a GPIO with a PWM Alt Mode available.

For example, these [Pololu RPi motor boards](https://www.pololu.com/product/2755)
could be used.

### References

The template of this driver borrows from [ServoBlaster by Richard Hirst](https://github.com/richardghirst/PiBits/tree/master/ServoBlaster)
including the register and memory map interface.

* [Using PWM to generate DCC signal](https://github.com/keybuk/SignalBox/wiki/Using-PWM-to-generate-DCC-signal)
* [NMRA DCC Packet Format](https://dccwiki.com/Digital_packet)
* [BCM2835 ARM Peripherals Programming Guide](https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf)
* [BCM2835 GPIOs (with Alt Modes)](https://elinux.org/RPi_BCM2835_GPIOs)
