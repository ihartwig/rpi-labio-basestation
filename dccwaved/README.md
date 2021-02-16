DCC Waveform Driver
===================

Work In Progress - many features not complete

This driver for the Raspberry Pi (tested on 3B+) implements a linux driver to
generate NMRA DCC compliant output waveforms for controlling a model railroad.
The PWM serializer and DMA components are used to ensure DCC pulse timing is
stable across software and network loads on the Pi.

### Hardware Support

This driver is designed to run on [RPi LabIO](https://github.com/ihartwig/rpi-labio) 
expansion board with an L298P H-Bridge, but it is adaptable
to any Raspberry Pi with a motor controller board that can meet the following:

* Supports the current required for your model railroad.
* Supports switching speed of 20+kHz (for 58us DCC pulses).
* DIR pin(s) must use a GPIO with a PWM Alt Mode available.

For example, these [Pololu RPi motor boards](https://www.pololu.com/product/2761)
could be used.

The oscillator and derived clock speeds are different on Raspberry Pi 4 and
may require some adjustments to use this driver.

### Hardware Configuration

Our implementation of the L298P H-Bridge requires the RPi to generate opposing
waveforms on the inputs connected to `GPIO18`, `GPIO19`. We'll connect these to
`PWM0`, `PWM1` respectively. The PWM peripheral uses a single FIFO with data for
both channels interleaved, so as long as we set up the line code data correctly
we can guarantee the data streaming out of PWM0 and PWM1 are an exact copy.

When used in this interleaved mode, the "repeat last FIFO data" feature is not
available on the PWM seralizer output, so we need to reach a safe state on track
a different way than continuing to send preamble bits. By setting PWM1
polarity inverted but leaving the silence bit same as PWM0 the voltage seen at
a locomotive would be 0V (both GND in this case) on fault.

DCC requires a "1 bit" represented by a 58us (55-61us) high pulse followed by a
58us low. The "0 bit" definition is a bit more flexible with high / low pulses
of at least 95us. We'll use `10` to represent a "1 bit" and `1100` to represent
a "0 bit" for simplicity, giving the 0 bit a approx. 116us pulse.

At this slow of a bit rate, we need to drive the PWM clock from the Oscillator
(OSC) which runs at 19.2 Mhz. The max 4096 divider of the PLLD source, which
runs at 500 MHz, leaves us with a min pulse width that is too short. With a
`OSC Divl` of `1,113` we get a bit width of `57.969us`.

* [keybuk: Setting the Clock](https://github.com/keybuk/SignalBox/wiki/Using-PWM-to-generate-DCC-signal#setting-the-clock)
* [keybuk: DCC Bit Rate Table](https://github.com/keybuk/SignalBox/wiki/Clock-and-bit-lengths-for-DCC#bit-rate)

### References

The template of this driver borrows from [ServoBlaster by Richard Hirst](https://github.com/richardghirst/PiBits/tree/master/ServoBlaster)
including the register and memory map interface.

* [keybuck: Using PWM to generate DCC signal](https://github.com/keybuk/SignalBox/wiki/Using-PWM-to-generate-DCC-signal)
* [NMRA DCC General Packet Format (s-9.2)](https://www.nmra.org/sites/default/files/s-92-2004-07.pdf)
* [NMRA DCC Extended Packet Format (s-9.2.1)](https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf)
* [BCM2835 ARM Peripherals Programming Guide](https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf)
* [BCM2835 GPIOs (with Alt Modes)](https://elinux.org/RPi_BCM2835_GPIOs)
