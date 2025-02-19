# High-performance PIO-based driver for ADS124S0X on Zephyr RTOS

As the name indicates, this driver works for **RP2040** and other RP family
microcontrollers. The niche I'm exploring here is to use ADS124S0X as a serious
affordable data acquisition (DAQ) device to perform some semi-demanding data
collection tasks on a low budget.

The pathway to DAQ includes making the sampling process deterministic with
little timing jitter, and minimizing the burden on the CPU. That means some
hardware level support for the communication protocol and DMA is essential.
Those happen to be available in a very handy way on RP2040 and its relatives.

This driver makes it possible (at the cost of a PIO state machine and 5 DMA
    channels) to almost completely offload the channel configuring
(including multiplexing, of course) and sample reading to a PIO state machine
by monitoring the DRDY pin. The samples are saved to the buffers on RAM via
DMA, with the buffer status communicated to user code through DMA interrupts.

The combination with Zephyr greatly expands the possibility of integrating with
a rich collection of high-quality subsystems and device drivers (e.g. Ethernet,
        etc.).

This work is an enhancement to the in-tree [ADS124S0X
driver](https://docs.zephyrproject.org/latest/build/dts/api/bindings/adc/ti%2Cads124s06.html)
(which I also had the honor to contribute to).

A prototype of this driver, relies solely on the pico-sdk is available
separately at [ADS124S0X-PIO](https://github.com/TerryGeng/ADS124S0X-PIO).
Please refer to this for explanations on the PIO design behind.

## How to use

This repo includes an example in the `app/` folder that is designed to work
right out of the box with a Raspberry Pico board with the following wiring:
 - `SCK`: GPIO 2,
 - `TX`: GPIO 3,
 - `RX`: GPIO 4,
 - `CS`: GPIO 5,
 - `DRDY`: GPIO 6.

The example program will first set the GPIO pin 0 on the ADS124S06 high, then
read channel 0 and 1, 2 and 3, 4 and 5, differentially in a cyclic way upon
user's request.

Please change the device tree bindings in `app/boards/rpi_pico.overlay` if you
have a different wiring or ADC channel settings.

Users can interact with this program via the UART on GPIO 0 and 1 (though UART
via USB console is easily available by compiling with the [cdc-acm-console
snippet](https://docs.zephyrproject.org/latest/snippets/cdc-acm-console/README.html)).

The shell subsystem is enabled and has been configured with two commands
 - `acq start`: Start acquisition, read channel 0 and 1, 2 and 3, 4 and 5,
 differentially and log the result to the console.
 - `acq stop`: Stop acquisition.
