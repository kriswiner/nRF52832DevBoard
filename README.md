# nRF52832DevBoard

This is a collection of sketches that can be run on the nRF52832 Development Board available [here](https://www.tindie.com/products/onehorse/nrf52832-development-board/). The sketches use Sandeep Mistry's [Arduino core](https://github.com/sandeepmistry/arduino-nRF5) and BLEPeripheral API to read sensor data and display it to smart devices via services and characteristics. The development board can be flashed using the J-Link debugger found on the nRF52 DK. This is not as convenient as programming via the USB connector but this is the easiest way to start using BLE I have found.

![nRF52823devboard](https://d3s5r33r268y59.cloudfront.net/44691/products/thumbs/2016-08-30T21:54:55.012Z-nRF52DevBrd.top.jpg.2560x2560_q85.jpg)

The board has a MAX1555 battery charger and a USB Micro-B connector for charging only; the nRF52 has no USB controller. There is a NCP161 450 mA LDO 3V3 voltage regulator providing plenty of current to the board for adding sensors, SD cards, and displays, etc.

There is an MPU9250 9 DoF motion sensor and a BMP280 altimeter on the board connected to nRF52 pins 6 and 7 for SDA/SCL, which are also routed to edge pins. The board breaks out all of the GPIOs except pin 10 (used for the MPU9250 interrupt), and pins 22, 23 and 24 (for the rgb led).
