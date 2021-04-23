# SimpleStm32F7Template
This is an example of a simple project template for the STM32F7 with full support for hardfloat, LwIP, and FreeRTOS.

# Build instructions
(Tested on Ubuntu 18.10)
* $ cd ~/
* $ mkdir opt
* $ cd opt
* $ wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/8-2018q4/gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2
* $ tar -xvfz gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2
* [Add gcc-arm-none-eabi-8-2018-q4-major/bin to your PATH however you see fit]
* $ cd [location of your repo]/boreas-atse/SimpleStm32F7HFTemplate
* $ mkdir build
* $ cd build
* $ cmake ..
* $ make

