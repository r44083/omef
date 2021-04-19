# omef
One more embedded framework

Omef was created in an attempt to standardize the low level periphery interface on the different development platform. It targets to resource-limited embedded projects. The main feature is providing one standard interface between different HW platforms.

## Goal
If you look around you could find some widely used HAL frameworks from hardware manufacturers, such as: *StdPeriph_Lib and HAL from ST, LPCOpen from NXP, Atmel Software Framework, Ameba SDK, ESP SDK etc.*

So, what's wrong with this situation?
- There isn't standard interface over the plenty of HW platforms (each of them uses its own interface)
- Each of them uses its own working principle (interrupts from peripheral, DMA, blocking polling)

Omef helps to avoid this. It provides transparent working principle and one interface (except init methods on some platforms, for example due to lack of DMA).

## Features
- simple API
- highly portable
- small code footprint
- OOP paradigm is used

## How to use
By default there is a placeholder with simple demo project for each HW platform.

Clone and build omef:
```
git clone --recursive https://github.com/yhsb2k/omef.git
cd omef
make
make flash
make debug
```
