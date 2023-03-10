## Additional information
- Additional information about this project and its progression (incl war stories and a flight video!)
  is found here: https://surge-lo.atlassian.net/wiki/spaces/PROJetcPRE/pages/524289/
- A presentation of a reverse engineering project I did is linked here:
  https://surge-lo.atlassian.net/wiki/spaces/PROJetcPRE/pages/65555/

## Note:
This repo is an early version of this project that flies well, especially in acro mode (when you
control the drone's angular velocity for each axis with the remote controller).
Other things like GPS (and better auto leveling) have been programmed. However, it's not
flight ready so it's not being uploaded (to avoid crashed drones and safety hazards :)
This project was initially built for another embedded processor.
Some of the capabilities from that version still need to be ported (if you wonder about empty code files).

One example is found in hardware_drivers/stream_flash_system/. This
is not included when compiling. However, it can be ported and utilized.

## Folders
- `arm_drivers/` Many drivers written almost from or from scratch for the stm32f411
  using the reference manual only.
- `arm_essentials/` Files required to compile.
- `hardware_drivers/` Drivers written mostly from scratch for external sensors etc. using reference manuals only.
- `flight_controller/` Main code that makes the drone fly
- `main.c` Flight loop.
- `startup_stm32*.s/c` Files required for booting the discovery board
  and activating the external oscillator.

## Coding style
- Made for quick prototyping.
  There will be a lot of defines to turn modes on and off. This is very useful when turning
  things on and off while out flight testing the drone and needing to upload changes to settings.

## Led menu drone values
- You can use the led menu with a Hitachi LCD panel and buttons to set drone-settings during start-up (PID values). Right now, the menu is not activated
  in the program (as it's faster to upload the software anew when out flying),
  however, it is straightforward to implement in the main loop.

## Compiling
- Use make. Install arm compiler first.
- https://www.instructables.com/Build-a-Program-for-STM32-MCU-Under-Linux/

## Interacting with the dev board
- Download Stlink
- Install openocd

## Makefile
- make all will compile and upload to the board.
- Make sure the dev board is loaded.

## Compiler warnings
- There are some compiler warnings however
  they are from code from the official STM libs
  or from warnings about unused functions and vars.
