## Additional information
- Additional information about this project and it's progression (incl war stories!)
  is found here: https://surge-lo.atlassian.net/wiki/spaces/PROJECTPRE/pages/524289/
- A presentation of a reverse engineering project I did is found here:
  https://surge-lo.atlassian.net/wiki/spaces/PROJECTPRE/pages/65555/

## Note:
This is an early version of this project that flyes well, especially in acro mode.
Additional stuff like gps (and better auto levelling) have been made however it's not
flight ready so it's not being uploaded (to avoid crashed drones or safety hazards :)
This project was initially built for another embedded processor.
Some of the capabilities from that version have not yet been ported (if you wonder about empty code files).

One example is found in hardware_drivers/stream_flash_system/ This
is not included in compilation. However it can be ported and utilized.

## Folders
- arm_drivers/ Many drivers written almost from or from scratch for the stm32f411
  using the reference manual only.
- arm_essentials/ files required to compile
- hardware_drivers/ Drivers written mostly from scratch for external sensors ect
  using using reference manuals only.
- flight_controller/ Main code that makes the drone fly
- main.c Flight loop
- startup_stm32*.s/c files required for booting the discovery board
  and activating the external oscillator.

## Coding style
- Made for quick prototyping.
  There will be a lot of defines to turn modes on and off. This is very useful when turning
  stuff on and off while out flight testing the drone and reuploading stuff.

## Led menu drone values
- You can use the led menu with a hitachi LCD panel and buttons to set drone
  settings during start up (PID values). Right now the menu is not activated
  in the program (as it's faster to upload the software anew when out flying),
  however it is very easy to implement in the main loop.

## Compiling
- Use make. Install arm compiler first.
- https://www.instructables.com/Build-a-Program-for-STM32-MCU-Under-Linux/

## Interacting with the dev board
- Download Stlink
- Install openocd

## Makefile
- make all will compile and upload to board.
- Make sure the dev board is loaded.

## Compiler warnings
- There are some compiler warnings however
  they are from code from the official STM libs
  or from warnings about unused functions and vars.
