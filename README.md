FPGA Blit
===========

This is an FPGA implementation of the Blit terminal (the original 68K version, not the DMD5620).
It's only meant to be "software-compatible" and not an accurate reproduction of the original hardware (which is hard without schematics).
It's based on the games/blit emulator shipped with 9front (which is itself based on reverse-engineering the register locations by reading the Blit source code in the V8 release).

The code in the root directory is meant to be independent of the hardware, although it may have to be adjusted because not all details will work on all platforms (e.g. it uses block RAMs for the 256KB RAM).
The `aijubrd` directory contains an implementation for [aijuboard](http://aiju.de/electronics/aijuboard/), which uses DisplayPort for output (padding the 800x1024 image from the blit to 1280x1024) and handles the keyboard, mouse and UART interfaces from Plan 9.

The `aijubrd` directory has files `top.v` and `top.xdc` which are generated using the [`vlt` tool](https://github.com/aiju/hdl/tree/master/tools/vlt).
