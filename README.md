# Daisy.Patch_sm
Patch and utilities for Daisy Patch submodule and in particular for Patch.init() eurorack module.

I plan to store all my files here for you to reuse. Just create an empty project with same name as the file here on GIthub with VScode. Drop the file in the VScode directory,  compile and flash  the Paztch.init().

* Waveshaper : this is the final version. this waveshaper needs an SPI Oled and a quadrature encoder + SD card to work. For compiling it : add ```APP_TYPE = BOOT_SRAM``` and ```USE_FATFS = 1``` to the Makefile
* Gerber.zip : Gerber files for patch.init() front panel
* Clocked chaotic LFO. NB: This LFO needs an Expert Sleepers FHX-8CV module
* Clocked LFO. NB: This LFO needs an Expert Sleepers FHX-8CV module
