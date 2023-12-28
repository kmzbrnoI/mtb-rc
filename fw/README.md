MTB-RC Firmware
===============

This repository contains firmware for main MCU STM32F103 of MTB-RC module written
in C99.

## Compiling

Requirements:
* `arm-none-eabi-gcc`
* STM32CubeF1 MCU Firmware Package v1.8.5
  - Download package
  - Set `STM32_CUBE_PATH_185` environment variable to the location of downloaded package
  - See `Makefile`
* `make`

Howto compile:

```bash
$ make
```

## Flashing

Requirements:
* `st-flash` (via STlink)

MTB-RC module contains programming connector. Use STlink to program the MCU.
```bash
$ make flash_stlink
```

## Debugging

Requirements:
* `openocd`
* `arm-none-eabi-gdb`

Howto debug:

```bash
$ openocd
$ arm-none-eabi-gdb build/mtb-rc.elf
(gdb) target extended-remote :3333
(gdb) b main
```

## License

This application is released under the [Apache License v2.0
](https://www.apache.org/licenses/LICENSE-2.0).
