MTB-RC – MTBbus RailCom detector
================================

MTB-RC is a RailCom detector sending addresses of a detected mobile decoders
via MTBbus.

## Features

* 8 track sections.
* [MTBbus v4](https://mtb.kmz-brno.cz/) interface.
* Reading DCC addresses of mobile decoders.
* Galvanic separation of MTB & DCC.
* Single STM32F103 main MCU.
* MTB part power input: 8-20 V.
* DCC part powered directly from DCC.
* Several IO & power protections (ESD, overvoltage, switched polarity, ...).
* 4->1 track time multiplexing.
* Opensource & openhardware design.

## PCB

See [README in `pcb` subfolder](pcb/README.md).

## Firmware

See [README in `fw` subfolder](fw/README.md).

## Authors

* Kateřina Hanáková
* [Jan Horáček](mailto:jan.horacek@kmz-brno.cz)

## Licence

See README in subfolders for licenses regarding pcb design and firmware.
