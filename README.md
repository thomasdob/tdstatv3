# TDstatv3
A USB-controlled potentiostat/galvanostat for thin-film battery characterization

## Introduction
This repository contains all the necessary design files to build your own USB-controlled potentiostat/galvanostat.
These files are intended as supplementary information accompanying the whitepaper at https://arxiv.org/pdf/1701.07650.pdf.

## Repository contents

### Directories
* `kicad`: KiCad design files (schematic diagram and PCB layout).
* `firmware`: Source code and compiled firmware for the PIC16F1459 microcontroller. Uses Microchip's XC8 compiler.
* `python`: Contains `tdstatv3.py`; run this file with Python 2.7 to bring up a GUI measurement tool.
* `gerber`: PCB design files in Gerber format, the universal standard for PCB manufacturing.
* `datasheets`: Datasheets in pdf format for the integrated circuits used in this design.
* `drivers`: Libusb drivers for Windows (not necessary on other operating systems).

###Files
* `full_schematic.pdf`: Complete schematic diagram in pdf format.
* `parts_list.txt`: Complete parts list, useful when ordering components.
* `pcb_3dview.png`: 3D renders of the PCB design, useful for verification of the PCB layout.
* `pcb_fabrication_diagram.pdf`: Front side of PCB with component locations and values, useful for circuit assembly.
* `photograph.jpg`: A photograph of the assembled circuit, using a PCB manufactured by [OSH Park](https://oshpark.com/).
* `README.md`: This file.

## USB access on Linux
In order to access the device without requiring root privileges, create a file
`/etc/udev/rules.d/99-tdstatv3.rules` containing the line

```
SUBSYSTEM=="usb", ATTRS{idVendor}=="a0a0", ATTRS{idProduct}=="0002", GROUP="plugdev", MODE="0666"
```
This assumes that the current user is a member of the `plugdev` group, and that the default USB Vendor and Product ID's
as coded in the microcontroller firmware are used; if not, these values need to be adjusted.

## Credits

* Thomas Dobbelaere
* email: thomas.dobbelaere@ugent.be
