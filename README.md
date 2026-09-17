# samd5-boot - a dual-bank bootloader for ATSAMD/E5x series microcontrollers.

The flash of samd5x is divided into 2 equal banks, which can be mapped into the main address space in either order depending on the value of the STATUS.AFIRST fuse. Use of this feature for safe flash updates is outlined in section [25.6.7 of the datasheet](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU32/ProductDocuments/DataSheets/SAM-D5x-E5x-Family-Data-Sheet-DS60001507.pdf#_OPENTOPIC_TOC_PROCESSING_d99375e230392). This bootloader is an implementation of this procedure, with pluggable transport, image verification using DSU checksum and rollback-on-failure using the WDT.

## Usage
Write your application as normal, and also a small bootloader bin using `samd5-boot`. This will flash to the beginning of each bank, and handle the download of your application over whichever transport peripheral you chose to plug in.
