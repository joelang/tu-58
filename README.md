# tu-58
DEC TU-58 emulator

This is an emulator for a DEC TU-58 block addressable tape system. 

It is wrtten in AVR assembler. 

This project has been ongoing for many years. On the back burner. A few years back
I got the rom image from a reat TU-58. I disassembled and commented the code.
(It's on bitsavers) That gave me an outline for my AVR code.

The storage is a single CF card that supports multiple images of 65536 512 byte blocks.
The number of images is limited by the RSP protocol UNIT being a byte (256 UNITS)
You may run out of CF card storage space before that limit is reached.
This is far more storage than a real TU-58 could support. 

Some things are different from a real TU-58:

BLOCKS greater than 512 supported

UNITS greater than 2 supported

No capstan roller to turn to goo

Boot switch not implemented

Special addressing mode not implemented.

The emulator has been tested with RT-11 and XXDP test CZTUU.


The files:

tu58-rom.lst --- dissassembly listing of DEC TU-58 ROM

tu-58.asm --- assembler source code (avr studio 6.2)

shifter.abl --- ABEL source for GAL16V8

tu-58.pdf --- schematic

I have done a PCB layout and had prototypes run (OSHPARK)
