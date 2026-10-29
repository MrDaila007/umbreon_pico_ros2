ENTRY(Reset_Handler)
MEMORY {
    FLASH (rx) : ORIGIN = 0x10000000, LENGTH = 4M
    RAM (rwx) : ORIGIN = 0x30000000, LENGTH = 32K
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS {
    .text : { KEEP(*(.vectors)) *(.text*) *(.rodata*) } > FLASH
    .ARM.exidx : { *(.ARM.exidx*) } > FLASH
    .data : { . = ALIGN(4); _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > RAM AT> FLASH
    _sidata = LOADADDR(.data);
    .bss (NOLOAD) : { . = ALIGN(4); _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM
    end = .; _end = .; __end__ = .;
}
