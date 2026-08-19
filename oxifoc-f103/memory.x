MEMORY
{
    FLASH : ORIGIN = 0x08003800, LENGTH = 26200
    RAM   : ORIGIN = 0x20000000, LENGTH = 4096
    RETAIN : ORIGIN = 0x20004F00, LENGTH = 256
}

/* The resident bootloader zeroes 0x200000DC..0x20000CF7 before starting the
 * application. Keep reset forensics above both that range and the application's
 * deliberately conservative 4 KiB runtime RAM/stack region. This address also
 * stays within the 20 KiB SRAM range accepted by the updater validator. */
SECTIONS
{
    .retained (NOLOAD) : ALIGN(4)
    {
        KEEP(*(.retained .retained.*));
    } > RETAIN
}
INSERT AFTER .got;
