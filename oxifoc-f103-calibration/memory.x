MEMORY
{
    FLASH : ORIGIN = 0x08003800, LENGTH = 26200
    RAM   : ORIGIN = 0x20000000, LENGTH = 4096
    RETAIN : ORIGIN = 0x20004F00, LENGTH = 256
}

/* Shared F103 reset forensics must remain outside both the resident
 * bootloader's zero-fill range and this application's 4 KiB runtime RAM. */
SECTIONS
{
    .retained (NOLOAD) : ALIGN(4)
    {
        KEEP(*(.retained .retained.*));
    } > RETAIN
}
INSERT AFTER .got;
