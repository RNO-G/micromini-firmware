/** Linker memory map, shared between application and bootloader */

MEMORY
{

  /* 8 kB for the bootloader */
  bootrom  (rx)  : ORIGIN = 0x00000000, LENGTH = 0x0002000

  /* The rest for the application */
  rom (rx)        : ORIGIN = 0x00002000, LENGTH = 0x003c000

  ram       (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00008000
}


__boot_rom_start__ = ORIGIN(bootrom);
__boot_rom_size__ = LENGTH(bootrom);

__rom_start__ = ORIGIN(rom);
__rom_size__ = LENGTH(rom);

__ram_start__ = ORIGIN(ram);
__ram_size__ = LENGTH(ram);


