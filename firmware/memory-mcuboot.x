MEMORY
{
  /* slot0 of mcuboot */
  FLASH : ORIGIN = 0xC000, LENGTH = 0x67000
  /* FLASH   : ORIGIN = 0, LENGTH = 1024K */
  CODERAM : ORIGIN = 0x00800000, LENGTH = 64K
  RAM     : ORIGIN = 0x20000000 + 64K, LENGTH = 256K - 64K
}
