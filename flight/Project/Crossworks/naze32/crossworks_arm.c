#include <pios.h>
#include <pios_board.h>

#include "pios_board_info.h"

#define BOARD_TYPE          0x04
#define BOARD_REVISION      0x02
#define BOOTLOADER_VERSION  0x03
#define HW_TYPE             0x01

#define FW_BANK_BASE        0x08000000  // Start of firmware flash
#define FW_BANK_SIZE        0x0001C000  // Should include FW_DESC_SIZE
#define FW_DESC_SIZE        0x00000064

const struct pios_board_info /*__attribute__((__used__)) __attribute__((__section__(".boardinfo")))*/ pios_board_info_blob = {
  .magic      = PIOS_BOARD_INFO_BLOB_MAGIC,
  .board_type = BOARD_TYPE,
  .board_rev  = BOARD_REVISION,
  .bl_rev     = BOOTLOADER_VERSION,
  .hw_type    = HW_TYPE,
  .fw_base    = FW_BANK_BASE,
  .fw_size    = FW_BANK_SIZE - FW_DESC_SIZE,
  .desc_base  = FW_BANK_BASE + FW_BANK_SIZE - FW_DESC_SIZE,
  .desc_size  = FW_DESC_SIZE,
};

// ---------------------------------------------------------

typedef struct _hardfault_args_t {
  unsigned int r0;
  unsigned int r1;
  unsigned int r2;
  unsigned int r3;
  unsigned int r12;
  unsigned int lr;
  unsigned int pc;
  unsigned int psr;
} hardfault_args_t;

void hard_fault_handler_c (hardfault_args_t* hardfault_args)
{
   for(;;) {
   }
}

void HardFault_Handler(void) __attribute__((naked));
void HardFault_Handler(void)
{
    __asm("TST LR, #4\r\n"
        "ITE EQ\r\n"
        "MRSEQ R0, MSP\r\n"
        "MRSNE R0, PSP\r\n"
        "B hard_fault_handler_c");
}
