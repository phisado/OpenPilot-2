#include <pios.h>
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
