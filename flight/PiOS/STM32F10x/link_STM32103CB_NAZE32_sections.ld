/* This is the size of the stack for all FreeRTOS IRQs */
_irq_stack_size = 0x240;
/* This is the size of the stack for early init: life span is until scheduler starts */
_init_stack_size = 0x100;

/* Stub out these functions since we don't use them anyway */
PROVIDE (   vPortSVCHandler = 0 ) ;
PROVIDE (   xPortPendSVHandler = 0 ) ;
PROVIDE (   xPortSysTickHandler = 0 ) ;

/* Section Definitions */
SECTIONS
{
    .text :
    {
        PROVIDE (pios_isr_vector_table_base = .);
        KEEP(*(.isr_vector .isr_vector.*))
        *(.text .text.* .gnu.linkonce.t.*)
        *(.glue_7t) *(.glue_7)
        *(.rodata .rodata* .gnu.linkonce.r.*)
    } > FLASH

    /* module sections */
    .initcallmodule.init :
    {
        . = ALIGN(4);
	__module_initcall_start = .;
        KEEP(*(.initcallmodule.init))
	. = ALIGN(4);
	__module_initcall_end   = .;
    } >FLASH

    /* exti sections */
    _exti :
    {
        . = ALIGN(4);
    __start__exti = .;
        KEEP (*(_exti _exti.*))
    . = ALIGN(4);
    __stop__exti   = .;
    } >FLASH

    . = ALIGN(4);
    _etext = .;
    _sidata = .;

    /* fw_version_blob */
    _fw_version_blob = FW_BANK_BASE + FW_BANK_SIZE - FW_DESC_SIZE;
    .fw_version_blob _fw_version_blob : AT(_fw_version_blob)
    {
        KEEP(*(.fw_version_blob))
        . = . + 100 ;
    } >FLASH
    
    /* flashfs (reservation) */
    .flashfs EE_BANK_BASE (NOLOAD) : AT(EE_BANK_BASE)
    {
        . = . + EE_BANK_SIZE ;
    } >FLASH

    /*
     * This stack is used both as the initial sp during early init as well as ultimately
     * being used as the STM32's MSP (Main Stack Pointer) which is the same stack that
     * is used for _all_ interrupt handlers.  The end of this stack should be placed
     * against the lowest address in RAM so that a stack overrun results in a hard fault
     * at the first access beyond the end of the stack.
     */
    .irq_stack :
    {
         . = ALIGN(4);
         _irq_stack_end = . ;
         . = . + _irq_stack_size ;
         . = ALIGN(4);
         _irq_stack_top = . - 4 ;
         . = ALIGN(4);
    } > SRAM

    .data : AT (_etext)
    {
        _sdata = .;
        *(.data .data.*)
        . = ALIGN(4);
        _edata = . ;
    } > SRAM



    /* .bss section which is used for uninitialized data */
    .bss (NOLOAD) :
    {
        _sbss = . ;
        *(.bss .bss.*)
        *(COMMON)
    } > SRAM

    .heap (NOLOAD) :
    {
        . = ALIGN(4);
        _sheap = . ;
        _sheap_pre_rtos = . ;
        *(.heap)
        . = ALIGN(4);
        _eheap = . ;
        _eheap_pre_rtos = . ;
        _init_stack_end = . ;
        _sheap_post_rtos = . ;
        . = . + _init_stack_size ;
        . = ALIGN(4);
        _eheap_post_rtos = . ;
        _init_stack_top = . - 4 ;
    } > SRAM

    _eram = ORIGIN(SRAM) + LENGTH(SRAM) ;
    _ebss = _eram ;
    
    /* keep the heap section at the end of the SRAM
     * this will allow to claim the remaining bytes not used
     * at run time! (done by the reset vector).
     */

    PROVIDE ( _end = _ebss ) ;
}
