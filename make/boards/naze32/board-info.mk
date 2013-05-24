BOARD_TYPE          := 0x04
BOARD_REVISION      := 0x02
BOOTLOADER_VERSION  := 0x03
HW_TYPE             := 0x01

MCU                 := cortex-m3
CHIP                := STM32F103CBT
BOARD               := STM32103CB_NAZE32
MODEL               := MD
MODEL_SUFFIX        := _CC

OPENOCD_JTAG_CONFIG := stlink-v2.cfg
OPENOCD_CONFIG      := stm32f1x_stlink.cfg

FW_BANK_BASE        := 0x08000000  # Start of firmware flash
FW_BANK_SIZE        := 0x0001D000  # Should include FW_DESC_SIZE

FW_DESC_SIZE        := 0x00000064

EE_BANK_BASE        := 0x0801D000  # Flash used as storage area
EE_BANK_SIZE        := 0x00003000  # Size of Flash used as storage area
