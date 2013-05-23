/**
 ******************************************************************************
 * @addtogroup OpenPilotSystem OpenPilot System
 * @{
 * @addtogroup OpenPilotCore OpenPilot Core
 * @{
 *
 * @file       board_hw_defs.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @brief      Defines board specific static initializers for hardware for the OpenPilot board.
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/* 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 3 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program; if not, write to the Free Software Foundation, Inc., 
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include <pios_config.h>
#include <pios_board_info.h>

#if defined(PIOS_INCLUDE_LED)

#include <pios_led_priv.h>
static const struct pios_led pios_leds[] = {
	[PIOS_LED_HEARTBEAT] = {
		.pin = {
			.gpio = GPIOB,
			.init = {
				.GPIO_Pin   = GPIO_Pin_3,
				.GPIO_Mode  = GPIO_Mode_Out_PP,
				.GPIO_Speed = GPIO_Speed_50MHz,
			},
		},
		.remap = GPIO_Remap_SWJ_JTAGDisable,
	},
	[PIOS_LED_ALARM] = {
		.pin = {
			.gpio = GPIOB,
			.init = {
				.GPIO_Pin   = GPIO_Pin_4,
				.GPIO_Mode  = GPIO_Mode_Out_PP,
				.GPIO_Speed = GPIO_Speed_50MHz,
			},
		},
	},
};

static const struct pios_led_cfg pios_led_cfg = {
	.leds     = pios_leds,
	.num_leds = NELEMENTS(pios_leds),
};

const struct pios_led_cfg * PIOS_BOARD_HW_DEFS_GetLedCfg (uint32_t board_revision)
{
	return &pios_led_cfg;
}

#endif	/* PIOS_INCLUDE_LED */

#include "pios_tim_priv.h"

static const TIM_TimeBaseInitTypeDef tim_1_2_3_4_time_base = {
	.TIM_Prescaler = (PIOS_MASTER_CLOCK / 1000000) - 1,
	.TIM_ClockDivision = TIM_CKD_DIV1,
	.TIM_CounterMode = TIM_CounterMode_Up,
	.TIM_Period = ((1000000 / PIOS_SERVO_UPDATE_HZ) - 1),
	.TIM_RepetitionCounter = 0x0000,
};

static const struct pios_tim_clock_cfg tim_1_cfg = {
	.timer = TIM1,
	.time_base_init = &tim_1_2_3_4_time_base,
	.irq = {
		.init = {
			.NVIC_IRQChannel                   = TIM1_CC_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
};

static const struct pios_tim_clock_cfg tim_2_cfg = {
	.timer = TIM2,
	.time_base_init = &tim_1_2_3_4_time_base,
	.irq = {
		.init = {
			.NVIC_IRQChannel                   = TIM2_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
};

static const struct pios_tim_clock_cfg tim_3_cfg = {
	.timer = TIM3,
	.time_base_init = &tim_1_2_3_4_time_base,
	.irq = {
		.init = {
			.NVIC_IRQChannel                   = TIM3_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
};

static const struct pios_tim_clock_cfg tim_4_cfg = {
	.timer = TIM4,
	.time_base_init = &tim_1_2_3_4_time_base,
	.irq = {
		.init = {
			.NVIC_IRQChannel                   = TIM4_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
};

static const struct pios_tim_channel pios_tim_rcvrport_all_channels[] = {
	{
		.timer = TIM2,
		.timer_chan = TIM_Channel_1,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin   = GPIO_Pin_0,
				.GPIO_Mode  = GPIO_Mode_IPD,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM2,
		.timer_chan = TIM_Channel_2,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin   = GPIO_Pin_1,
				.GPIO_Mode  = GPIO_Mode_IPD,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM2,
		.timer_chan = TIM_Channel_3,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin   = GPIO_Pin_2,
				.GPIO_Mode  = GPIO_Mode_IPD,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM2,
		.timer_chan = TIM_Channel_4,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin   = GPIO_Pin_3,
				.GPIO_Mode  = GPIO_Mode_IPD,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM3,
		.timer_chan = TIM_Channel_1,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin   = GPIO_Pin_6,
				.GPIO_Mode  = GPIO_Mode_IPD,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM3,
		.timer_chan = TIM_Channel_2,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin   = GPIO_Pin_7,
				.GPIO_Mode  = GPIO_Mode_IPD,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM3,
		.timer_chan = TIM_Channel_3,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.GPIO_Pin   = GPIO_Pin_0,
				.GPIO_Mode  = GPIO_Mode_IPD,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM3,
		.timer_chan = TIM_Channel_4,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.GPIO_Pin   = GPIO_Pin_1,
				.GPIO_Mode  = GPIO_Mode_IPD,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
};

static const struct pios_tim_channel pios_tim_servoport_all_pins[] = {
	{
		.timer = TIM1,
		.timer_chan = TIM_Channel_1,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin   = GPIO_Pin_8,
				.GPIO_Mode  = GPIO_Mode_AF_PP,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM1,
		.timer_chan = TIM_Channel_4,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin   = GPIO_Pin_11,
				.GPIO_Mode  = GPIO_Mode_AF_PP,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM4,
		.timer_chan = TIM_Channel_1,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.GPIO_Pin   = GPIO_Pin_6,
				.GPIO_Mode  = GPIO_Mode_AF_PP,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM4,
		.timer_chan = TIM_Channel_2,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.GPIO_Pin   = GPIO_Pin_7,
				.GPIO_Mode  = GPIO_Mode_AF_PP,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM4,
		.timer_chan = TIM_Channel_3,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.GPIO_Pin   = GPIO_Pin_8,
				.GPIO_Mode  = GPIO_Mode_AF_PP,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM4,
		.timer_chan = TIM_Channel_4,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.GPIO_Pin   = GPIO_Pin_9,
				.GPIO_Mode  = GPIO_Mode_AF_PP,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
};


static const struct pios_tim_channel pios_tim_servoport_rcvrport_pins[] = {
	{
		.timer = TIM2,
		.timer_chan = TIM_Channel_1,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin   = GPIO_Pin_0,
				.GPIO_Mode  = GPIO_Mode_IPD,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM2,
		.timer_chan = TIM_Channel_2,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin   = GPIO_Pin_1,
				.GPIO_Mode  = GPIO_Mode_IPD,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM2,
		.timer_chan = TIM_Channel_3,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin   = GPIO_Pin_2,
				.GPIO_Mode  = GPIO_Mode_IPD,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM2,
		.timer_chan = TIM_Channel_4,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin   = GPIO_Pin_3,
				.GPIO_Mode  = GPIO_Mode_IPD,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	}, 
	{
		.timer = TIM3,
		.timer_chan = TIM_Channel_1,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin   = GPIO_Pin_6,
				.GPIO_Mode  = GPIO_Mode_IPD,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM3,
		.timer_chan = TIM_Channel_2,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin   = GPIO_Pin_7,
				.GPIO_Mode  = GPIO_Mode_IPD,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM3,
		.timer_chan = TIM_Channel_3,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.GPIO_Pin   = GPIO_Pin_0,
				.GPIO_Mode  = GPIO_Mode_IPD,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM3,
		.timer_chan = TIM_Channel_4,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.GPIO_Pin   = GPIO_Pin_1,
				.GPIO_Mode  = GPIO_Mode_IPD,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	
	// Receiver port pins
	// S3-S6 inputs are used as outputs in this case
	{
		.timer = TIM1,
		.timer_chan = TIM_Channel_1,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin   = GPIO_Pin_8,
				.GPIO_Mode  = GPIO_Mode_AF_PP,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM1,
		.timer_chan = TIM_Channel_4,
		.pin = {
			.gpio = GPIOA,
			.init = {
				.GPIO_Pin   = GPIO_Pin_11,
				.GPIO_Mode  = GPIO_Mode_AF_PP,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM4,
		.timer_chan = TIM_Channel_1,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.GPIO_Pin   = GPIO_Pin_6,
				.GPIO_Mode  = GPIO_Mode_AF_PP,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM4,
		.timer_chan = TIM_Channel_2,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.GPIO_Pin   = GPIO_Pin_7,
				.GPIO_Mode  = GPIO_Mode_AF_PP,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM4,
		.timer_chan = TIM_Channel_3,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.GPIO_Pin   = GPIO_Pin_8,
				.GPIO_Mode  = GPIO_Mode_AF_PP,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
	{
		.timer = TIM4,
		.timer_chan = TIM_Channel_4,
		.pin = {
			.gpio = GPIOB,
			.init = {
				.GPIO_Pin   = GPIO_Pin_9,
				.GPIO_Mode  = GPIO_Mode_AF_PP,
				.GPIO_Speed = GPIO_Speed_2MHz,
			},
		},
	},
};

#if defined(PIOS_INCLUDE_USART)

#include "pios_usart_priv.h"

static const struct pios_usart_cfg pios_usart_generic_main_cfg = {
	.regs  = USART1,
	.init = {
		.USART_BaudRate            = 57600,
		.USART_WordLength          = USART_WordLength_8b,
		.USART_Parity              = USART_Parity_No,
		.USART_StopBits            = USART_StopBits_1,
		.USART_HardwareFlowControl = USART_HardwareFlowControl_None,
		.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx,
	},
	.irq = {
		.init    = {
			.NVIC_IRQChannel                   = USART1_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
	.rx   = {
		.gpio = GPIOA,
		.init = {
			.GPIO_Pin   = GPIO_Pin_10,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_IPU,
		},
	},
	.tx   = {
		.gpio = GPIOA,
		.init = {
			.GPIO_Pin   = GPIO_Pin_9,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_AF_PP,
		},
	},
};

// PWM and USART2 are mutually exclusive
// because they have common pins (tx=RC_CH3, rx=RC_CH4)
static const struct pios_usart_cfg pios_usart_generic_rcvr_cfg = {
	.regs  = USART2,
	.init = {
		.USART_BaudRate            = 57600,
		.USART_WordLength          = USART_WordLength_8b,
		.USART_Parity              = USART_Parity_No,
		.USART_StopBits            = USART_StopBits_1,
		.USART_HardwareFlowControl = USART_HardwareFlowControl_None,
		.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx,
	},
	.irq = {
		.init    = {
			.NVIC_IRQChannel                   = USART2_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
	.rx   = {	// USART2_RX    PA3	( PWM_IN4 )
		.gpio = GPIOA,
		.init = {
			.GPIO_Pin   = GPIO_Pin_3,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_IPU,
		},
	},
	.tx   = {	// USART2_TX    PA2	( PWM_IN3 )
		.gpio = GPIOA,
		.init = {
			.GPIO_Pin   = GPIO_Pin_2,
			.GPIO_Speed = GPIO_Speed_2MHz,
			.GPIO_Mode  = GPIO_Mode_AF_PP,
		},
	},
};

#endif  /* PIOS_INCLUDE_USART */

#if defined(PIOS_INCLUDE_COM)

#include "pios_com_priv.h"

#endif	/* PIOS_INCLUDE_COM */

#if defined(PIOS_INCLUDE_RTC)
/*
 * Realtime Clock (RTC)
 */
#include <pios_rtc_priv.h>

void PIOS_RTC_IRQ_Handler (void);
void RTC_IRQHandler() __attribute__ ((alias ("PIOS_RTC_IRQ_Handler")));
static const struct pios_rtc_cfg pios_rtc_main_cfg = {
	.clksrc = RCC_RTCCLKSource_HSE_Div128,
	.prescaler = 100,
	.irq = {
		.init = {
			.NVIC_IRQChannel                   = RTC_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
};

void PIOS_RTC_IRQ_Handler (void)
{
	PIOS_RTC_irq_handler ();
}

#endif

#if defined(PIOS_INCLUDE_SERVO) && defined(PIOS_INCLUDE_TIM)
/* 
 * Servo outputs 
 */
#include <pios_servo_priv.h>

const struct pios_servo_cfg pios_servo_cfg = {
	.tim_oc_init = {
		.TIM_OCMode = TIM_OCMode_PWM1,
		.TIM_OutputState = TIM_OutputState_Enable,
		.TIM_OutputNState = TIM_OutputNState_Disable,
		.TIM_Pulse = PIOS_SERVOS_INITIAL_POSITION,
		.TIM_OCPolarity = TIM_OCPolarity_High,
		.TIM_OCNPolarity = TIM_OCPolarity_High,
		.TIM_OCIdleState = TIM_OCIdleState_Reset,
		.TIM_OCNIdleState = TIM_OCNIdleState_Reset,
	},
	.channels = pios_tim_servoport_all_pins,
	.num_channels = NELEMENTS(pios_tim_servoport_all_pins),
};

const struct pios_servo_cfg pios_servo_rcvr_cfg = {
	.tim_oc_init = {
		.TIM_OCMode = TIM_OCMode_PWM1,
		.TIM_OutputState = TIM_OutputState_Enable,
		.TIM_OutputNState = TIM_OutputNState_Disable,
		.TIM_Pulse = PIOS_SERVOS_INITIAL_POSITION,
		.TIM_OCPolarity = TIM_OCPolarity_High,
		.TIM_OCNPolarity = TIM_OCPolarity_High,
		.TIM_OCIdleState = TIM_OCIdleState_Reset,
		.TIM_OCNIdleState = TIM_OCNIdleState_Reset,
	},
	.channels = pios_tim_servoport_rcvrport_pins,
	.num_channels = NELEMENTS(pios_tim_servoport_rcvrport_pins),
};

#endif	/* PIOS_INCLUDE_SERVO && PIOS_INCLUDE_TIM */

/*
 * PPM Inputs
 */
#if defined(PIOS_INCLUDE_PPM)
#include <pios_ppm_priv.h>

const struct pios_ppm_cfg pios_ppm_cfg = {
	.tim_ic_init = {
		.TIM_ICPolarity = TIM_ICPolarity_Rising,
		.TIM_ICSelection = TIM_ICSelection_DirectTI,
		.TIM_ICPrescaler = TIM_ICPSC_DIV1,
		.TIM_ICFilter = 0x0,
	},
	/* Use only the first channel for ppm */
	.channels = &pios_tim_rcvrport_all_channels[0],
	.num_channels = 1,
};

#endif	/* PIOS_INCLUDE_PPM */

/* 
 * PWM Inputs 
 */
#if defined(PIOS_INCLUDE_PWM)
#include <pios_pwm_priv.h>

const struct pios_pwm_cfg pios_pwm_cfg = {
	.tim_ic_init = {
		.TIM_ICPolarity = TIM_ICPolarity_Rising,
		.TIM_ICSelection = TIM_ICSelection_DirectTI,
		.TIM_ICPrescaler = TIM_ICPSC_DIV1,
		.TIM_ICFilter = 0x0,
	},
	.channels = pios_tim_rcvrport_all_channels,
	.num_channels = NELEMENTS(pios_tim_rcvrport_all_channels),
};

const struct pios_pwm_cfg pios_pwm_with_ppm_cfg = {
	.tim_ic_init = {
		.TIM_ICPolarity = TIM_ICPolarity_Rising,
		.TIM_ICSelection = TIM_ICSelection_DirectTI,
		.TIM_ICPrescaler = TIM_ICPSC_DIV1,
		.TIM_ICFilter = 0x0,
	},
	/* Leave the first channel for PPM use and use the rest for PWM */
	.channels = &pios_tim_rcvrport_all_channels[1],
	.num_channels = NELEMENTS(pios_tim_rcvrport_all_channels) - 1,
};

#endif

#if defined(PIOS_INCLUDE_I2C)

#include <pios_i2c_priv.h>

/*
 * I2C Adapters
 */

void PIOS_I2C_flexi_adapter_ev_irq_handler(void);
void PIOS_I2C_flexi_adapter_er_irq_handler(void);
void I2C2_EV_IRQHandler() __attribute__ ((alias ("PIOS_I2C_flexi_adapter_ev_irq_handler")));
void I2C2_ER_IRQHandler() __attribute__ ((alias ("PIOS_I2C_flexi_adapter_er_irq_handler")));

static const struct pios_i2c_adapter_cfg pios_i2c_flexi_adapter_cfg = {
	.regs = I2C2,
	.init = {
		.I2C_Mode                = I2C_Mode_I2C,
		.I2C_OwnAddress1         = 0,
		.I2C_Ack                 = I2C_Ack_Enable,
		.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit,
		.I2C_DutyCycle           = I2C_DutyCycle_2,
		.I2C_ClockSpeed          = 400000,	/* bits/s */
	},
	.transfer_timeout_ms = 50,
	.scl = {
		.gpio = GPIOB,
		.init = {
			.GPIO_Pin   = GPIO_Pin_10,
			.GPIO_Speed = GPIO_Speed_10MHz,
			.GPIO_Mode  = GPIO_Mode_AF_OD,
		},
	},
	.sda = {
		.gpio = GPIOB,
		.init = {
			.GPIO_Pin   = GPIO_Pin_11,
			.GPIO_Speed = GPIO_Speed_10MHz,
			.GPIO_Mode  = GPIO_Mode_AF_OD,
		},
	},
	.event = {
		.flags   = 0,		/* FIXME: check this */
		.init = {
			.NVIC_IRQChannel                   = I2C2_EV_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_EXTREME,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
	.error = {
		.flags   = 0,		/* FIXME: check this */
		.init = {
			.NVIC_IRQChannel                   = I2C2_ER_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_EXTREME,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
};

uint32_t pios_i2c_flexi_adapter_id;
void PIOS_I2C_flexi_adapter_ev_irq_handler(void)
{
	/* Call into the generic code to handle the IRQ for this specific device */
	PIOS_I2C_EV_IRQ_Handler(pios_i2c_flexi_adapter_id);
}

void PIOS_I2C_flexi_adapter_er_irq_handler(void)
{
	/* Call into the generic code to handle the IRQ for this specific device */
	PIOS_I2C_ER_IRQ_Handler(pios_i2c_flexi_adapter_id);
}

#endif /* PIOS_INCLUDE_I2C */

#if defined(PIOS_INCLUDE_GCSRCVR)
#include "pios_gcsrcvr_priv.h"
#endif	/* PIOS_INCLUDE_GCSRCVR */

#if defined(PIOS_INCLUDE_RCVR)
#include "pios_rcvr_priv.h"

#endif /* PIOS_INCLUDE_RCVR */

#if defined(PIOS_INCLUDE_COM_MSG)

#include <pios_com_msg_priv.h>

#endif /* PIOS_INCLUDE_COM_MSG */
