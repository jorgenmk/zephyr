/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for Nordic Semiconductor nRF91 family processor
 *
 * This module provides routines to initialize and support board-level hardware
 * for the Nordic Semiconductor nRF91 family processor.
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <cortex_m/exc.h>

#ifdef CONFIG_RUNTIME_NMI
extern void _NmiInit(void);
#define NMI_INIT() _NmiInit()
#else
#define NMI_INIT()
#endif

#include "nrf.h"

#define __SYSTEM_CLOCK_64M (64000000UL)

uint32_t SystemCoreClock __used = __SYSTEM_CLOCK_64M;

static void clock_init(void)
{
	SystemCoreClock = __SYSTEM_CLOCK_64M;
}

static int nordicsemi_nrf91_init(struct device *arg)
{
	u32_t key;

	ARG_UNUSED(arg);

	key = irq_lock();

	/* Enable the FPU if the compiler used floating point unit
	 * instructions. Since the FPU consumes energy, remember to
	 * disable FPU use in the compiler if floating point unit
	 * operations are not used in your code.
	 */
#if defined(CONFIG_FLOAT)
	SCB->CPACR |= (3UL << 20) | (3UL << 22);
	__DSB();
	__ISB();
#endif

#if defined(CONFIG_SOC_NRF9120_MLM1)
        NRF_CLOCK->HFCLKSRC         = CLOCK_HFCLKSRC_SRC_HFXO;
        NRF_CLOCK->TASKS_HFCLKSTART = 1;
#endif /* CONFIG_SOC_NRF9120_MLM1 */

#if defined (CONFIG_GPIO_AS_PINRESET)
	if (((NRF_UICR->PSELRESET[0] & UICR_PSELRESET_CONNECT_Msk) != (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos)) ||
	((NRF_UICR->PSELRESET[1] & UICR_PSELRESET_CONNECT_Msk) != (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos)))
	{
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
		NRF_UICR->PSELRESET[0] = 20;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
		NRF_UICR->PSELRESET[1] = 20;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
		NVIC_SystemReset();
	}
#endif

	_ClearFaults();

	/* Setup master clock */
	clock_init();

	/* Install default handler that simply resets the CPU
	* if configured in the kernel, NOP otherwise
	*/
	NMI_INIT();

	irq_unlock(key);

	return 0;
}

SYS_INIT(nordicsemi_nrf91_init, PRE_KERNEL_1, 0);
