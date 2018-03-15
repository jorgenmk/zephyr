/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file SoC configuration macros for the Nordic Semiconductor nRF91 family processors.
 */

#ifndef _NORDICSEMI_NRF91_SOC_H_
#define _NORDICSEMI_NRF91_SOC_H_

#ifndef _ASMLANGUAGE

#include <nrf_common.h>
#include <nrf.h>
#include <device.h>
#include <misc/util.h>
#include <random/rand32.h>

#endif /* !_ASMLANGUAGE */

#define FLASH_PAGE_ERASE_MAX_TIME_US 	89700UL
#define FLASH_PAGE_MAX_CNT		256UL

#endif /* _NORDICSEMI_NRF91_SOC_H_ */
