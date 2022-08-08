/*
 * Copyright (c) 2021 Andes Technology Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Macros for the Andes AE250 platform
 */

#ifndef __RISCV_ANDES_AE250_SOC_H_
#define __RISCV_ANDES_AE250_SOC_H_

#include <soc_common.h>
#include <devicetree.h>

/* Machine timer memory-mapped registers */
#define RISCV_MTIME_BASE             0xE6000000
#define RISCV_MTIMECMP_BASE          0xE6000008

/* ILM/DLM Abstraction for both soc.h and linker.ld usage */
#define ILM_BASE                     DT_REG_ADDR(DT_PATH(memory_0))
#define ILM_SIZE                     DT_REG_SIZE(DT_PATH(memory_0))
#define DLM_BASE                     DT_REG_ADDR(DT_PATH(memory_200000))
#define DLM_SIZE                     DT_REG_SIZE(DT_PATH(memory_200000))

/* lib-c hooks required RAM defined variables */
#ifdef CONFIG_XIP
/* AE250 RAMABLE_REGION is ILM in XIP mode */
# define RISCV_RAM_BASE              ILM_BASE
# define RISCV_RAM_SIZE              ILM_SIZE
#else /* CONFIG_XIP */
/* AE250 RAMABLE_REGION is DLM in non-XIP mode */
# define RISCV_RAM_BASE              DLM_BASE
# define RISCV_RAM_SIZE              DLM_SIZE
#endif /* CONFIG_XIP */

/* Include CSRs available for Andes V5 SoCs */
#include "soc_v5.h"

/* Include platform peripherals */
#include "smu.h"

#endif /* __RISCV_ANDES_AE250_SOC_H_ */
