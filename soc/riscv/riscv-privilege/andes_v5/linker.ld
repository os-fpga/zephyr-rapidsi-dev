/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Linker command/script file
 *
 * linker script for andes_v5 SoC Series
 */

#include <autoconf.h>

#if defined(CONFIG_SOC_RISCV_ANDES_AE250)
# include <ae250/linker.ld>
#elif defined(CONFIG_SOC_RISCV_ANDES_AE350)
# include <ae350/linker.ld>
#endif
