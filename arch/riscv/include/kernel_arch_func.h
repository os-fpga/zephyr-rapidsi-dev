/*
 * Copyright (c) 2016 Jean-Paul Etienne <fractalclone@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Private kernel definitions
 *
 * This file contains private kernel function/macro definitions and various
 * other definitions for the RISCV processor architecture.
 */

#ifndef ZEPHYR_ARCH_RISCV_INCLUDE_KERNEL_ARCH_FUNC_H_
#define ZEPHYR_ARCH_RISCV_INCLUDE_KERNEL_ARCH_FUNC_H_

#include <kernel_arch_data.h>
#include <kernel_structs.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _ASMLANGUAGE
static ALWAYS_INLINE void arch_kernel_init(void)
{
	_cpu_t* cpu0 = &_kernel.cpus[0];

	__asm__ volatile("csrw mscratch, %0" :: "r" (cpu0));
}

#ifdef CONFIG_USE_SWITCH
void arch_switch(void *switch_to, void **switched_from);
#else

/* This is a arch function traditionally, but when the switch-based
 * z_swap() is in use it's a simple inline provided by the kernel.
 */
static ALWAYS_INLINE void
arch_thread_return_value_set(struct k_thread *thread, unsigned int value)
{
	thread->arch.swap_return_value = value;
}
#endif

#if defined(CONFIG_SMP) && defined(CONFIG_SCHED_IPI_SUPPORTED)
void arch_sched_ipi(void);
#endif

FUNC_NORETURN void z_riscv_fatal_error(unsigned int reason,
				       const z_arch_esf_t *esf);

static inline bool arch_is_in_isr(void)
{
#ifdef CONFIG_SMP
	/* Refer x86 implementation. we should disable irq to prevent race condition
	 * of current CPU changing.
	 */
	bool ret;

	unsigned int key = arch_irq_lock();
	ret = (arch_curr_cpu()->nested != 0U);
	arch_irq_unlock(key);
	return ret;
#else
	return _kernel.cpus[0].nested != 0U;
#endif
}

#ifdef CONFIG_IRQ_OFFLOAD
int z_irq_do_offload(void);
#endif

#if defined(CONFIG_SMP) || (CONFIG_MP_NUM_CPUS > 1)
#if defined(CONFIG_SCHED_IPI_SUPPORTED)
/*
 * Send IPI to all CPUs except itself.
 */
void z_riscv_ipi_boardcast(void);
#endif /* defined(CONFIG_SCHED_IPI_SUPPORTED) */
#endif /* defined(CONFIG_SMP) || (CONFIG_MP_NUM_CPUS > 1) */

#endif /* _ASMLANGUAGE */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_ARCH_RISCV_INCLUDE_KERNEL_ARCH_FUNC_H_ */
