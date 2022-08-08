/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Per-arch thread definition
 *
 * This file contains definitions for
 *
 *  struct _thread_arch
 *  struct _callee_saved
 *
 * necessary to instantiate instances of struct k_thread.
 */

#ifndef ZEPHYR_INCLUDE_ARCH_RISCV_THREAD_H_
#define ZEPHYR_INCLUDE_ARCH_RISCV_THREAD_H_

#ifndef _ASMLANGUAGE
#include <zephyr/types.h>

#if !defined(RV_FP_TYPE) && defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING)
#ifdef CONFIG_CPU_HAS_FPU_DOUBLE_PRECISION
#define RV_FP_TYPE uint64_t
#else
#define RV_FP_TYPE uint32_t
#endif
#endif

/*
 * The following structure defines the list of registers that need to be
 * saved/restored when a cooperative context switch occurs.
 */
struct _callee_saved {
	ulong_t sp;	/* Stack pointer, (x2 register) */

	ulong_t s0;	/* saved register/frame pointer */
	ulong_t s1;	/* saved register */
#ifndef CONFIG_EMBEDDED_ISA
	ulong_t s2;	/* saved register */
	ulong_t s3;	/* saved register */
	ulong_t s4;	/* saved register */
	ulong_t s5;	/* saved register */
	ulong_t s6;	/* saved register */
	ulong_t s7;	/* saved register */
	ulong_t s8;	/* saved register */
	ulong_t s9;	/* saved register */
	ulong_t s10;	/* saved register */
	ulong_t s11;	/* saved register */
#endif

#if defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING)
	uint32_t fcsr;		/* Control and status register */
	RV_FP_TYPE fs0;		/* saved floating-point register */
	RV_FP_TYPE fs1;		/* saved floating-point register */
	RV_FP_TYPE fs2;		/* saved floating-point register */
	RV_FP_TYPE fs3;		/* saved floating-point register */
	RV_FP_TYPE fs4;		/* saved floating-point register */
	RV_FP_TYPE fs5;		/* saved floating-point register */
	RV_FP_TYPE fs6;		/* saved floating-point register */
	RV_FP_TYPE fs7;		/* saved floating-point register */
	RV_FP_TYPE fs8;		/* saved floating-point register */
	RV_FP_TYPE fs9;		/* saved floating-point register */
	RV_FP_TYPE fs10;	/* saved floating-point register */
	RV_FP_TYPE fs11;	/* saved floating-point register */
#endif

#if defined(CONFIG_SMP_HOTFIX_SPIN_ON_RISCV_CALLEE)
	/* There is an unavoidable SMP race when threads swap.
	 * (see wait_for_switch())
	 *
	 * When restoring callee register, spin until these registers saved
	 * by previous CPU to avoid race condition.
	 *
	 * 0 means all callee registers are saved so that we can restore it.
	 * 1 means some callee registers aren't saved, please spin on it.
	 */
	uint32_t callee_state;	/* Callee saved context state */
#endif
};
typedef struct _callee_saved _callee_saved_t;

struct _thread_arch {
	uint32_t swap_return_value; /* Return value of z_swap() */
};

typedef struct _thread_arch _thread_arch_t;

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_RISCV_THREAD_H_ */
