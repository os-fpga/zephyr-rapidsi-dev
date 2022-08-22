/*
 * Copyright (c) 2021 Andes Technology Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT andestech_plic_sw

/**
 * @brief Andes V5 PLIC-SW IPI Controller driver
 */

#include <kernel.h>
#include <kernel_structs.h>
#include <ksched.h>
#include <init.h>
#include <soc.h>

/*
 * PLIC-SW Register Base Address
 */

#define ANDES_V5_PLIC_SW_BASE               DT_INST_REG_ADDR(0)

/*
 * PLIC-SW Register Offset
 */

#define PLIC_SW_PRIORITY_OFFSET             0x0
#define PLIC_SW_PENDING_OFFSET              0x1000
#define PLIC_SW_ENABLE_OFFSET               0x2000
#define PLIC_SW_CLAIM_OFFSET                0x200004

#define PLIC_SW_ENABLE_SHIFT_PER_TARGET     7
#define PLIC_SW_CLAIM_SHIFT_PER_TARGET      12

/*
 * Note of boot IPI:
 *   Assume the IPI is the boot IPI if and only if this is the 1st IPI
 *   received by 1 CPU. Otherwise it's sched IPI.
 *   CPU0 don't need boot IPI so setting it as received at start.
 */
static uint8_t boot_ipi_received[CONFIG_MP_NUM_CPUS] = {1};

static void plic_sw_irq_set_pending(uint32_t mask)
{
	volatile uint32_t *pend =
		(uint32_t *)(ANDES_V5_PLIC_SW_BASE + PLIC_SW_PENDING_OFFSET);

	*pend = mask;
}

void z_riscv_ipi_boardcast(void)
{
	unsigned int key = arch_irq_lock();

	/* Send IPI to all CPUs except itself. */
	uint32_t cpu_mask = (1 << CONFIG_MP_NUM_CPUS) - 1;

	cpu_mask &= ~(1 << _current_cpu->id);

	/* Current implementation only support 31 CPUs at most. */
	uint32_t source_mask = cpu_mask << 1;

	/* Send IPI by triggering the pending register of PLIC SW. */
	plic_sw_irq_set_pending(source_mask);

	arch_irq_unlock(key);
}

#if defined(CONFIG_SMP) && defined(CONFIG_SCHED_IPI_SUPPORTED)
void arch_sched_ipi(void)
{
	z_riscv_ipi_boardcast();
}
#endif /* defined(CONFIG_SMP) && defined(CONFIG_SCHED_IPI_SUPPORTED) */

static void plic_sw_irq_handler(void)
{
	volatile uint32_t *claim_complete =
		(uint32_t *)(ANDES_V5_PLIC_SW_BASE + PLIC_SW_CLAIM_OFFSET);
	claim_complete += (_current_cpu->id << (PLIC_SW_CLAIM_SHIFT_PER_TARGET - 2));

	/* PLIC claim: Get the SW IRQ number generating the interrupt. */
	uint32_t irq = *claim_complete;

	uint8_t is_boot_ipi = !boot_ipi_received[_current_cpu->id];

	if (is_boot_ipi) {
		boot_ipi_received[_current_cpu->id] = 1;
	}
#if defined(CONFIG_SMP) && defined(CONFIG_SCHED_IPI_SUPPORTED)
	else {
		z_sched_ipi();
	}
#endif /* defined(CONFIG_SMP) && defined(CONFIG_SCHED_IPI_SUPPORTED) */

	/* PLIC complete: Finish handling this SW IRQ. */
	*claim_complete = irq;
}

static int plic_sw_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	volatile uint32_t *prio = (uint32_t *)(ANDES_V5_PLIC_SW_BASE + PLIC_SW_PRIORITY_OFFSET);

	/* Set IRQ priority to 1 to enable used SW IRQ. */
	for (int irq = 1; irq <= CONFIG_MP_NUM_CPUS; irq++) {
		prio++;
		*prio = 1;
	}

	/*
	 * Enable SW IRQ to corresponding CPU core.
	 *
	 * Because PLIC SW IRQ is start from 1, CPU core N use SW IRQ N+1 as its IPI.
	 * Only enable SW IRQ N+1 in the CPU core N to make this SW IRQ as the IPI
	 * to the single core.
	 */
	for (int irq = 1; irq <= CONFIG_MP_NUM_CPUS; irq++) {
		int cpu_id = irq - 1;
		volatile uint32_t *en =
			(uint32_t *)(ANDES_V5_PLIC_SW_BASE + PLIC_SW_ENABLE_OFFSET);
		en += cpu_id << (PLIC_SW_ENABLE_SHIFT_PER_TARGET - 2);

		*en = (1 << (irq & 0x1f));
	}

	/* Setup IRQ handler for PLIC SW driver */
	IRQ_CONNECT(RISCV_MACHINE_SOFT_IRQ,
		    0,
		    plic_sw_irq_handler,
		    NULL,
		    0);

	/* Enable IRQ for PLIC SW driver */
	irq_enable(RISCV_MACHINE_SOFT_IRQ);

	return 0;
}

SYS_INIT(plic_sw_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
