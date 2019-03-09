// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2012 Regents of the University of California
 * Copyright (C) 2017 SiFive
 */
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <asm/sbi.h>
#include <utils/puts.h>

/*
 * All RISC-V systems have a timer attached to every hart.  These timers can be
 * read by the 'rdcycle' pseudo instruction, and can use the SBI to setup
 * events.  In order to abstract the architecture-specific timer reading and
 * setting functions away from the clock event insertion code, we provide
 * function pointers to the clockevent subsystem that perform two basic
 * operations: rdtime() reads the timer on the current CPU, and
 * next_event(delta) sets the next timer event to 'delta' cycles in the future.
 * As the timers are inherently a per-cpu resource, these callbacks perform
 * operations on the current hart.  There is guaranteed to be exactly one timer
 * per hart on all RISC-V systems.
 */
#define TIMER0_BASE 0xe0002800

static int riscv_clock_next_event(unsigned long delta,
		struct clock_event_device *ce)
{
	//DBGMSG("set_timer %lu\n", delta);
	delta *= 100;
	volatile uint32_t *timer = (uint32_t*)TIMER0_BASE;
	timer[14] = 0;
	csr_set(sie, SIE_STIE);
	timer[8] = 0;
	timer[0] = (delta >> 24) & 0xFF;
	timer[1] = (delta >> 16) & 0xFF;
	timer[2] = (delta >>  8) & 0xFF;
	timer[3] = (delta      ) & 0xFF;
	timer[16] = 1;
	timer[8] = 1;
	return 0;
}

static DEFINE_PER_CPU(struct clock_event_device, riscv_clock_event) = {
	.name			= "riscv_timer_clockevent",
	.features		= CLOCK_EVT_FEAT_ONESHOT,
	.rating			= 100,
	.set_next_event		= riscv_clock_next_event,
};

/*
 * It is guaranteed that all the timers across all the harts are synchronized
 * within one tick of each other, so while this could technically go
 * backwards when hopping between CPUs, practically it won't happen.
 */
static unsigned long long riscv_clocksource_rdtime(struct clocksource *cs)
{
#if 0
	unsigned long long cycles = get_cycles64();
	DBGMSG("RDTIME %llu", cycles);
	return cycles;
#else
	static unsigned long long fake_timer = 0;
	fake_timer += 1000000;
	//DBGMSG("RDTIME [fake] %llu", fake_timer);
	return fake_timer;
#endif
}

static DEFINE_PER_CPU(struct clocksource, riscv_clocksource) = {
	.name		= "riscv_clocksource",
	.rating		= 300,
	.mask		= CLOCKSOURCE_MASK(BITS_PER_LONG),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
	.read		= riscv_clocksource_rdtime,
};

static int riscv_timer_starting_cpu(unsigned int cpu)
{
	struct clock_event_device *ce = per_cpu_ptr(&riscv_clock_event, cpu);

	ce->cpumask = cpumask_of(cpu);
	clockevents_config_and_register(ce, riscv_timebase, 100, 0x7fffffff);

	csr_set(sie, SIE_STIE);
	return 0;
}

static int riscv_timer_dying_cpu(unsigned int cpu)
{
	csr_clear(sie, SIE_STIE);
	return 0;
}

/* called directly from the low-level interrupt handler */
void riscv_timer_interrupt(void)
{
	volatile uint32_t *timer = (uint32_t*)TIMER0_BASE;
	timer[16] = 0;

	struct clock_event_device *evdev = this_cpu_ptr(&riscv_clock_event);
	csr_clear(sie, SIE_STIE);
	evdev->event_handler(evdev);
}

static int __init riscv_timer_init_dt(struct device_node *n)
{
	int cpu_id = riscv_of_processor_hart(n), error;
	struct clocksource *cs;

	if (cpu_id != smp_processor_id())
		return 0;

	cs = per_cpu_ptr(&riscv_clocksource, cpu_id);
	clocksource_register_hz(cs, riscv_timebase);

	error = cpuhp_setup_state(CPUHP_AP_RISCV_TIMER_STARTING,
			 "clockevents/riscv/timer:starting",
			 riscv_timer_starting_cpu, riscv_timer_dying_cpu);
	if (error)
		pr_err("RISCV timer register failed [%d] for cpu = [%d]\n",
		       error, cpu_id);
	return error;
}

TIMER_OF_DECLARE(riscv_timer, "riscv", riscv_timer_init_dt);
