/*
 * Copyright (C) 2012 Regents of the University of California
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation, version 2.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/sched/debug.h>
#include <linux/sched/signal.h>
#include <linux/signal.h>
#include <linux/kdebug.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/irq.h>

#include <asm/processor.h>
#include <asm/ptrace.h>
#include <asm/csr.h>
#include <utils/puts.h>


int show_unhandled_signals = 1;

extern asmlinkage void handle_exception(void);
extern asmlinkage void handle_exception_m(void);


static DEFINE_SPINLOCK(die_lock);

void die(struct pt_regs *regs, const char *str)
{
	static int die_counter;
	int ret;

	oops_enter();

	spin_lock_irq(&die_lock);
	console_verbose();
	bust_spinlocks(1);

	pr_emerg("%s [#%d]\n", str, ++die_counter);
	print_modules();
	show_regs(regs);

	ret = notify_die(DIE_OOPS, str, regs, 0, regs->scause, SIGSEGV);

	bust_spinlocks(0);
	add_taint(TAINT_DIE, LOCKDEP_NOW_UNRELIABLE);
	spin_unlock_irq(&die_lock);
	oops_exit();

	if (in_interrupt())
		panic("Fatal exception in interrupt");
	if (panic_on_oops)
		panic("Fatal exception");
	if (ret != NOTIFY_STOP)
		do_exit(SIGSEGV);
}

void do_trap(struct pt_regs *regs, int signo, int code,
	unsigned long addr, struct task_struct *tsk)
{
	if (show_unhandled_signals && unhandled_signal(tsk, signo)
	    && printk_ratelimit()) {
		pr_info("%s[%d]: unhandled signal %d code 0x%x at 0x" REG_FMT,
			tsk->comm, task_pid_nr(tsk), signo, code, addr);
		print_vma_addr(KERN_CONT " in ", GET_IP(regs));
		pr_cont("\n");
		show_regs(regs);
	}

	force_sig_fault(signo, code, (void __user *)addr, tsk);
}

static void do_trap_error(struct pt_regs *regs, int signo, int code,
	unsigned long addr, const char *str)
{
	DBGMSG(" code=%d scause=%08x sepc=%08x sstatus=%08x ra=%08x", code, regs->scause, regs->sepc, regs->sstatus, regs->ra);
	if (user_mode(regs)) {
		do_trap(regs, signo, code, addr, current);
	} else {
		if (!fixup_exception(regs))
			die(regs, str);
	}
}

uint32_t get_register_value(struct pt_regs *regs, int x) {
	if (x == 0)
		return 0;
	else
		return *((uint32_t*)regs + x);
}

void set_register_value(struct pt_regs *regs, int x, uint32_t val) {
	if (x != 0)
		*((uint32_t*)regs + x) = val;
}

int maybe_handle_illegal(struct pt_regs *regs)
{
	uint32_t insn;
	get_user(insn, (uint32_t*)regs->sepc);
	//DBGMSG("trying to handle illegal instr %08x", insn);
	if ((insn & 0x7F) == 0x2F) {
		//DBGMSG("handling A instr %08x", insn);
		// Atomic instructions - FIXME: make these actually atomic...
		uint32_t rs1v = get_register_value(regs, (insn >> 15) & 0x1F);
		uint32_t rs2v = get_register_value(regs, (insn >> 20) & 0x1F);
		uint32_t rdv = 0;
		uint32_t funct = (insn >> 27) & 0x1F;
		if (funct == 0x2) {
			// LR
			get_user(rdv, (uint32_t*)rs1v);
		} else if (funct == 0x3) {
			// SC
			put_user(rs2v, (uint32_t*)rs1v);
			rdv = 0;
		} else {
			//AMO
			uint32_t opres;
			get_user(rdv, (uint32_t*)rs1v);
			switch (funct) {
				case 0x00: //AMOADD
					opres = rdv + rs2v;
					break;
				case 0x01: // AMOSWAP
					opres = rs2v;
					break;
				case 0x04: //AMOXOR
					opres = rdv ^ rs2v;
					break;
				case 0x0C: //AMOAND
					opres = rdv & rs2v;
					break;
				case 0x08: //AMOOR
					opres = rdv | rs2v;
					break;
				case 0x10: //AMOMIN
					opres = (uint32_t)(min((int)rdv, (int)rs2v));
					break;
				case 0x14: //AMOMAX
					opres = (uint32_t)(max((int)rdv, (int)rs2v));
					break;
				case 0x18: //AMOMINU
					opres = min(rdv, rs2v);
					break;
				case 0x1C:
					opres = max(rdv, rs2v);
					break;
				default:
					DBG();
					return 0;
			}
			put_user(opres, (uint32_t*)rs1v);
		}
		set_register_value(regs, (insn >> 7) & 0x1F, rdv);
		return 1;
	}
	return 0;
}

asmlinkage void do_trap_insn_illegal(struct pt_regs *regs) {
	if (maybe_handle_illegal(regs)) {
		regs->sepc += 4;
		return;
	} else {
		do_trap_error(regs, SIGILL, ILL_ILLOPC, regs->sepc, "Oops - illegal instruction");
	}
}

#define DO_ERROR_INFO(name, signo, code, str)				\
asmlinkage void name(struct pt_regs *regs)				\
{									\
	do_trap_error(regs, signo, code, regs->sepc, "Oops - " str);	\
}

DO_ERROR_INFO(do_trap_unknown,
	SIGILL, ILL_ILLTRP, "unknown exception");
DO_ERROR_INFO(do_trap_insn_misaligned,
	SIGBUS, BUS_ADRALN, "instruction address misaligned");
DO_ERROR_INFO(do_trap_insn_fault,
	SIGSEGV, SEGV_ACCERR, "instruction access fault");
//DO_ERROR_INFO(do_trap_insn_illegal,
//	SIGILL, ILL_ILLOPC, "illegal instruction");
DO_ERROR_INFO(do_trap_load_misaligned,
	SIGBUS, BUS_ADRALN, "load address misaligned");
DO_ERROR_INFO(do_trap_load_fault,
	SIGSEGV, SEGV_ACCERR, "load access fault");
DO_ERROR_INFO(do_trap_store_misaligned,
	SIGBUS, BUS_ADRALN, "store (or AMO) address misaligned");
DO_ERROR_INFO(do_trap_store_fault,
	SIGSEGV, SEGV_ACCERR, "store (or AMO) access fault");
DO_ERROR_INFO(do_trap_ecall_u,
	SIGILL, ILL_ILLTRP, "environment call from U-mode");
DO_ERROR_INFO(do_trap_ecall_s,
	SIGILL, ILL_ILLTRP, "environment call from S-mode");
DO_ERROR_INFO(do_trap_ecall_m,
	SIGILL, ILL_ILLTRP, "environment call from M-mode");

asmlinkage void do_trap_break(struct pt_regs *regs)
{
#ifdef CONFIG_GENERIC_BUG
	if (!user_mode(regs)) {
		enum bug_trap_type type;

		type = report_bug(regs->sepc, regs);
		switch (type) {
		case BUG_TRAP_TYPE_NONE:
			break;
		case BUG_TRAP_TYPE_WARN:
			regs->sepc += sizeof(bug_insn_t);
			return;
		case BUG_TRAP_TYPE_BUG:
			die(regs, "Kernel BUG");
		}
	}
#endif /* CONFIG_GENERIC_BUG */

	force_sig_fault(SIGTRAP, TRAP_BRKPT, (void __user *)(regs->sepc), current);
}

#ifdef CONFIG_GENERIC_BUG
int is_valid_bugaddr(unsigned long pc)
{
	bug_insn_t insn;

	if (pc < PAGE_OFFSET)
		return 0;
	if (probe_kernel_address((bug_insn_t *)pc, insn))
		return 0;
	return (insn == __BUG_INSN);
}
#endif /* CONFIG_GENERIC_BUG */


#define IRQ_S_SOFT   1
#define IRQ_S_TIMER  5
#define IRQ_S_EXT    9

#define MIP_SSIP            (1 << IRQ_S_SOFT)
#define MIP_STIP            (1 << IRQ_S_TIMER)
#define MIP_SEIP            (1 << IRQ_S_EXT)

#define CAUSE_MISALIGNED_FETCH 0x0
#define CAUSE_ILLEGAL_INSTRUCTION 0x2
#define CAUSE_FETCH_PAGE_FAULT 0xc
#define CAUSE_STORE_PAGE_FAULT 0xf
#define CAUSE_LOAD_PAGE_FAULT 0xd
#define CAUSE_USER_ECALL 0x8
#define CAUSE_BREAKPOINT 0x3
#define CAUSE_MISALIGNED_FETCH 0x0

void __init mtrap_init(void)
{
  csr_write(mtvec, &handle_exception_m);
  uintptr_t interrupts = MIP_SSIP | MIP_STIP | MIP_SEIP;
  uintptr_t exceptions =
    (1U << CAUSE_MISALIGNED_FETCH) |
    (1U << CAUSE_ILLEGAL_INSTRUCTION) |
    (1U << CAUSE_FETCH_PAGE_FAULT) |
    (1U << CAUSE_BREAKPOINT) |
    (1U << CAUSE_LOAD_PAGE_FAULT) |
    (1U << CAUSE_STORE_PAGE_FAULT) |
    (1U << CAUSE_USER_ECALL);
  csr_write(mideleg, interrupts);
  csr_write(medeleg, exceptions);

  /* Enable Machine External Interrupt */
  csr_write(mie, (1 << 11));
}


void __init trap_init(void)
{

	/*
	 * Set sup0 scratch register to 0, indicating to exception vector
	 * that we are presently executing in the kernel
	 */
	csr_write(sscratch, 0);
	/* Set the exception vector address */
	csr_write(stvec, &handle_exception);
	/* Enable all interrupts */
	csr_write(sie, -1);
}
