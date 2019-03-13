/*
 * Copyright (C) 2009 Sunplus Core Technology Co., Ltd.
 *  Lennox Wu <lennox.wu@sunplusct.com>
 *  Chen Liqin <liqin.chen@sunplusct.com>
 * Copyright (C) 2012 Regents of the University of California
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 */


#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/perf_event.h>
#include <linux/signal.h>
#include <linux/uaccess.h>

#include <asm/pgalloc.h>
#include <asm/ptrace.h>
#include <utils/puts.h>

/*
 * This routine handles page faults.  It determines the address and the
 * problem, and then passes it off to one of the appropriate routines.
 */

uint32_t shadow_tlb[256], shadow_tlb_phys_and_flags[256];

#pragma GCC push_options
#pragma GCC optimize ("O0")
asmlinkage void vexriscv_mmu_map(uint32_t virt, uint32_t phys_and_flags, uint32_t location) {
   asm("");
   LOAD_VIRTUAL(0, 10, 0);
   LOAD_TLB(0, 12, 11);
   shadow_tlb[location] = virt;
   shadow_tlb_phys_and_flags[location] = phys_and_flags;
}

asmlinkage void vexriscv_dcache_clear(uint32_t line) {
	FLUSH_DCACHE_LINE(0, 10);
}



asmlinkage uint32_t direct_read(uint32_t addr) {
	uint32_t value;
	__asm__ __volatile__ (
		"lw %0, 0x700(%1)"
		: "=r" (value) :  "r" (addr));
	return value;
}

asmlinkage uint32_t direct_read_n(uint32_t addr) {
	uint32_t value;
	__asm__ __volatile__ (
		"lw %0, 0x600(%1)"
		: "=r" (value) :  "r" (addr));
	return value;
}


asmlinkage uint32_t direct_read_z(uint32_t addr) {
	uint32_t value;
	__asm__ __volatile__ (
		"lw %0, 0(%1)"
		: "=r" (value) :  "r" (addr));
	return value;
}

asmlinkage uint32_t direct_read_m800(uint32_t addr) {
	uint32_t value;
	__asm__ __volatile__ (
		"lw %0, -0x800(%1)"
		: "=r" (value) :  "r" (addr));
	return value;
}


asmlinkage uint32_t direct_read_m400(uint32_t addr) {
	uint32_t value;
	__asm__ __volatile__ (
		"lw %0, -0x400(%1)"
		: "=r" (value) :  "r" (addr));
	return value;
}



asmlinkage uint32_t direct_read_400(uint32_t addr) {
	uint32_t value;
	__asm__ __volatile__ (
		"lw %0, 0x400(%1)"
		: "=r" (value) :  "r" (addr));
	return value;
}


#pragma GCC pop_options

uint32_t page_location = 0;
uint32_t page_virt[PAGE_COUNT];
uint32_t page_phys[PAGE_COUNT];

asmlinkage void do_page_fault(struct pt_regs *regs)
{
	struct task_struct *tsk;
	struct vm_area_struct *vma;
	struct mm_struct *mm;
	unsigned long addr, cause;
	unsigned int flags = FAULT_FLAG_ALLOW_RETRY | FAULT_FLAG_KILLABLE;
	int code = SEGV_MAPERR;
	unsigned char cause_text[30];
	uint32_t sepc;
	int i;
	uint32_t a, b;

	vm_fault_t fault;

	cause = regs->scause;
	addr = regs->sbadaddr;
	sepc = regs->sepc;

	switch (cause) {
		case 0: sprintf(cause_text, "INS_ADDR_MISAL");
		        break;
		case 1:
		        sprintf(cause_text, "INS_ACC_FAULT");
			break;
		case 2:
		        sprintf(cause_text, "INS_ILLEGAL");
			break;
		case 3:
		        sprintf(cause_text, "BREAKPOINT");
		        break;
		case 5:
		        sprintf(cause_text, "LOAD_ACC_FAULT");
			break;
		case 7:
		        sprintf(cause_text, "STORE_ACC_FAULT");
		        break;
		case 8:
		        sprintf(cause_text, "ECALL");
			break;
		case 12:
		        sprintf(cause_text, "INS_PAGE_FAULT");
			break;
		case 13:
		        sprintf(cause_text, "LOAD_FAULT");
			break;
		case 15:
		        sprintf(cause_text, "STORE_FAULT/AMO");
			break;
		default:
		        sprintf(cause_text, "??????");
			break;

	}

	MSG(1, "Doing page fault @ 0x%08X, cause = 0x%X [%s], sepc = 0x%08X!", addr, cause, cause_text, sepc);

	tsk = current;
	mm = tsk->mm;

	/*
	 * Fault-in kernel-space virtual memory on-demand.
	 * The 'reference' page table is init_mm.pgd.
	 *
	 * NOTE! We MUST NOT take any locks for this case. We may
	 * be in an interrupt or a critical region, and should
	 * only copy the information from the master page table,
	 * nothing more.
	 */
	if (unlikely((addr >= VMALLOC_START) && (addr <= VMALLOC_END))) {
	DBGMSG("vmalloc_fault because addr=%X >= %X and <= %X!", addr, VMALLOC_START, VMALLOC_END);
		goto vmalloc_fault;
		}

	/* Enable interrupts if they were enabled in the parent context. */
	if (likely(regs->sstatus & SR_SPIE))
		local_irq_enable();

	/*
	 * If we're in an interrupt, have no user context, or are running
	 * in an atomic region, then we must not take the fault.
	 */
	if (unlikely(faulthandler_disabled() || !mm))
		goto no_context;

	if (user_mode(regs))
		flags |= FAULT_FLAG_USER;

	perf_sw_event(PERF_COUNT_SW_PAGE_FAULTS, 1, regs, addr);

retry:
	down_read(&mm->mmap_sem);
	vma = find_vma(mm, addr);
	if (unlikely(!vma))
		goto bad_area;
	if (likely(vma->vm_start <= addr))
		goto good_area;
	if (unlikely(!(vma->vm_flags & VM_GROWSDOWN)))
		goto bad_area;
	if (unlikely(expand_stack(vma, addr)))
		goto bad_area;

	/*
	 * Ok, we have a good vm_area for this memory access, so
	 * we can handle it.
	 */
good_area:
DBGMSG("Good area :)");
	code = SEGV_ACCERR;

	switch (cause) {
	case EXC_INST_PAGE_FAULT:
		if (!(vma->vm_flags & VM_EXEC))
			goto bad_area;
		break;
	case EXC_LOAD_PAGE_FAULT:
		if (!(vma->vm_flags & VM_READ))
			goto bad_area;
		break;
	case EXC_STORE_PAGE_FAULT:
		if (!(vma->vm_flags & VM_WRITE))
			goto bad_area;
		flags |= FAULT_FLAG_WRITE;
		break;
	default:
		panic("%s: unhandled cause %lu", __func__, cause);
	}

	/*
	 * If for any reason at all we could not handle the fault,
	 * make sure we exit gracefully rather than endlessly redo
	 * the fault.
	 */
	DBGMSG("Handling vaddr = %X, flags = %X...", addr, flags);
	fault = handle_mm_fault(vma, addr, flags);

	/*
	 * If we need to retry but a fatal signal is pending, handle the
	 * signal first. We do not need to release the mmap_sem because it
	 * would already be released in __lock_page_or_retry in mm/filemap.c.
	 */
	if ((fault & VM_FAULT_RETRY) && fatal_signal_pending(tsk))
		return;

	if (unlikely(fault & VM_FAULT_ERROR)) {
		if (fault & VM_FAULT_OOM)
			goto out_of_memory;
		else if (fault & VM_FAULT_SIGBUS)
			goto do_sigbus;
			DBGMSG("BUG!");
		BUG();
	}

	/*
	 * Major/minor page fault accounting is only done on the
	 * initial attempt. If we go through a retry, it is extremely
	 * likely that the page will be found in page cache at that point.
	 */
	if (flags & FAULT_FLAG_ALLOW_RETRY) {
		if (fault & VM_FAULT_MAJOR) {
			tsk->maj_flt++;
			perf_sw_event(PERF_COUNT_SW_PAGE_FAULTS_MAJ,
				      1, regs, addr);
		} else {
			tsk->min_flt++;
			perf_sw_event(PERF_COUNT_SW_PAGE_FAULTS_MIN,
				      1, regs, addr);
		}
		if (fault & VM_FAULT_RETRY) {
			/*
			 * Clear FAULT_FLAG_ALLOW_RETRY to avoid any risk
			 * of starvation.
			 */
			flags &= ~(FAULT_FLAG_ALLOW_RETRY);
			flags |= FAULT_FLAG_TRIED;

			/*
			 * No need to up_read(&mm->mmap_sem) as we would
			 * have already released it in __lock_page_or_retry
			 * in mm/filemap.c.
			 */
			goto retry;
		}
	}
      DBGMSG("going out to 0x%08X", regs->sepc);
    for (i = 0; i < 256; i++) {
    	if (shadow_tlb[i] == 0 && shadow_tlb_phys_and_flags[i] == 0)
    		continue;
    	DBGMSG("shadow tlb [%d] virt=%08x phys_and_flags=%08x", i, shadow_tlb[i], shadow_tlb_phys_and_flags[i]);
    }
	up_read(&mm->mmap_sem);
	return;

	/*
	 * Something tried to access memory that isn't in our memory map.
	 * Fix it, but check if it's kernel or user first.
	 */
bad_area:
      MSG(1, "We are in bad area");
	up_read(&mm->mmap_sem);
	/* User mode accesses just cause a SIGSEGV */
	if (user_mode(regs)) {
	DBGMSG("We are user, doing SIGSEGV");
	//*((uint32_t*)0x5001e604) = 0xdeadbeef;
	for (i = 0; i < 32; i+=4) {
		DBGMSG("5001e6%02x == %08x c00136%02x == %08x", i, *((uint8_t*)(0x5001e600 + i)), i, *((uint32_t*)(0xc0013600 + i)));

		DBGMSG("a (xb) = %08x %08x", direct_read(0x5001DF00 + i), direct_read(0xc0012f00 + i));
		DBGMSG("a (nxb) = %08x", direct_read_n(0x5001E000 + i));
		DBGMSG("a (zxb) = %08x", direct_read_z(0x5001E600 + i));
		DBGMSG("a (ze0) = %08x", direct_read_z(0x5001E000 + i));

		DBGMSG("a (m800) = %08x", direct_read_m800(0x5001EE00 + i));
		DBGMSG("a (m400) = %08x", direct_read_m400(0x5001EA00 + i));

		DBGMSG("a (5xb) = %08x", direct_read_400(0x5001E200 + i));

		/*	
		DBGMSG("500006%02x == %08x c002a6%02x == %08x", i, *((uint32_t*)(0x50000600 + i)), i, *((uint32_t*)(0xc002a600 + i)));
		DBGMSG("500026%02x == %08x c002c6%02x == %08x", i, *((uint32_t*)(0x50002600 + i)), i, *((uint32_t*)(0xc002c600 + i)));
		DBGMSG("500036%02x == %08x c002d6%02x == %08x", i, *((uint32_t*)(0x50003600 + i)), i, *((uint32_t*)(0xc002d600 + i)));
		DBGMSG("5000d6%02x == %08x c00146%02x == %08x", i, *((uint32_t*)(0x5000d600 + i)), i, *((uint32_t*)(0xc0014600 + i)));
		DBGMSG("5001df%02x == %08x c0463f%02x == %08x", i, *((uint32_t*)(0x5001df00 + i)), i, *((uint32_t*)(0xc0463f00 + i)));
*/

	}
		do_trap(regs, SIGSEGV, code, addr, tsk);
		return;
	} else DBGMSG("We were not in user!!!");

no_context:
         DBGMSG("No contex, we are going to die!");
	/* Are we prepared to handle this kernel fault? */
	if (fixup_exception(regs))
		return;

	/*
	 * Oops. The kernel tried to access some bad page. We'll have to
	 * terminate things with extreme prejudice.
	 */
	bust_spinlocks(1);
	pr_alert("Unable to handle kernel %s at virtual address " REG_FMT "\n",
		(addr < PAGE_SIZE) ? "NULL pointer dereference" :
		"paging request", addr);
	die(regs, "Oops");
	do_exit(SIGKILL);

	/*
	 * We ran out of memory, call the OOM killer, and return the userspace
	 * (which will retry the fault, or kill us if we got oom-killed).
	 */
out_of_memory:
	up_read(&mm->mmap_sem);
	if (!user_mode(regs))
		goto no_context;
	pagefault_out_of_memory();
	return;

do_sigbus:
	up_read(&mm->mmap_sem);
	/* Kernel mode? Handle exceptions or die */
	if (!user_mode(regs))
		goto no_context;
	do_trap(regs, SIGBUS, BUS_ADRERR, addr, tsk);
	return;

vmalloc_fault:
	{
	DBG();
		pgd_t *pgd, *pgd_k;
		pud_t *pud, *pud_k;
		p4d_t *p4d, *p4d_k;
		pmd_t *pmd, *pmd_k;
		pte_t *pte_k;
		int index;
DBG();
		if (user_mode(regs))
			goto bad_area;
DBG();
		/*
		 * Synchronize this task's top level page-table
		 * with the 'reference' page table.
		 *
		 * Do _not_ use "tsk->active_mm->pgd" here.
		 * We might be inside an interrupt in the middle
		 * of a task switch.
		 *
		 * Note: Use the old spbtr name instead of using the current
		 * satp name to support binutils 2.29 which doesn't know about
		 * the privileged ISA 1.10 yet.
		 */
		index = pgd_index(addr);
		pgd = (pgd_t *)pfn_to_virt(csr_read(sptbr)) + index;
		pgd_k = init_mm.pgd + index;

DBG();
		if (!pgd_present(*pgd_k))
			goto no_context;
DBG();
		set_pgd(pgd, *pgd_k);

DBG();
		p4d = p4d_offset(pgd, addr);
		p4d_k = p4d_offset(pgd_k, addr);
DBG();
		if (!p4d_present(*p4d_k))
			goto no_context;

DBG();
		pud = pud_offset(p4d, addr);
DBG();
		pud_k = pud_offset(p4d_k, addr);
		if (!pud_present(*pud_k))
			goto no_context;

DBG();
		/*
		 * Since the vmalloc area is global, it is unnecessary
		 * to copy individual PTEs
		 */
		pmd = pmd_offset(pud, addr);
		pmd_k = pmd_offset(pud_k, addr);
		if (!pmd_present(*pmd_k))
			goto no_context;
		set_pmd(pmd, *pmd_k);

DBG();
		/*
		 * Make sure the actual PTE exists as well to
		 * catch kernel vmalloc-area accesses to non-mapped
		 * addresses. If we don't do this, this will just
		 * silently loop forever.
		 */
		pte_k = pte_offset_kernel(pmd_k, addr);
		if (!pte_present(*pte_k))
			goto no_context;

		uint32_t paddr = (((uint32_t)pte_val(*pte_k) >> 10) & 0xFFFFF);
		DBGMSG("Adding kernel mapping 0x%08X -> 0x%08X", addr, paddr << 12);
		vexriscv_mmu_map((addr >> 12) & 0xFFFFF, paddr | 0xF8000000, page_location++ % PAGE_COUNT);
		return;
	}
}
