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

#include <asm/thread_info.h>
#include <asm/asm-offsets.h>
#include <asm/asm.h>
#include <linux/init.h>
#include <linux/linkage.h>
#include <asm/thread_info.h>
#include <asm/page.h>
#include <asm/csr.h>

#define VEXRISCV_CSR_IRQ_MASK 0xBC0
#define VEXRISCV_CACHE_CONTROL 0xCC0

#define r_type_insn(_f7, _rs2, _rs1, _f3, _rd, _opc) .word (((_f7) << 25) | ((_rs2) << 20) | ((_rs1) << 15) | ((_f3) << 12) | ((_rd) << 7) | ((_opc) << 0))
#define LOAD_VIRTUAL(_rd, _rs1, _rs2 ) r_type_insn(0b0000000, _rs2, _rs1, 0b111, _rd, 0b0001111)
#define LOAD_TLB(_rd, _rs1, _rs2 ) r_type_insn(0b0000001, _rs2, _rs1, 0b111, _rd, 0b0001111)

#define MMU_CTRL(_rd, _rs1, _rs2 ) r_type_insn(0b0000010, _rs2, _rs1, 0b111, _rd, 0b0001111)

__INIT
ENTRY(_start)
        /* Mask all interrupts */
        csrci mstatus, 8
        csrw VEXRISCV_CSR_IRQ_MASK, zero

        /* temporary stack */
        li sp, 0xD0000000

        /* Save hart ID and DTB physical address */
        mv s0, a0
        mv s1, a1

        call mtrap_init

_enter_supervisor:
	la t0, _start_further
	li t1, 0x0FFFFFFF
	and t0, t0, t1
	li t1, 0xC0000000
	or t0, t0, t1
	csrw mepc, t0
	csrr t0, mstatus
	li t1, ~(3<<11)
	and t0, t0, t1
	li t1,  (1<<11)
	or t0, t0, t1
	csrw mstatus, t0
	mret

_start_further:

        call relocate

	/* Load the global pointer */
.option push
.option norelax
	la gp, __global_pointer$
.option pop

	la sp, init_thread_union + THREAD_SIZE

	call setup_vm
	/* Initialize page tables and relocate to virtual addresses */

	call trap_init

        li a0, 1
        call mmu_ctrl
     
	/* Restore C environment */
	la tp, init_task
	sw s0, TASK_TI_CPU(tp)

	la sp, init_thread_union
	li a0, ASM_THREAD_SIZE
	add sp, sp, a0

	/* Start the kernel */
	mv a0, s0
	mv a1, s1

	call parse_dtb

	tail start_kernel

_end_loop: j _end_loop        

relocate:
	/* Relocate return address */
	li a1, PAGE_OFFSET
	la a0, _start
	sub a1, a1, a0
	add ra, ra, a1

	/* Point stvec to virtual address of intruction after satp write */
	la a0, 1f
	add a0, a0, a1
	csrw stvec, a0

	/* Compute satp for kernel page tables, but don't load it yet */
	la a2, swapper_pg_dir
	srl a2, a2, PAGE_SHIFT
	li a1, SATP_MODE
	or a2, a2, a1

	/*
	 * Load trampoline page directory, which will cause us to trap to
	 * stvec if VA != PA, or simply fall through if VA == PA
	 */
	la a0, trampoline_pg_dir
	srl a0, a0, PAGE_SHIFT
	or a0, a0, a1

	sfence.vma
	csrw sptbr, a0

.align 2
1:
       /* Reload the global pointer */
.option push
.option norelax
       la gp, __global_pointer$
.option pop

       /* Switch to kernel page tables */
       csrw sptbr, a2

       ret


.global mmu_ctrl                                                                                     
 mmu_ctrl: 
   MMU_CTRL(0, 0, 10)                                                                                 
   ret

END(_start)

__PAGE_ALIGNED_BSS
	/* Empty zero page */
	.balign PAGE_SIZE
