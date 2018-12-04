/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ARCH_M68K_CMPXCHG__
#define __ARCH_M68K_CMPXCHG__

#include <linux/irqflags.h>
#include <utils/puts.h>

struct __xchg_dummy { unsigned long a[100]; };
#define __xg(x) ((volatile struct __xchg_dummy *)(x))

extern unsigned long __invalid_xchg_size(unsigned long, volatile void *, int);

static inline unsigned long __xchg(unsigned long x, volatile void * ptr, int size)
{
	unsigned long flags, tmp;
	local_irq_save(flags);

	switch (size) {
	case 1:
		tmp = *(u8 *)ptr;
		*(u8 *)ptr = x;
		x = tmp;
		break;
	case 2:
		tmp = *(u16 *)ptr;
		*(u16 *)ptr = x;
		x = tmp;
		break;
	case 4:
		tmp = *(u32 *)ptr;
		*(u32 *)ptr = x;
		x = tmp;
		break;
	default:
		tmp = __invalid_xchg_size(x, ptr, size);
		break;
	}

	local_irq_restore(flags);
	return x;
}

#define xchg(ptr,x) ((__typeof__(*(ptr)))__xchg((unsigned long)(x),(ptr),sizeof(*(ptr))))

#include <asm-generic/cmpxchg-local.h>

#define cmpxchg64_local(ptr, o, n) __cmpxchg64_local_generic((ptr), (o), (n))

extern unsigned long __invalid_cmpxchg_size(volatile void *,
					    unsigned long, unsigned long, int);

/*
 * Atomic compare and exchange.  Compare OLD with MEM, if identical,
 * store NEW in MEM.  Return the initial value in MEM.  Success is
 * indicated by comparing RETURN with OLD.
 */

/*
 * cmpxchg_local and cmpxchg64_local are atomic wrt current CPU. Always make
 * them available.
 */
#define cmpxchg_local(ptr, o, n)				  	       \
	((__typeof__(*(ptr)))__cmpxchg_local_generic((ptr), (unsigned long)(o),\
			(unsigned long)(n), sizeof(*(ptr))))

#include <asm-generic/cmpxchg.h>

#endif /* __ARCH_M68K_CMPXCHG__ */
