/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_LITEX_H
#define _LINUX_LITEX_H

#include <linux/io.h>
#include <linux/types.h>
#include <linux/compiler_types.h>

#define LITEX_REG_SIZE             0x4
#define LITEX_SUBREG_SIZE          0x1
#define LITEX_SUBREG_SIZE_BIT      (LITEX_SUBREG_SIZE * 8)

// pointers to read and write functions
// set in soc controller driver
//
// note: or1k and vexriscv kernels use different CONFIG_GENERIC_IOMAP
//	setting and it changes the definition of io{read,write}32
//	functions so we are checking here to use correct definition
#ifndef CONFIG_GENERIC_IOMAP
extern unsigned int (*litex_read_reg)(const volatile void __iomem *addr);
extern void (*litex_write_reg)(u32 val, volatile void __iomem *addr);
#else
extern unsigned int (*litex_read_reg)(void __iomem *addr);
extern void (*litex_write_reg)(u32 val, void __iomem *addr);
#endif

// function implemented in
// drivers/soc/litex/litex_soc_controller.c
// to check if accessors are ready to use
// returns true if yes - false if not
//
// Important: all drivers that use functions from this header
// must check at the beginning of their probe()
// if litex_read_reg and litex_write_reg are initialized
// and return -EPROBE_DEFER when they are not
//
// example:
// if (!litex_check_accessors())
//	return -EPROBE_DEFER;
int litex_check_accessors(void);

// Helper functions for manipulating LiteX registers
static inline void litex_set_reg(void __iomem *reg, u32 reg_size, u32 val)
{
	u32 shifted_data, shift, i;

	for (i = 0; i < reg_size; ++i) {
		shift = ((reg_size - i - 1) * LITEX_SUBREG_SIZE_BIT);
		shifted_data = val >> shift;
		litex_write_reg(shifted_data, reg + (LITEX_REG_SIZE * i));
	}
}

static inline u32 litex_get_reg(void __iomem *reg, u32 reg_size)
{
	u32 shifted_data, shift, i;
	u32 result = 0;

	for (i = 0; i < reg_size; ++i) {
		shifted_data = litex_read_reg(reg + (LITEX_REG_SIZE * i));
		shift = ((reg_size - i - 1) * LITEX_SUBREG_SIZE_BIT);
		result |= (shifted_data << shift);
	}

	return result;
}

#endif /* _LINUX_LITEX_H */
