// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020 Antmicro Ltd. <www.antmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */
#include "clk-litex.h"
#include <linux/litex.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>

struct litex_drp_reg {
	u32 offset;
	u32 size;
};

struct litex_drp_reg drp[] = {
	{DRP_OF_RESET,  DRP_SIZE_RESET},
	{DRP_OF_READ,   DRP_SIZE_READ},
	{DRP_OF_WRITE,  DRP_SIZE_WRITE},
	{DRP_OF_DRDY,   DRP_SIZE_DRDY},
	{DRP_OF_ADR,    DRP_SIZE_ADR},
	{DRP_OF_DAT_W,  DRP_SIZE_DAT_W},
	{DRP_OF_DAT_R,  DRP_SIZE_DAT_R},
	{DRP_OF_LOCKED, DRP_SIZE_LOCKED},
};

struct litex_clk_range {
	u32 min;
	u32 max;
};

struct litex_clk_default {
	struct clk_duty duty;
	u32 phase;
	u32 freq;
};

struct litex_clk_glob_params {
	u64 freq;
	u32 div;
	u32 mul;
};

/* Divider configuration bits group */
struct litex_clk_div_params {
	u8 high_time;
	u8 low_time;
	u8 no_cnt;
	u8 edge;
};

/* Phase configuration bits group */
struct litex_clk_phase_params {
	u8 phase_mux;
	u8 delay_time;
	u8 mx;
};

/* Fractional configuration bits group */
struct litex_clk_frac_params {
	u8 frac_en;
	u8 frac;
	u8 phase_mux_f;
	u8 frac_wf_r;
	u8 frac_wf_f;
};

struct litex_clk_params {
	struct clk_duty duty;
	u32 phase;
	u32 freq;
	u32 period_off;
	u8 div;
};

struct litex_clk_timeout {
	u32 lock;
	u32 drdy;
};

struct litex_clk_device {
	void __iomem *base;
	struct clk_hw clk_hw;
	struct litex_clk_clkout *clkouts;	/* array of clock outputs */
	struct litex_clk_timeout timeout;	/* timeouts for wait functions*/
	struct litex_clk_glob_params g_config;	/* general MMCM settings */
	struct litex_clk_glob_params ts_g_config;/* settings to set*/
	struct litex_clk_range divclk;		/* divclk_divide_range */
	struct litex_clk_range clkfbout;	/* clkfbout_mult_frange */
	struct litex_clk_range vco;		/* vco_freq_range */
	u8 *update_clkout;			/* which clkout needs update */
	u32 sys_clk_freq;			/* input frequency */
	u32 vco_margin;
	u32 nclkout;
};

struct litex_clk_clkout_addr {
	u8 reg1;
	u8 reg2;
};

struct litex_clk_regs_addr {
	struct litex_clk_clkout_addr clkout[CLKOUT_MAX];
};

struct litex_clk_clkout_margin {
	u32 m;				/* margin factor scaled to integer */
	u32 exp;
};

struct litex_clk_clkout {
	void __iomem *base;
	struct clk_hw clk_hw;
	struct litex_clk_default def;		/* DTS defaults */
	struct litex_clk_params config;		/* real CLKOUT settings */
	struct litex_clk_params ts_config;	/* CLKOUT settings to set */
	struct litex_clk_div_params div;	/* CLKOUT configuration groups*/
	struct litex_clk_phase_params phase;
	struct litex_clk_frac_params frac;
	struct litex_clk_range clkout_div;	/* clkout_divide_range */
	struct litex_clk_clkout_margin margin;
	u32 id;
};

struct litex_clk_device *ldev;		/* global struct for whole driver */
struct litex_clk_clkout *clkouts;	/* clkout array for whole driver */

struct litex_clk_regs_addr litex_clk_regs_addr_init(void)
{
	struct litex_clk_regs_addr m;
	u32 i, addr;

	addr = CLKOUT0_REG1;
	for (i = 0; i <= CLKOUT_MAX; i++) {
		if (i == 5) {
		/*
		 *special case because CLKOUT5 have its reg addresses
		 *placed lower than other CLKOUTs
		 */
			m.clkout[5].reg1 = CLKOUT5_REG1;
			m.clkout[5].reg2 = CLKOUT5_REG2;
		} else {
			m.clkout[i].reg1 = addr;
			addr++;
			m.clkout[i].reg2 = addr;
			addr++;
		}
	}
	return m;
}

/*
 * This code is taken from:
 * https://github.com/torvalds/linux/blob/master/drivers/clk/clk-axi-clkgen.c
 *
 *	Copyright 2012-2013 Analog Devices Inc.
 *	Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * FIXME: make this code common
 */

/* MMCM loop filter lookup table */
static u32 litex_clk_lookup_filter(u32 glob_mul)
{
	switch (glob_mul) {
	case 0:
		return 0x01001990;
	case 1:
		return 0x01001190;
	case 2:
		return 0x01009890;
	case 3:
		return 0x01001890;
	case 4:
		return 0x01008890;
	case 5 ... 8:
		return 0x01009090;
	case 9 ... 11:
		return 0x01000890;
	case 12:
		return 0x08009090;
	case 13 ... 22:
		return 0x01001090;
	case 23 ... 36:
		return 0x01008090;
	case 37 ... 46:
		return 0x08001090;
	default:
		return 0x08008090;
	}
}

/* MMCM lock detection lookup table */
static const u32 litex_clk_lock_table[] = {
	0x060603e8, 0x060603e8, 0x080803e8, 0x0b0b03e8,
	0x0e0e03e8, 0x111103e8, 0x131303e8, 0x161603e8,
	0x191903e8, 0x1c1c03e8, 0x1f1f0384, 0x1f1f0339,
	0x1f1f02ee, 0x1f1f02bc, 0x1f1f028a, 0x1f1f0271,
	0x1f1f023f, 0x1f1f0226, 0x1f1f020d, 0x1f1f01f4,
	0x1f1f01db, 0x1f1f01c2, 0x1f1f01a9, 0x1f1f0190,
	0x1f1f0190, 0x1f1f0177, 0x1f1f015e, 0x1f1f015e,
	0x1f1f0145, 0x1f1f0145, 0x1f1f012c, 0x1f1f012c,
	0x1f1f012c, 0x1f1f0113, 0x1f1f0113, 0x1f1f0113,
};

/* Helper function for lock lookup table */
static u32 litex_clk_lookup_lock(u32 glob_mul)
{
	if (glob_mul < ARRAY_SIZE(litex_clk_lock_table))
		return litex_clk_lock_table[glob_mul];
	return 0x1f1f00fa;
}
/* End of copied code */

static inline struct litex_clk_device *clk_hw_to_litex_clk_device
						(struct clk_hw *clk_hw)
{
	return container_of(clk_hw, struct litex_clk_device, clk_hw);
}

static inline struct litex_clk_clkout *clk_hw_to_litex_clk_clkout
						(struct clk_hw *clk_hw)
{
	return container_of(clk_hw, struct litex_clk_clkout, clk_hw);
}

static inline void litex_clk_set_reg(struct clk_hw *clk_hw, u32 reg, u32 val)
{
	struct litex_clk_clkout *lcko = clk_hw_to_litex_clk_clkout(clk_hw);

	litex_set_reg(lcko->base + drp[reg].offset, drp[reg].size, val);
}

static inline u32 litex_clk_get_reg(struct clk_hw *clk_hw, u32 reg)
{
	struct litex_clk_clkout *lcko = clk_hw_to_litex_clk_clkout(clk_hw);

	return litex_get_reg(lcko->base + drp[reg].offset, drp[reg].size);
}

static inline void litex_clk_assert_reg(struct clk_hw *clk_hw, u32 reg)
{
	int assert = (1 << (drp[reg].size * BITS_PER_BYTE)) - 1;

	litex_clk_set_reg(clk_hw, reg, assert);
}

static inline void litex_clk_deassert_reg(struct clk_hw *clk_hw, u32 reg)
{
	litex_clk_set_reg(clk_hw, reg, ZERO_REG);
}

static int litex_clk_wait_lock(struct clk_hw *clk_hw)
{
	struct litex_clk_clkout *lcko;
	u32 timeout;

	lcko = clk_hw_to_litex_clk_clkout(clk_hw);
	timeout = ldev->timeout.lock;

	/*Waiting for LOCK signal to assert in MMCM*/
	while (!litex_clk_get_reg(clk_hw, DRP_LOCKED) && timeout) {
		timeout--;
		usleep_range(900, 1000);
	}
	if (timeout == 0) {
		pr_warn("CLKOUT%d: %s timeout", lcko->id, __func__);
		return -ETIME;
	}
	return 0;
}

static int litex_clk_wait_drdy(struct clk_hw *clk_hw)
{
	struct litex_clk_clkout *lcko;
	u32 timeout;

	lcko = clk_hw_to_litex_clk_clkout(clk_hw);
	timeout = ldev->timeout.drdy;

	/*Waiting for DRDY signal to assert in MMCM*/
	while (!litex_clk_get_reg(clk_hw, DRP_DRDY) && timeout) {
		timeout--;
		usleep_range(900, 1000);
	}
	if (timeout == 0) {
		pr_warn("CLKOUT%d: %s timeout", lcko->id, __func__);
		return -ETIME;
	}
	return 0;
}

/* Read value written in given internal MMCM register*/
static int litex_clk_get_DO(struct clk_hw *clk_hw, u8 clk_reg_addr, u16 *res)
{
	int ret;

	litex_clk_set_reg(clk_hw, DRP_ADR, clk_reg_addr);
	litex_clk_assert_reg(clk_hw, DRP_READ);

	litex_clk_deassert_reg(clk_hw, DRP_READ);
	ret = litex_clk_wait_drdy(clk_hw);
	if (ret != 0)
		return ret;

	*res = litex_clk_get_reg(clk_hw, DRP_DAT_R);

	return 0;
}

/* Get global divider and multiplier values and update global config */
static int litex_clk_get_global_divider(struct clk_hw *hw)
{
	int ret;
	u16 divreg, mult2;
	u8 low_time, high_time;

	ret = litex_clk_get_DO(hw, CLKFBOUT_REG2, &mult2);
	if (ret != 0)
		return ret;
	ret = litex_clk_get_DO(hw, DIV_REG, &divreg);
	if (ret != 0)
		return ret;

	if (mult2 & (NO_CNT_MASK << NO_CNT_POS)) {
		ldev->g_config.mul = 1;
	} else {
		u16 mult1;

		ret = litex_clk_get_DO(hw, CLKFBOUT_REG1, &mult1);
		if (ret != 0)
			return ret;
		low_time = mult1 & HL_TIME_MASK;
		high_time = (mult1 >> HIGH_TIME_POS) & HL_TIME_MASK;
		ldev->g_config.mul = low_time + high_time;
	}

	if (divreg & (NO_CNT_MASK << NO_CNT_DIVREG_POS)) {
		ldev->g_config.div = 1;
	} else {
		low_time = divreg & HL_TIME_MASK;
		high_time = (divreg >> HIGH_TIME_POS) & HL_TIME_MASK;
		ldev->g_config.div = low_time + high_time;
	}

	return 0;
}

static u64 litex_clk_calc_global_frequency(u32 mul, u32 div)
{
	u64 f;

	f = (u64)ldev->sys_clk_freq * (u64)mul;
	do_div(f, div);

	return f;
}

/* Calculate frequency with real global params and update global config */
static u64 litex_clk_get_real_global_frequency(struct clk_hw *hw)
{
	u64 f;

	litex_clk_get_global_divider(hw);
	f = litex_clk_calc_global_frequency(ldev->g_config.mul,
					    ldev->g_config.div);
	ldev->g_config.freq = f;
	ldev->ts_g_config.div = ldev->g_config.div;
	ldev->ts_g_config.mul = ldev->g_config.mul;
	ldev->ts_g_config.freq = ldev->g_config.freq;

	return f;
}

/* Return dividers of given CLKOUT */
static int litex_clk_get_clkout_divider(struct clk_hw *hw, u32 *divider,
							   u32 *fract_cnt)
{
	struct litex_clk_clkout *lcko = clk_hw_to_litex_clk_clkout(hw);
	struct litex_clk_regs_addr drp_addr = litex_clk_regs_addr_init();
	int ret;
	u16 div, frac;
	u8 clkout_nr = lcko->id;
	u8 low_time, high_time;

	ret = litex_clk_get_DO(hw, drp_addr.clkout[clkout_nr].reg1, &div);
	if (ret != 0)
		return ret;
	ret = litex_clk_get_DO(hw, drp_addr.clkout[clkout_nr].reg2, &frac);
	if (ret != 0)
		return ret;

	low_time = div & HL_TIME_MASK;
	high_time = (div >> HIGH_TIME_POS) & HL_TIME_MASK;
	*divider = low_time + high_time;
	*fract_cnt = (frac >> FRAC_POS) & FRAC_MASK;

	return 0;
}

/* Debug functions */
#ifdef DEBUG
static void litex_clk_print_general_regs(struct clk_hw *clk_hw)
{
	u16 power_reg, div_reg, clkfbout_reg1, clkfbout_reg2,
	lock_reg1, lock_reg2, lock_reg3, filt_reg1, filt_reg2;

	litex_clk_get_DO(clk_hw, POWER_REG, &power_reg);
	litex_clk_get_DO(clk_hw, DIV_REG, &div_reg);
	litex_clk_get_DO(clk_hw, CLKFBOUT_REG1, &clkfbout_reg1);
	litex_clk_get_DO(clk_hw, CLKFBOUT_REG1, &clkfbout_reg2);
	litex_clk_get_DO(clk_hw, LOCK_REG1, &lock_reg1);
	litex_clk_get_DO(clk_hw, LOCK_REG2, &lock_reg2);
	litex_clk_get_DO(clk_hw, LOCK_REG3, &lock_reg3);
	litex_clk_get_DO(clk_hw, FILT_REG1, &filt_reg1);
	litex_clk_get_DO(clk_hw, FILT_REG2, &filt_reg2);

	pr_debug("POWER REG:  0x%x\n", power_reg);
	pr_debug("DIV REG:    0x%x\n", div_reg);
	pr_debug("MUL REG1:   0x%x\n", clkfbout_reg1);
	pr_debug("MUL REG2:   0x%x\n", clkfbout_reg2);
	pr_debug("LOCK_REG1:  0x%x\n", lock_reg1);
	pr_debug("LOCK_REG2:  0x%x\n", lock_reg2);
	pr_debug("LOCK_REG3:  0x%x\n", lock_reg3);
	pr_debug("FILT_REG1:  0x%x\n", filt_reg1);
	pr_debug("FILT_REG2:  0x%x\n", filt_reg2);
}

static void litex_clk_print_clkout_regs(struct clk_hw *clk_hw, u8 clkout,
						      u8 reg1, u8 reg2)
{
	u16 clkout_reg1, clkout_reg2;
	int ret;

	ret = litex_clk_get_DO(clk_hw, reg1, &clkout_reg1);
	if (ret != 0)
		pr_debug("CLKOUT%u REG1: read error %d \n", clkout, ret);

	ret = litex_clk_get_DO(clk_hw, reg1, &clkout_reg2);
	if (ret != 0)
		pr_debug("CLKOUT%u REG2: read error %d \n", clkout, ret);

	pr_debug("CLKOUT%u REG1: 0x%x\n", clkout, clkout_reg1);
	pr_debug("CLKOUT%u REG2: 0x%x\n", clkout, clkout_reg2);
}

static void litex_clk_print_all_regs(struct clk_hw *clk_hw)
{
	struct litex_clk_regs_addr drp_addr = litex_clk_regs_addr_init();
	u32 i;

	litex_clk_print_general_regs(clk_hw);
	for (i = 0; i < ldev->nclkout; i++) {
		litex_clk_print_clkout_regs(clk_hw, i, drp_addr.clkout[i].reg1,
						       drp_addr.clkout[i].reg2);
	}
}

static void litex_clk_print_params(struct litex_clk_clkout *lcko)
{
	pr_debug("CLKOUT%d DUMP:\n", lcko->id);
	pr_debug("Defaults:\n");
	pr_debug("f: %u d: %u/%u p: %u\n",
		lcko->def.freq, lcko->def.duty.num,
		lcko->def.duty.den, lcko->def.phase);
	pr_debug("Config to set:\n");
	pr_debug("div: %u freq: %u duty: %u/%u phase: %u per_off: %u\n",
		lcko->ts_config.div, lcko->ts_config.freq,
		lcko->ts_config.duty.num, lcko->ts_config.duty.den,
		lcko->ts_config.phase, lcko->config.period_off);
	pr_debug("Config:\n");
	pr_debug("div: %u freq: %u duty: %u/%u phase: %u per_off: %u\n",
		lcko->config.div, lcko->config.freq,
		lcko->config.duty.num, lcko->config.duty.den,
		lcko->config.phase, lcko->config.period_off);
	pr_debug("Divide group:\n");
	pr_debug("e: %u ht: %u lt: %u nc: %u\n",
		lcko->div.edge, lcko->div.high_time,
		lcko->div.low_time, lcko->div.no_cnt);
	pr_debug("Frac group:\n");
	pr_debug("f: %u fen: %u fwff: %u fwfr: %u pmf: %u\n",
		lcko->frac.frac, lcko->frac.frac_en, lcko->frac.frac_wf_f,
		lcko->frac.frac_wf_r, lcko->frac.phase_mux_f);
	pr_debug("Phase group:\n");
	pr_debug("dt: %u pm: %u mx: %u\n",
		lcko->phase.delay_time, lcko->phase.phase_mux, lcko->phase.mx);
}

static void litex_clk_print_all_params(void)
{
	u32 c;

	pr_debug("Global Config to set:\n");
	pr_debug("freq: %llu mul: %u div: %u\n",
		ldev->ts_g_config.freq, ldev->ts_g_config.mul,
					ldev->ts_g_config.div);
	pr_debug("Global Config:\n");
	pr_debug("freq: %llu mul: %u div: %u\n",
		ldev->g_config.freq, ldev->g_config.mul, ldev->g_config.div);
	for (c = 0; c < ldev->nclkout; c++)
		litex_clk_print_params(&ldev->clkouts[c]);
}
#endif /* #ifdef DEBUG */

/* Returns raw value ready to be written into MMCM */
static inline u16 litex_clk_calc_DI(u16 DO_val, u16 mask, u16 bitset)
{
	u16 DI_val;

	DI_val = DO_val & mask;
	DI_val |= bitset;

	return DI_val;
}

/* Sets calculated DI value into DI DRP register */
static int litex_clk_set_DI(struct clk_hw *clk_hw, u16 DI_val)
{
	int ret;

	litex_clk_set_reg(clk_hw, DRP_DAT_W, DI_val);
	litex_clk_assert_reg(clk_hw, DRP_WRITE);
	litex_clk_deassert_reg(clk_hw, DRP_WRITE);
	ret = litex_clk_wait_drdy(clk_hw);
	if (ret != 0)
		return ret;
	return 0;
}

/*
 * Change register value as specified in arguments
 *
 * clk_hw:		hardware specific clock structure
 * mask:		preserve or zero MMCM register bits
 *			by selecting 1 or 0 on desired specific mask positions
 * bitset:		set those bits in MMCM register which are 1 in bitset
 * clk_reg_addr:	internal MMCM address of control register
 *
 */
static int litex_clk_change_value(struct clk_hw *clk_hw, u16 mask, u16 bitset,
							 u8 clk_reg_addr)
{
	u16 DO_val, DI_val;
	int ret;

	litex_clk_assert_reg(clk_hw, DRP_RESET);

	ret = litex_clk_get_DO(clk_hw, clk_reg_addr, &DO_val);
	if (ret != 0)
		return ret;
	DI_val = litex_clk_calc_DI(DO_val, mask, bitset);
	ret = litex_clk_set_DI(clk_hw, DI_val);
	if (ret != 0)
		return ret;
#ifdef DEBUG
	DI_val = litex_clk_get_reg(clk_hw, DRP_DAT_W);
#endif
	litex_clk_deassert_reg(clk_hw, DRP_DAT_W);
	litex_clk_deassert_reg(clk_hw, DRP_RESET);
	ret = litex_clk_wait_lock(clk_hw);
	if (ret != 0)
		return ret;

	pr_debug("set 0x%x under: 0x%x", DI_val, clk_reg_addr);

	return 0;
}

/*
 * Set register values for given CLKOUT
 *
 * clk_hw:		hardware specific clock structure
 * clkout_nr:		clock output number
 * mask_regX:		preserve or zero MMCM register X bits
 *			by selecting 1 or 0 on desired specific mask positions
 * bitset_regX:		set those bits in MMCM register X which are 1 in bitset
 *
 */
static int litex_clk_set_clock(struct clk_hw *clk_hw, u8 clkout_nr,
				       u16 mask_reg1, u16 bitset_reg1,
				       u16 mask_reg2, u16 bitset_reg2)
{
	struct litex_clk_regs_addr drp_addr = litex_clk_regs_addr_init();
	int ret;

	if (!(mask_reg2 == KEEP_REG_16 && bitset_reg2 == ZERO_REG)) {
		ret = litex_clk_change_value(clk_hw, mask_reg2, bitset_reg2,
					    drp_addr.clkout[clkout_nr].reg2);
		if (ret != 0)
			return ret;
	}
	if (!(mask_reg1 == KEEP_REG_16 && bitset_reg1 == ZERO_REG)) {
		ret = litex_clk_change_value(clk_hw, mask_reg1, bitset_reg1,
					drp_addr.clkout[clkout_nr].reg1);
		if (ret != 0)
			return ret;
	}

	return 0;
}

/* Set global divider for all CLKOUTs */
static int litex_clk_set_divreg(struct clk_hw *clk_hw)
{
	int ret;
	u8 no_cnt = 0, edge = 0, ht = 0, lt = 0, div = ldev->ts_g_config.div;
	u16 bitset = 0;

	if (div == 1) {
		no_cnt = 1;
	} else {
		ht = div / 2;
		lt = ht;
		edge = div % 2;
		if (edge)
			lt += edge;
	}

	bitset = (edge << EDGE_DIVREG_POS)	|
		 (no_cnt << NO_CNT_DIVREG_POS)	|
		 (ht << HIGH_TIME_POS)		|
		 (lt << LOW_TIME_POS);

	ret = litex_clk_change_value(clk_hw, KEEP_IN_DIV, bitset, DIV_REG);
	if (ret != 0)
		return ret;

	ldev->g_config.div = div;
	pr_debug("Global divider set to %u", div);

	return 0;
}

/* Set global multiplier for all CLKOUTs */
static int litex_clk_set_mulreg(struct clk_hw *clk_hw)
{
	int ret;
	u8 no_cnt = 0, edge = 0, ht = 0, lt = 0, mul = ldev->ts_g_config.mul;
	u16 bitset1 = 0;

	if (mul == 1) {
		no_cnt = 1;
	} else {
		ht = mul / 2;
		lt = ht;
		edge = mul % 2;
		if (edge)
			lt += edge;
	}

	bitset1 = (ht << HIGH_TIME_POS) |
		  (lt << LOW_TIME_POS);

	if (edge || no_cnt) {
		u16 bitset2 = (edge << EDGE_POS)	|
			      (no_cnt << NO_CNT_POS);

		ret = litex_clk_change_value(clk_hw, KEEP_IN_MUL2,
					    bitset2, CLKFBOUT_REG2);
		if (ret != 0)
			return ret;
	}

	ret = litex_clk_change_value(clk_hw, KEEP_IN_MUL1,
				    bitset1, CLKFBOUT_REG1);
	if (ret != 0)
		return ret;

	ldev->g_config.mul = mul;
	pr_debug("Global multiplier set to %u", mul);

	return 0;
}

static int litex_clk_set_filt(struct clk_hw *hw)
{
	u32 filt, mul;
	int ret;

	mul = ldev->g_config.mul;
	filt = litex_clk_lookup_filter(mul - 1);

	ret = litex_clk_change_value(hw, FILT_MASK, filt >> 16, FILT_REG1);
	if (ret != 0)
		return ret;
	ret = litex_clk_change_value(hw, FILT_MASK, filt, FILT_REG2);
	if (ret != 0)
		return ret;
	return 0;
}

static int litex_clk_set_lock(struct clk_hw *hw)
{
	u32 lock, mul;
	int ret;

	mul = ldev->g_config.mul;

	lock = litex_clk_lookup_lock(mul - 1);

	ret = litex_clk_change_value(hw, LOCK1_MASK, lock & 0x3ff, LOCK_REG1);
	if (ret != 0)
		return ret;
	ret = litex_clk_change_value(hw, LOCK23_MASK,
			(((lock >> 16) & 0x1f) << 10) | 0x1, LOCK_REG2);
	if (ret != 0)
		return ret;
	ret = litex_clk_change_value(hw, LOCK23_MASK,
			(((lock >> 24) & 0x1f) << 10) | 0x3e9, LOCK_REG3);
	if (ret != 0)
		return ret;
	return 0;
}

/* Set all multiplier-related regs: mul, filt and lock regs */
static int litex_clk_set_mul(struct clk_hw *clk_hw)
{
	int ret;

	ret = litex_clk_set_mulreg(clk_hw);
	if (ret != 0)
		return ret;
	ret = litex_clk_set_filt(clk_hw);
	if (ret != 0)
		return ret;
	ret = litex_clk_set_lock(clk_hw);
	if (ret != 0)
		return ret;
	return 0;
}

static int litex_clk_set_both_globs(struct clk_hw *clk_hw)
{
	/*
	 * we need to check what change first to prevent
	 * getting our VCO_FREQ out of possible range
	 */
	u64 vco_freq;
	int ret;

	/* div-first case */
	vco_freq = litex_clk_calc_global_frequency(
					ldev->g_config.mul,
					ldev->ts_g_config.div);
	if (vco_freq > ldev->vco.max || vco_freq < ldev->vco.min) {
		/* div-first not safe */
		vco_freq = litex_clk_calc_global_frequency(
					ldev->ts_g_config.mul,
					ldev->g_config.div);
		if (vco_freq > ldev->vco.max || vco_freq < ldev->vco.min) {
			/* mul-first not safe */
			ret = litex_clk_set_divreg(clk_hw);
			/* Ignore timeout because we expect that to happen */
			if (ret != -ETIME && ret != 0)
				return ret;
			ret = litex_clk_set_mul(clk_hw);
			if (ret != 0)
				return ret;
		} else {
			/* mul-first safe */
			ret = litex_clk_set_mul(clk_hw);
			if (ret != 0)
				return ret;
			ret = litex_clk_set_divreg(clk_hw);
			if (ret != 0)
				return ret;
		}
	} else {
		/* div-first safe */
		ret = litex_clk_set_divreg(clk_hw);
		if (ret != 0)
			return ret;
		ret = litex_clk_set_mul(clk_hw);
		if (ret != 0)
			return ret;
	}
	return 0;
}

/* Set global divider, multiplier, filt and lock values */
static int litex_clk_set_globs(void)
{
	struct clk_hw *clk_hw = &ldev->clk_hw;
	int ret;
	u8 set_div = 0,
	   set_mul = 0;

	set_div = ldev->ts_g_config.div != ldev->g_config.div;
	set_mul = ldev->ts_g_config.mul != ldev->g_config.mul;

	if (set_div || set_mul) {
		if (set_div && set_mul) {
			ret = litex_clk_set_both_globs(clk_hw);
			if (ret != 0)
				return ret;
		} else if (set_div) {
			/* set divider only */
			ret = litex_clk_set_divreg(clk_hw);
			if (ret != 0)
				return ret;
		} else {
			/* set multiplier only */
			ret = litex_clk_set_mul(clk_hw);
			if (ret != 0)
				return ret;
		}
		ldev->g_config.freq = ldev->ts_g_config.freq;
	}
	return 0;
}

/* Round scaled value*/
static inline u32 litex_round(u32 val, u32 mod)
{
	if (val % mod > mod / 2)
		return val / mod + 1;
	return val / mod;
}

/*
 *	Duty Cycle
 */

/* Returns accurate duty ratio of given clkout*/
int litex_clk_get_duty_cycle(struct clk_hw *hw, struct clk_duty *duty)
{
	struct litex_clk_clkout *lcko = clk_hw_to_litex_clk_clkout(hw);
	struct litex_clk_regs_addr drp_addr = litex_clk_regs_addr_init();
	int ret;
	u32 divider;
	u16 clkout_reg1, clkout_reg2;
	u8 clkout_nr, high_time, edge, no_cnt, frac_en, frac_cnt;

	clkout_nr = lcko->id;
	/* Check if divider is off */
	ret = litex_clk_get_DO(hw, drp_addr.clkout[clkout_nr].reg2,
						      &clkout_reg2);
	if (ret != 0)
		return ret;

	edge = (clkout_reg2 >> EDGE_POS) & EDGE_MASK;
	no_cnt = (clkout_reg2 >> NO_CNT_POS) & NO_CNT_MASK;
	frac_en = (clkout_reg2 >> FRAC_EN_POS) & FRAC_EN_MASK;
	frac_cnt = (clkout_reg2 >> FRAC_POS) & FRAC_MASK;

	/* get duty 50% when divider is off or fractional is enabled */
	if (no_cnt || (frac_en && frac_cnt)) {
		duty->num = 1;
		duty->den = 2;
		return 0;
	}

	ret = litex_clk_get_DO(hw, drp_addr.clkout[clkout_nr].reg1,
						      &clkout_reg1);
	if (ret != 0)
		return ret;

	divider = clkout_reg1 & HL_TIME_MASK;
	high_time = (clkout_reg1 >> HIGH_TIME_POS) & HL_TIME_MASK;
	divider += high_time;

	/* Scaling to consider edge control bit */
	duty->num = high_time * 10 + edge * 5;
	duty->den = (divider + edge) * 10;

	return 0;
}

/* Calculates duty cycle for given ratio in percent, 1% accuracy */
static inline u8 litex_clk_calc_duty_percent(struct clk_duty *duty)
{
	u32 div, duty_ratio, ht;

	ht = duty->num;
	div = duty->den;
	duty_ratio = ht * 10000 / div;

	return (u8)litex_round(duty_ratio, 100);
}

/*
 * Calculate necessary values for setting duty cycle in fractional mode
 *
 * This function is based on Xilinx Unified Simulation Library Component
 * Advanced Mixed Mode Clock Manager (MMCM) taken from Xilinx Vivado
 * found in: <Vivado-dir>/data/verilog/src/unisims/MMCME2_ADV.v
 *
 */
static int litex_clk_calc_duty_fract(struct clk_hw *hw, int calc_new)
{
	struct litex_clk_clkout *lcko = clk_hw_to_litex_clk_clkout(hw);
	int even_part_high, even_part_low, odd, odd_and_frac;
	u8 high_duty, divider, frac;

	if (calc_new) {
		high_duty = 50;
	} else {
		struct clk_duty duty;

		litex_clk_get_duty_cycle(hw, &duty);
		high_duty = litex_clk_calc_duty_percent(&duty);
	}

	divider = lcko->config.div;
	frac = lcko->frac.frac;

			/* divider / 2 */
	even_part_high = divider >> 1;
	even_part_low = even_part_high;
	odd = divider - even_part_high - even_part_low;
	odd_and_frac = (8 * odd) + frac;

	lcko->div.low_time = even_part_high;
	if (odd_and_frac <= ODD_AND_FRAC)
		lcko->div.low_time--;

	lcko->div.high_time = even_part_low;
	if (odd_and_frac <= EVEN_AND_FRAC)
		lcko->div.high_time--;

	lcko->div.edge = divider % 2;

	lcko->frac.frac_wf_f = 0;
	if (((odd_and_frac >= 2) && (odd_and_frac <= ODD_AND_FRAC)) ||
				   ((frac == 1) && (divider == 2)))
		lcko->frac.frac_wf_f++;

	lcko->frac.frac_wf_r = 0;
	if ((odd_and_frac >= 1) && (odd_and_frac <= EVEN_AND_FRAC))
		lcko->frac.frac_wf_r++;

	return 0;
}
/* End of Vivado based code */

/* Calculate necessary values for setting duty cycle in normal mode */
static int litex_clk_calc_duty_normal(struct clk_hw *hw, int calc_new)
{
	struct litex_clk_clkout *lcko = clk_hw_to_litex_clk_clkout(hw);
	struct clk_duty duty;
	u32 ht_aprox, synthetic_duty, delta_d, min_d;
	u8 high_time_it, edge_it, high_duty,
	   divider = lcko->config.div;

	if (calc_new)
		duty = lcko->ts_config.duty;
	else
		litex_clk_get_duty_cycle(hw, &duty);

	high_duty = litex_clk_calc_duty_percent(&duty);
	min_d = INT_MAX;
	/* check if duty is available to set */
	ht_aprox = high_duty * divider;

	if (ht_aprox > ((HIGH_LOW_TIME_REG_MAX * 100) + 50) ||
		       ((HIGH_LOW_TIME_REG_MAX * 100) + 50) <
			(divider * 100) - ht_aprox)
		return -EINVAL;

	/* to prevent high_time == 0 or low_time == 0 */
	for (high_time_it = 1; high_time_it < divider; high_time_it++) {
		for (edge_it = 0; edge_it < 2; edge_it++) {
			synthetic_duty = (high_time_it * 100 + 50 * edge_it) /
								    divider;
			delta_d = abs(synthetic_duty - high_duty);
			/* check if low_time won't be above acceptable range */
			if (delta_d < min_d && (divider - high_time_it) <=
						  HIGH_LOW_TIME_REG_MAX) {
				min_d = delta_d;
				lcko->div.high_time = high_time_it;
				lcko->div.low_time = divider - high_time_it;
				lcko->div.edge = edge_it;
				lcko->config.duty.num = high_time_it * 100 + 50
								     * edge_it;
				lcko->config.duty.den = divider * 100;
			}
		}
	}
	/*
	 * Calculating values in normal mode,
	 * clear control bits of fractional part
	 */
	lcko->frac.frac_wf_f = 0;
	lcko->frac.frac_wf_r = 0;

	return 0;
}

/* Calculates duty high_time for given divider and ratio */
static inline int litex_clk_calc_duty_high_time(struct clk_hw *hw,
						struct clk_duty *duty,
						   u32 divider)
{
	u32 high_duty;

	high_duty = litex_clk_calc_duty_percent(duty) * divider;

	return litex_round(high_duty, 100);
}

/* Set duty cycle with given ratio */
int litex_clk_set_duty_cycle(struct clk_hw *hw, struct clk_duty *duty)
{
	struct litex_clk_clkout *lcko = clk_hw_to_litex_clk_clkout(hw);
	u16 bitset1, bitset2;
	u8 clkout_nr = lcko->id,
	   *edge = &lcko->div.edge,
	   *high_time = &lcko->div.high_time,
	    high_duty = litex_clk_calc_duty_percent(duty),
	   *low_time = &lcko->div.low_time;

	if (lcko->frac.frac == 0) {
		int ret;

		lcko->ts_config.duty = *duty;
		pr_debug("set_duty: %u/%u", duty->num, duty->den);
		ret = litex_clk_calc_duty_normal(hw, true);
		if (ret != 0) {
			pr_err("CLKOUT%d: cannot set %d%% duty cycle for that frequency",
				clkout_nr, high_duty);
			return ret;
		}
	} else if (high_duty == 50) {
		return 0;
	} else {
		pr_err("CLKOUT%d: cannot set duty cycle when fractional divider enabled!",
			clkout_nr);
		return -EACCES;
	}

	bitset1 = (*high_time << HIGH_TIME_POS) |
		  (*low_time << LOW_TIME_POS);
	bitset2 = (*edge << EDGE_POS);

	pr_debug("SET DUTY CYCLE: e:%u ht:%u lt:%u\nbitset1: 0x%x bitset2: 0x%x",
		*edge, *high_time, *low_time, bitset1, bitset2);

	return litex_clk_set_clock(hw, clkout_nr, REG1_DUTY_MASK, bitset1,
						  REG2_DUTY_MASK, bitset2);
}

/*
 *	Phase
 */

/*
 * Calculate necessary values for setting phase in fractional mode
 *
 * This function is based on Xilinx Unified Simulation Library Component
 * Advanced Mixed Mode Clock Manager (MMCM) taken from Xilinx Vivado
 * found in: <Vivado-dir>/data/verilog/src/unisims/MMCME2_ADV.v
 *
 */
static int litex_clk_calc_phase_fract(struct clk_hw *hw)
{
	struct litex_clk_clkout *lcko = clk_hw_to_litex_clk_clkout(hw);
	int phase, dt_calc, dt, a_per_in_octets, a_phase_in_cycles,
	    pm_rise_frac, pm_rise_frac_filtered, pm_fall,
	    pm_fall_frac, pm_fall_frac_filtered;
	u32 phase_offset, divider, frac;

	phase_offset = lcko->ts_config.phase;
	divider = lcko->config.div;
	frac = lcko->frac.frac;

	phase = phase_offset * 1000;
	a_per_in_octets = (8 * divider) + frac;
	a_phase_in_cycles = (phase + 10) * a_per_in_octets / 360000;

	if ((a_phase_in_cycles & FULL_BYTE) != 0)
		pm_rise_frac = (a_phase_in_cycles & PHASE_MUX_MAX);
	else
		pm_rise_frac = 0;

	dt_calc = a_phase_in_cycles / 8;
	dt = dt_calc & FULL_BYTE;

	pm_rise_frac_filtered = pm_rise_frac;
	if (pm_rise_frac >= 8)
		pm_rise_frac_filtered -= 8;

	pm_fall = ((divider % 2) << 2) + ((frac >> 1) & TWO_LSBITS);
	pm_fall_frac = pm_fall + pm_rise_frac;
	pm_fall_frac_filtered = pm_fall + pm_rise_frac -
					 (pm_fall_frac & F_FRAC_MASK);

	/* keep only the lowest bits to fit in control registers bit groups */
	lcko->phase.delay_time = dt % (1 << 6);
	lcko->phase.phase_mux = pm_rise_frac_filtered % (1 << 3);
	lcko->phase.mx = 0;
	lcko->frac.phase_mux_f = pm_fall_frac_filtered % (1 << 3);

	return 0;
}
/* End of Vivado based code */

/* Calculate necessary values for setting phase in normal mode */
static int litex_clk_calc_phase_normal(struct clk_hw *hw)
{
	struct litex_clk_clkout *lcko = clk_hw_to_litex_clk_clkout(hw);
	u64 period_buff;
	u32 post_glob_div_f, global_period,  clkout_period,
	    *period_off = &lcko->ts_config.period_off;
	u8 divider = lcko->config.div;
	/* ps unit */

	post_glob_div_f = (u32)litex_clk_get_real_global_frequency(hw);
	period_buff = PICOS_IN_SEC;
	do_div(period_buff, post_glob_div_f);
	global_period = (u32)period_buff;
	clkout_period = global_period * divider;

	if (lcko->ts_config.phase != 0) {
		int synthetic_phase, delta_p, min_p;
		u8 d_t, p_m;

		*period_off = litex_round(clkout_period * (*period_off), 10000);

		if (*period_off / global_period > DELAY_TIME_MAX)
			return -EINVAL;

		min_p = INT_MAX;
		/* Delay_time: (0-63) */
		for (d_t = 0; d_t <= DELAY_TIME_MAX; d_t++) {
			/* phase_mux: (0-7) */
			for (p_m = 0; p_m <= PHASE_MUX_MAX; p_m++) {
				synthetic_phase = (d_t * global_period) +
				((p_m * ((global_period * 100) / 8) / 100));

				delta_p = abs(synthetic_phase - *period_off);
				if (delta_p < min_p) {
					min_p = delta_p;
					lcko->phase.phase_mux = p_m;
					lcko->phase.delay_time = d_t;
					lcko->config.period_off = synthetic_phase;
				}
			}
		}
	} else {
		/* Don't change phase offset*/
		lcko->phase.phase_mux = 0;
		lcko->phase.delay_time = 0;
	}
	/*
	 * Calculating values in normal mode,
	 * fractional control bits need to be zero
	 */
	lcko->frac.phase_mux_f = 0;

	return 0;
}

/* Convert phase offset to positive lower than 360 deg. and calculate period */
static int litex_clk_prepare_phase(struct clk_hw *hw)
{
	struct litex_clk_clkout *lcko = clk_hw_to_litex_clk_clkout(hw);
	u32 *phase = &lcko->ts_config.phase;

	*phase %= 360;

	if (*phase < 0)
		*phase += 360;

	lcko->ts_config.period_off = ((*phase * 10000) / 360);

	return 0;
}

/* Calculate necessary values for setting phase */
static int litex_clk_calc_phase(struct clk_hw *hw)
{
	struct litex_clk_clkout *lcko = clk_hw_to_litex_clk_clkout(hw);

	litex_clk_prepare_phase(hw);

	if (lcko->id == 0 && lcko->frac.frac > 0)
		return litex_clk_calc_phase_fract(hw);
	return litex_clk_calc_phase_normal(hw);
}

/* Returns phase-specific values of given clock output */
static int litex_clk_get_phase_data(struct clk_hw *hw, u8 *phase_mux,
				       u8 *delay_time, u8 *phase_mux_f)
{
	struct litex_clk_clkout *lcko = clk_hw_to_litex_clk_clkout(hw);
	struct litex_clk_regs_addr drp_addr = litex_clk_regs_addr_init();
	int ret;
	u16 r1, r2;
	u8 clkout_nr = lcko->id;

	ret = litex_clk_get_DO(hw, drp_addr.clkout[clkout_nr].reg1, &r1);
	if (ret != 0)
		return ret;
	ret = litex_clk_get_DO(hw, drp_addr.clkout[clkout_nr].reg2, &r2);
	if (ret != 0)
		return ret;

	*phase_mux = (r1 >> PHASE_MUX_POS) & PHASE_MUX_MASK;
	*delay_time = (r2 >> DELAY_TIME_POS) & HL_TIME_MASK;

	if (clkout_nr == 0) {
		ret = litex_clk_get_DO(hw, CLKOUT5_REG2, &r2);
		if (ret != 0)
			return ret;
		*phase_mux_f = (r2 >> PHASE_MUX_F_POS) & PHASE_MUX_F_MASK;
	} else {
		*phase_mux_f = 0;
	}

	return 0;
}

/* Returns phase of given clock output in degrees */
int litex_clk_get_phase(struct clk_hw *hw)
{
	u64 period_buff;
	u32 divider = 0, fract_cnt, post_glob_div_f,
	    pm, global_period, clkout_period, period;
	u8 phase_mux = 0, delay_time = 0, phase_mux_f;

	litex_clk_get_phase_data(hw, &phase_mux, &delay_time, &phase_mux_f);
	litex_clk_get_clkout_divider(hw, &divider, &fract_cnt);

	post_glob_div_f = (u32)litex_clk_get_real_global_frequency(hw);
	period_buff = PICOS_IN_SEC;
	do_div(period_buff, post_glob_div_f);
	/* ps unit */
	global_period = (u32)period_buff;
	clkout_period = global_period * divider;

	pm = (phase_mux * global_period * 1000) / PHASE_MUX_RES_FACTOR;
	pm = litex_round(pm, 1000);

	period = delay_time * global_period + pm;

	period = period * 1000 / clkout_period;
	period = period * 360;

	return litex_round(period, 1000);
}

/* Sets phase given in degrees on given clock output */
int litex_clk_set_phase(struct clk_hw *hw, int degrees)
{
	struct litex_clk_clkout *lcko = clk_hw_to_litex_clk_clkout(hw);
	int ret;
	u16 bitset1, bitset2, reg2_mask;
	u8 *phase_mux = &lcko->phase.phase_mux,
	   *delay_time = &lcko->phase.delay_time,
	   *frac = &lcko->frac.frac,
	   *frac_en = &lcko->frac.frac_en,
	   *frac_wf_f = &lcko->frac.frac_wf_f,
	   *frac_wf_r = &lcko->frac.frac_wf_r,
	   *phase_mux_f = &lcko->frac.phase_mux_f,
	   clkout_nr = lcko->id;

	lcko->ts_config.phase = degrees;
	reg2_mask = REG2_PHASE_MASK;
	pr_debug("set_phase: %u deg", degrees);

	if (*frac > 0 && degrees != 0) {
		pr_err("CLKOUT%d: cannot set phase on that frequency",
			clkout_nr);
		return -ENOSYS;
	}
	ret = litex_clk_calc_phase(hw);
	if (ret != 0) {
		pr_err("CLKOUT%d: phase offset %d deg is too high for given frequency",
			clkout_nr, degrees);
		return ret;
	}

	bitset1 = (*phase_mux << PHASE_MUX_POS);
	bitset2 = (*delay_time << DELAY_TIME_POS);

	if (clkout_nr == 0 && frac > 0) {
		u16 clkout5_reg2_bitset;

		litex_clk_calc_duty_fract(hw, true);
		bitset2 |= (*frac << FRAC_POS)		|
			   (*frac_en << FRAC_EN_POS)	|
			   (*frac_wf_r << FRAC_WF_R_POS);

		clkout5_reg2_bitset = (*phase_mux_f << PHASE_MUX_F_POS)	|
				      (*frac_wf_f << FRAC_WF_F_POS);

		ret = litex_clk_change_value(hw, CLKOUT5_FRAC_MASK,
					   clkout5_reg2_bitset,
					   CLKOUT5_REG2);
		if (ret != 0)
			return ret;
		reg2_mask = REG2_PHASE_F_MASK;
	}

	pr_debug("SET PHASE: pm:%u pmf:%u dt:%u\nbitset1: 0x%x bitset2: 0x%x",
		*phase_mux, *phase_mux_f, *delay_time, bitset1, bitset2);

	return litex_clk_set_clock(hw, clkout_nr, REG1_PHASE_MASK, bitset1,
						  reg2_mask, bitset2);
}

/*
 *	Frequency
 */

/* Returns rate in Hz */
static inline u32 litex_clk_calc_rate(struct clk_hw *hw)
{
	struct litex_clk_clkout *lcko = clk_hw_to_litex_clk_clkout(hw);
	u64 f = litex_clk_calc_global_frequency(ldev->ts_g_config.mul,
						ldev->ts_g_config.div);

	do_div(f, lcko->config.div);

	return (u32)f;
}

/* Returns true when possible to set frequency with given global settings */
static int litex_clk_calc_clkout_params(struct litex_clk_clkout *lcko,
					   u64 vco_freq)
{
	int valid = false;
	u32 d;
	u64 m, clk_freq = 0;
	u32 margin = 0;

	if (lcko->margin.exp) {
		u32 e;

		for (e = 1, margin = 10; e < lcko->margin.exp; e++)
			margin *= margin;
	}

	lcko->div.no_cnt = 0;

	for (d = lcko->clkout_div.min; d <= lcko->clkout_div.max; d++) {
		clk_freq = vco_freq;
		do_div(clk_freq, d);
		m = lcko->ts_config.freq * lcko->margin.m;
		/* Scale margin according to its exponent */
		if (lcko->margin.exp)
			do_div(m, margin);
		if (abs(clk_freq - (u64)lcko->ts_config.freq) <= m) {
			lcko->config.freq = (u32)clk_freq;
			if (lcko->config.div != d)
				ldev->update_clkout[lcko->id] = 1;
			lcko->config.div = d;
			/* not necessary, added for sake of completeness */
			lcko->ts_config.div = d;
			lcko->frac.frac_en = 0;
			lcko->frac.frac = 0;
			if (d == 1)
				lcko->div.no_cnt = 1;
			valid = true;
			pr_debug("CLKOUT%d: freq:%u div:%u gdiv:%u gmul:%u",
				 lcko->id, lcko->config.freq, lcko->config.div,
				 ldev->ts_g_config.div, ldev->ts_g_config.mul);
			break;
		}
	}

	return valid;
}

/* Compute dividers for all active clock outputs */
static int litex_clk_calc_all_clkout_params(u64 vco_freq)
{
	struct litex_clk_clkout *lcko;
	int valid = true;
	u32 c;

	for (c = 0; c < ldev->nclkout; c++) {
		lcko = &ldev->clkouts[c];
		valid = litex_clk_calc_clkout_params(lcko, vco_freq);
		if (!valid)
			break;
	}
	return valid;
}

/* Calculate parameters for whole active part of MMCM */
static int litex_clk_calc_all_params(void)
{
	u32 div, mul;
	u64 vco_freq = 0;

	for (div = ldev->divclk.min; div <= ldev->divclk.max; div++) {
		ldev->ts_g_config.div = div;
		for (mul = ldev->clkfbout.max; mul >= ldev->clkfbout.min; mul--) {
			int bellow, above, all_valid = true;

			vco_freq = (u64)ldev->sys_clk_freq * (u64)mul;
			do_div(vco_freq, div);
			bellow = vco_freq < (ldev->vco.min
					     * (1 + ldev->vco_margin));
			above = vco_freq > (ldev->vco.max
					    * (1 - ldev->vco_margin));

			if (!bellow && !above) {
				all_valid = litex_clk_calc_all_clkout_params
								     (vco_freq);
			} else {
				all_valid = false;
			}
			if (all_valid) {
				ldev->ts_g_config.mul = mul;
				ldev->ts_g_config.freq = vco_freq;
				pr_debug("GLOBAL: freq:%llu g_div:%u g_mul:%u",
					ldev->ts_g_config.freq,
					ldev->ts_g_config.div,
					ldev->ts_g_config.mul);
				return 0;
			}
		}
	}
	pr_err("Cannot find correct settings for all clock outputs!");
	return -EINVAL;
}

/* Returns rate of given CLKOUT, parent_rate ignored */
unsigned long litex_clk_recalc_rate(struct clk_hw *hw,
				    unsigned long parent_rate)
{
	LITEX_UNUSED(parent_rate);
	return litex_clk_calc_rate(hw);
}

int litex_clk_check_rate_range(struct clk_hw *hw, unsigned long rate)
{
	struct litex_clk_clkout *lcko = clk_hw_to_litex_clk_clkout(hw);
	u64 max, min, m;
	u32 div, margin;

	m = rate * lcko->margin.m;
	if (lcko->margin.exp) {
		u32 e;

		for (e = 1, margin = 10; e < lcko->margin.exp; e++)
			margin *= margin;
		/* Scale margin according to its exponent */
		do_div(m, margin);
	}

	max = (u64)ldev->sys_clk_freq * (u64)ldev->clkfbout.max;
	div = ldev->divclk.min * lcko->clkout_div.min;
	do_div(max, div);
	max += m;

	min = ldev->sys_clk_freq * ldev->clkfbout.min;
	div = ldev->divclk.max * lcko->clkout_div.max;
	do_div(min, div);

	if (min < m)
		min = 0;
	else
		min -= m;

	if ((u64)rate < min || (u64)rate > max)
		return -EINVAL;
	return 0;
}

/* Returns closest available clock rate in Hz, parent_rate ignored */
long litex_clk_round_rate(struct clk_hw *hw, unsigned long rate,
					     unsigned long *parent_rate)
{
	struct litex_clk_clkout *lcko = clk_hw_to_litex_clk_clkout(hw);
	int ret;

	LITEX_UNUSED(parent_rate);

	ret = litex_clk_check_rate_range(hw, rate);
	if (ret != 0)
		return -EINVAL;

	lcko->ts_config.freq = rate;

	ret = litex_clk_calc_all_params();
	if (ret != 0)
		return ret;

	return litex_clk_calc_rate(hw);
}

int litex_clk_write_rate(struct clk_hw *hw)
{
	struct litex_clk_clkout *lcko = clk_hw_to_litex_clk_clkout(hw);
	int ret;
	u16 bitset1, bitset2;
	u8 *divider = &lcko->config.div,
	   *edge = &lcko->div.edge,
	   *high_time = &lcko->div.high_time,
	   *low_time = &lcko->div.low_time,
	   *no_cnt = &lcko->div.no_cnt,
	   *frac = &lcko->frac.frac,
	   *frac_en = &lcko->frac.frac_en,
	   *frac_wf_f = &lcko->frac.frac_wf_f,
	   *frac_wf_r = &lcko->frac.frac_wf_r;

	bitset1 = (*high_time << HIGH_TIME_POS)	|
		  (*low_time << LOW_TIME_POS);

	bitset2 = (*frac << FRAC_POS)		|
		  (*frac_en << FRAC_EN_POS)	|
		  (*frac_wf_r << FRAC_WF_R_POS)	|
		  (*edge << EDGE_POS)		|
		  (*no_cnt << NO_CNT_POS);

	if (*frac_en != 0) {
		u8 *phase_mux = &lcko->phase.phase_mux,
		   *delay_time = &lcko->phase.delay_time,
		   *phase_mux_f = &lcko->frac.phase_mux_f;

		lcko->config.phase = 0;
		litex_clk_calc_phase_fract(hw);

		bitset1 |= (*phase_mux << PHASE_MUX_POS);
		bitset2 |= (*delay_time << DELAY_TIME_POS);

		if (frac_wf_f > 0 || phase_mux_f > 0) {
			u16 clkout5_reg2_bitset = 0;

			clkout5_reg2_bitset = (*phase_mux_f << PHASE_MUX_F_POS) |
					      (*frac_wf_f << FRAC_WF_F_POS);

			ret = litex_clk_change_value(hw, CLKOUT5_FRAC_MASK,
				    clkout5_reg2_bitset, CLKOUT5_REG2);
			if (ret != 0)
				return ret;
		}
	}

	pr_debug("SET RATE: div:%u f:%u fwfr:%u fwff:%u fen:%u nc:%u e:%u ht:%u lt:%u\nbitset1: 0x%x bitset2: 0x%x",
		*divider, *frac, *frac_wf_r, *frac_wf_f, *frac_en,
		*no_cnt, *edge, *high_time, *low_time, bitset1, bitset2);

	ret = litex_clk_set_clock(hw, lcko->id, REG1_FREQ_MASK, bitset1,
						REG2_FREQ_MASK, bitset2);
	if (ret != 0)
		return ret;

	ldev->update_clkout[lcko->id] = 0;

	return 0;
}

int litex_clk_update_clkouts(void)
{
	struct litex_clk_clkout *lcko;
	int ret;
	u8 c;

	for (c = 0; c < ldev->nclkout; c++) {
		if (ldev->update_clkout[c]) {
			lcko = &ldev->clkouts[c];
			ret = litex_clk_calc_duty_normal(&lcko->clk_hw, false);
			if (ret != 0)
				return ret;
			ret = litex_clk_write_rate(&lcko->clk_hw);
			if (ret != 0)
				return ret;
		}
	}

	return 0;
}

/* Set closest available clock rate in Hz, parent_rate ignored */
int litex_clk_set_rate(struct clk_hw *hw, unsigned long rate,
					  unsigned long parent_rate)
{
	int ret;

	LITEX_UNUSED(parent_rate);
	pr_debug("set_rate: %lu", rate);

	ret = litex_clk_set_globs();
	if (ret != 0)
		return ret;
	ret = litex_clk_calc_duty_normal(hw, false);
	if (ret != 0)
		return ret;
	ret = litex_clk_write_rate(hw);
	if (ret != 0)
		return ret;
	ret = litex_clk_update_clkouts();
	if (ret != 0)
		return ret;

#ifdef DEBUG
	litex_clk_print_all_params();
	litex_clk_print_all_regs(&ldev->clk_hw);
#endif /* #ifdef DEBUG */

	return 0;
}

/* Enable Clock Control MMCM module */
int litex_clk_enable(struct clk_hw *hw)
{
	return litex_clk_change_value(hw, FULL_REG_16, FULL_REG_16, POWER_REG);
}

/* Disable Clock Control MMCM module */
void litex_clk_disable(struct clk_hw *hw)
{
	litex_clk_change_value(hw, ZERO_REG, ZERO_REG, POWER_REG);
}

/* Set default clock value from device tree for given clkout*/
static int litex_clk_set_def_clkout(struct clk_hw *hw)
{
	struct litex_clk_clkout *lcko = clk_hw_to_litex_clk_clkout(hw);
	int ret;

	litex_clk_round_rate(hw, lcko->def.freq, 0);
	ret = litex_clk_set_rate(hw, lcko->def.freq, 0);
	if (ret != 0)
		return ret;
	ret = litex_clk_set_duty_cycle(hw, &lcko->def.duty);
	if (ret != 0)
		return ret;
	return litex_clk_set_phase(hw, lcko->def.phase);
}

static int litex_clk_set_all_def_clkouts(void)
{
	struct clk_hw *hw;
	int c, ret;

	for (c = 0; c < ldev->nclkout; c++) {
		hw = &ldev->clkouts[c].clk_hw;

		ret = litex_clk_set_def_clkout(hw);
		if (ret != 0)
			return ret;
	}
	return 0;
}

static const struct clk_ops litex_clk_ops = {
	.enable			=	litex_clk_enable,
	.disable		=	litex_clk_disable,
	.recalc_rate		=	litex_clk_recalc_rate,
	.round_rate		=	litex_clk_round_rate,
	.set_rate		=	litex_clk_set_rate,
	.get_phase		=	litex_clk_get_phase,
	.set_phase		=	litex_clk_set_phase,
	.get_duty_cycle		=	litex_clk_get_duty_cycle,
	.set_duty_cycle		=	litex_clk_set_duty_cycle,
};

static const struct of_device_id litex_of_match[] = {
	{.compatible = "litex,clk"},
	{},
};

MODULE_DEVICE_TABLE(of, litex_of_match);

static int litex_clk_dts_clkout_ranges_read(struct device_node *node,
					    struct litex_clk_range *clkout_div)
{
	int ret;

	ret = of_property_read_u32(node, "litex,clkout-divide-min",
					       &clkout_div->min);
	if (ret != 0)
		return -ret;
	ret = of_property_read_u32(node, "litex,clkout-divide-max",
					       &clkout_div->max);
	if (ret != 0)
		return -ret;

	return 0;
}

static int litex_clk_dts_clkout_def_read(struct device_node *child,
					 struct litex_clk_clkout *lcko)
{
	int ret;

	ret = of_property_read_u32(child, "reg", &lcko->id);
	if (ret != 0)
		return -ret;

	if (lcko->id > ldev->nclkout) {
		pr_err("%s: Invalid CLKOUT index!", child->name);
		return -EINVAL;
	}

	ret = of_property_read_u32(child, "litex,clock-margin",
						 &lcko->margin.m);
	if (ret != 0)
		return -ret;
	ret = of_property_read_u32(child, "litex,clock-margin-exp",
						 &lcko->margin.exp);
	if (ret != 0)
		return -ret;
	ret = of_property_read_u32(child, "litex,clock-frequency",
						 &lcko->def.freq);
	if (ret != 0)
		return -ret;
	ret = of_property_read_u32(child, "litex,clock-phase",
						 &lcko->def.phase);
	if (ret != 0)
		return -ret;
	ret = of_property_read_u32(child, "litex,clock-duty-den",
						 &lcko->def.duty.den);
	if (ret != 0)
		return -ret;
	ret = of_property_read_u32(child, "litex,clock-duty-num",
						 &lcko->def.duty.num);
	if (ret != 0)
		return -ret;
	if (lcko->def.duty.den <= 0 ||
	    lcko->def.duty.num > lcko->def.duty.den) {
		pr_err("%s: Invalid default duty! %d/%d",
			child->name, lcko->def.duty.num, lcko->def.duty.den);
		return -EINVAL;
	}

	return 0;
}

static int litex_clk_dts_timeout_read(struct device_node *node,
				      struct litex_clk_timeout *timeout)
{
	int ret;

	/* Read wait_lock timeout from device property*/
	ret = of_property_read_u32(node, "litex,lock-timeout", &timeout->lock);
	if (ret < 0) {
		pr_err("No litex,lock_timeout entry in the dts file\n");
		return -ENODEV;
	}
	if (timeout->lock < 1) {
		pr_err("LiteX CLK driver cannot wait for time bellow ca. 1ms\n");
		return -EINVAL;
	}

	/* Read wait_drdy timeout from device property*/
	ret = of_property_read_u32(node, "litex,drdy-timeout", &timeout->drdy);
	if (ret < 0) {
		pr_err("No litex,lock_drdy entry in the dts file\n");
		return -ENODEV;
	}
	if (timeout->drdy < 1) {
		pr_err("LiteX CLK driver cannot wait for time bellow ca. 1ms\n");
		return -EINVAL;
	}

	return 0;
}

static int litex_clk_dts_clkouts_read(struct device_node *node)
{
	struct device_node *child_node;
	struct litex_clk_range clkout_div;
	struct litex_clk_clkout *lcko;
	int ret, i = 0;

	ret = litex_clk_dts_clkout_ranges_read(node, &clkout_div);
	if (ret != 0)
		return ret;

	for_each_child_of_node(node, child_node) {
		lcko = &ldev->clkouts[i];
		lcko->clkout_div = clkout_div;

		ret = litex_clk_dts_clkout_def_read(child_node, lcko);
		if (ret != 0)
			return ret;
		i++;
	}
	return 0;
}

static int litex_clk_init_clkouts(struct device *dev, struct device_node *node)
{
	struct device_node *child_node;
	struct litex_clk_clkout *lcko;
	int ret, i = 0;

	for_each_child_of_node(node, child_node) {
		struct clk_init_data clkout_init;

		lcko = &ldev->clkouts[i];
		clkout_init.name = child_node->name;
		clkout_init.ops = &litex_clk_ops;
		clkout_init.num_parents = 1;
		clkout_init.parent_names = &node->name;
		clkout_init.flags = CLK_SET_RATE_UNGATE | CLK_GET_RATE_NOCACHE;
		lcko->clk_hw.init = &clkout_init;
		lcko->base = ldev->base;
		/* mark defaults to set */
		lcko->ts_config.freq = lcko->def.freq;
		lcko->ts_config.duty = lcko->def.duty;
		lcko->ts_config.phase = lcko->def.phase;

		ret = devm_clk_hw_register(dev, &lcko->clk_hw);
		if (ret != 0) {
			pr_err("devm_clk_hw_register failure, %s ret: %d\n",
						     child_node->name, ret);
			return ret;
		}

		ret = of_clk_add_hw_provider(child_node, of_clk_hw_simple_get,
							       &lcko->clk_hw);
		if (ret != 0)
			return ret;
		i++;
	}
	return 0;
}

static int litex_clk_dts_cnt_clocks(struct device_node *node)
{
	struct device_node *child_node;
	u32 i = 0;

	/* count existing CLKOUTs */
	for_each_child_of_node(node, child_node)
		i++;
	return i;
}

static int litex_clk_dts_global_ranges_read(struct device_node *node)
{
	int ret;

	ret = of_property_read_u32(node, "litex,divclk-divide-min",
						&ldev->divclk.min);
	if (ret != 0)
		return -ret;
	ret = of_property_read_u32(node, "litex,divclk-divide-max",
						&ldev->divclk.max);
	if (ret != 0)
		return -ret;
	ret = of_property_read_u32(node, "litex,clkfbout-mult-min",
					      &ldev->clkfbout.min);
	if (ret != 0)
		return -ret;
	ret = of_property_read_u32(node, "litex,clkfbout-mult-max",
					      &ldev->clkfbout.max);
	if (ret != 0)
		return -ret;
	ret = of_property_read_u32(node, "litex,vco-freq-min",
					      &ldev->vco.min);
	if (ret != 0)
		return -ret;
	ret = of_property_read_u32(node, "litex,vco-freq-max",
					      &ldev->vco.max);
	if (ret != 0)
		return -ret;
	return of_property_read_u32(node, "litex,vco-margin",
					  &ldev->vco_margin);
}
static int litex_clk_dts_global_read(struct device *dev,
				     struct device_node *node)
{
	int ret;

	ret = of_property_read_u32(node, "litex,sys-clock-frequency",
						&ldev->sys_clk_freq);
	if (ret < 0) {
		pr_err("No clock-frequency entry in the dts file\n");
		return -ENODEV;
	}

	ldev->nclkout = litex_clk_dts_cnt_clocks(node);
	if (ret != 0)
		return ret;

	clkouts = devm_kzalloc(dev,
			       sizeof(struct litex_clk_clkout) * ldev->nclkout,
			       GFP_KERNEL);
	ldev->update_clkout = devm_kzalloc(dev, sizeof(u8) * ldev->nclkout,
							       GFP_KERNEL);
	if (!clkouts || !ldev->update_clkout) {
		pr_err("CLKOUT memory allocation failure!");
		return -ENOMEM;
	}
	ldev->clkouts = clkouts;

	ret = litex_clk_dts_timeout_read(node, &ldev->timeout);
	if (ret != 0)
		return ret;

	return litex_clk_dts_global_ranges_read(node);
}

static int litex_clk_init_glob_clk(struct device *dev, struct device_node *node)
{
	struct clk_init_data init;
	struct clk_hw *hw;
	int ret;

	init.name = node->name;
	init.num_parents = 0;
	init.ops = &litex_clk_ops;
	init.flags = CLK_SET_RATE_UNGATE;

	hw = &ldev->clk_hw;
	hw->init = &init;

	ret = devm_clk_hw_register(dev, hw);
	if (ret != 0) {
		pr_err("devm_clk_hw_register failure, ret: %d\n", ret);
		return ret;
	}

	ret = of_clk_add_hw_provider(node, of_clk_hw_simple_get, hw);
	if (ret != 0) {
		pr_err("of_clk_add_hw_provider failure, ret: %d", ret);
		return ret;
	}

	/* Power on MMCM module */
	ret = litex_clk_change_value(hw, FULL_REG_16, FULL_REG_16, POWER_REG);
	if (ret != 0) {
		pr_err("MMCM initialization failure, ret: %d", ret);
		return ret;
	}

	return 0;
}

static int litex_clk_probe(struct platform_device *pdev)
{
	struct device *dev;
	struct device_node *node;
	struct resource *res;
	int ret;

	if (!litex_check_accessors())
		return -EPROBE_DEFER;

	dev = &pdev->dev;
	node = dev->of_node;
	if (!node)
		return -ENODEV;

	ldev = devm_kzalloc(dev, sizeof(struct litex_clk_device), GFP_KERNEL);
	if (IS_ERR_OR_NULL(ldev))
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(res))
		return -EBUSY;

	ldev->base = devm_of_iomap(dev, node, 0, &res->end);
	if (IS_ERR_OR_NULL(ldev->base))
		return -EIO;

	ret = litex_clk_dts_global_read(dev, node);
	if (ret != 0)
		return ret;

	ret = litex_clk_dts_clkouts_read(node);
	if (ret != 0)
		return ret;

	ret = litex_clk_init_clkouts(dev, node);
	if (ret != 0)
		return ret;

	ret = litex_clk_init_glob_clk(dev, node);
	if (ret != 0)
		return ret;

	ret = litex_clk_set_all_def_clkouts();
	if (ret != 0)
		return ret;

#ifdef DEBUG
	litex_clk_print_all_params();
	litex_clk_print_all_regs(&ldev->clk_hw);
#endif /* #ifdef DEBUG */

	pr_info("litex clk control driver initialized");
	return 0;
}

static int litex_clk_remove(struct platform_device *pdev)
{
	of_clk_del_provider(pdev->dev.of_node);
	return 0;
}

static struct platform_driver litex_clk_driver = {
	.driver = {
		.name = "litex-clk",
		.of_match_table = of_match_ptr(litex_of_match),
	},
	.probe = litex_clk_probe,
	.remove = litex_clk_remove,
};

module_platform_driver(litex_clk_driver);
MODULE_DESCRIPTION("LiteX clock driver");
MODULE_AUTHOR("Antmicro <www.antmicro.com>");
MODULE_LICENSE("GPL v2");
