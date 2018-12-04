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

#include <linux/init.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/initrd.h>
#include <linux/memblock.h>
#include <linux/swap.h>
#include <linux/sizes.h>

#include <asm/tlbflush.h>
#include <asm/sections.h>
#include <asm/pgtable.h>
#include <asm/io.h>

#include <utils/puts.h>

static void __init zone_sizes_init(void)
{
	unsigned long max_zone_pfns[MAX_NR_ZONES] = { 0, };

#ifdef CONFIG_ZONE_DMA32
	max_zone_pfns[ZONE_DMA32] = PFN_DOWN(min(4UL * SZ_1G, max_low_pfn));
#endif
	max_zone_pfns[ZONE_NORMAL] = max_low_pfn;

	free_area_init_nodes(max_zone_pfns);
}

void setup_zero_page(void)
{
        DBGMSG("Going to fill 0x%08X .. 0x%08X (size=%X) with zeros.", (uint32_t)empty_zero_page, (uint32_t)((uint32_t)empty_zero_page + PAGE_SIZE), (uint32_t)PAGE_SIZE);
	memset((void *)empty_zero_page, 0, PAGE_SIZE);
	DBGMSG("Cleaning done.");
}

extern void dbgprintf(char const *fmt, ...);

void __init paging_init(void)
{
        DBGMSG("before zero_page");
	setup_zero_page();
	DBGMSG("before flush_tlb_all");
	local_flush_tlb_all();
	DBGMSG("before zone_sizes_init");
	zone_sizes_init();
	DBGMSG("leaving paging init");
}

void __init mem_init(void)
{
    DBG();
#ifdef CONFIG_FLATMEM
	BUG_ON(!mem_map);
#endif /* CONFIG_FLATMEM */
DBG();
	high_memory = (void *)(__va(PFN_PHYS(max_low_pfn)));
	DBG();
        DBGMSG("high_memory = 0x%08X (phys=%08X)", (uint32_t)high_memory, (uint32_t)PFN_PHYS(max_low_pfn));
	free_all_bootmem();
	DBG();

	mem_init_print_info(NULL);
	DBGMSG("leaving mem_init");
}

void free_initmem(void)
{
	free_initmem_default(0);
}

#ifdef CONFIG_BLK_DEV_INITRD
void free_initrd_mem(unsigned long start, unsigned long end)
{
}
#endif /* CONFIG_BLK_DEV_INITRD */
