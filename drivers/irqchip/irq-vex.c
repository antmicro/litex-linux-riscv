// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Antmicro
 */
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <utils/puts.h>

static void vex_enable_irq(int num, int enable)
{
    u32 mask = csr_read(0xBC0);
    //DBGMSG("vexirq %s irq %d", enable? "enable" : "disable",  num);
    if(enable) {
        csr_write(0xBC0, mask | (1 << num));
    } else {
        csr_write(0xBC0, mask & ~(1 << num));
    }
    //DBGMSG("done!");

}

static int vex_get_irq(void)
{
    int i;
    u32 pend = csr_read(0xFC0);
    //DBGMSG("vexirq get irq");
    for(i = 31; i >= 0; i--) {
        if(pend & (1 << i)) {
            //DBGMSG("vexirq get irq found %d", i);
            return i;
        }
    }
    return -1;
}

static void vex_irq_enable(struct irq_data *d)
{
	vex_enable_irq(d->hwirq, 1);
}

static void vex_irq_disable(struct irq_data *d)
{
	vex_enable_irq(d->hwirq, 0);
}

static struct irq_chip vex_chip = {
	.name		= "VexRiscv IRQ",
	/*
	 * There is no need to mask/unmask PLIC interrupts.  They are "masked"
	 * by reading claim and "unmasked" when writing it back.
	 */
	.irq_enable	= vex_irq_enable,
	.irq_disable	= vex_irq_disable,
};
static int vex_irqdomain_map(struct irq_domain *d, unsigned int irq,
			      irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &vex_chip, handle_simple_irq);
	irq_set_chip_data(irq, NULL);
	irq_set_noprobe(irq);
	return 0;
}

static const struct irq_domain_ops vex_irqdomain_ops = {
	.map		= vex_irqdomain_map,
	.xlate		= irq_domain_xlate_onecell,
};

static struct irq_domain *vex_irqdomain;
static void vex_handle_irq(struct pt_regs *regs)
{
	irq_hw_number_t hwirq;
    //DBGMSG("vexirq handle irq");

	while ((hwirq = vex_get_irq())!= -1) {
		int irq = irq_find_mapping(vex_irqdomain, hwirq);
        //DBGMSG("vexirq handle %d", hwirq);
        vex_enable_irq(hwirq, 0);

		if (unlikely(irq <= 0))
			pr_warn_ratelimited("can't find mapping for hwirq %lu\n",
					hwirq);
		else
			generic_handle_irq(irq);
        vex_enable_irq(hwirq, 1);
	}
}
static int __init vex_init(struct device_node *node,
		struct device_node *parent)
{
    int nr_irqs = 10;
    DBGMSG("vexirq init");
	vex_irqdomain = irq_domain_add_linear(node, nr_irqs + 1,
			&vex_irqdomain_ops, NULL);
	if (WARN_ON(!vex_irqdomain))
		return -22;

    DBGMSG("vexirq init handle set");
	set_handle_irq(vex_handle_irq);
	return 0;

}

IRQCHIP_DECLARE(vex, "riscv,vex", vex_init);
