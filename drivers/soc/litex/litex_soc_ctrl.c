// SPDX-License-Identifier: GPL-2.0
/*
 * LiteX SoC Controller Driver
 *
 * Copyright (C) 2020 Antmicro
 *
 */

#include <linux/litex.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/io.h>

#define SCRATCH_REG_OFF		0x04
#define SCRATCH_REG_VALUE	0x12345678

/*
 * Pointers for accessor functions
 *
 * note: or1k and vexriscv kernels use different CONFIG_GENERIC_IOMAP
 *	setting and it changes the definition of io{read,write}32
 *	functions so we are checking here to use correct definition
 */
#ifndef CONFIG_GENERIC_IOMAP
unsigned int (*litex_read_reg)(const volatile void __iomem *addr) = NULL;
void (*litex_write_reg)(u32 val, volatile void __iomem *addr) = NULL;
#else
unsigned int (*litex_read_reg)(void __iomem *addr) = NULL;
void (*litex_write_reg)(u32 val, void __iomem *addr) = NULL;
#endif

/*
 * check if accessors are ready to use
 * returns true if yes - false if not
 */
int litex_check_accessors(void)
{
	if (litex_read_reg == NULL || litex_write_reg == NULL)
		return 0;
	return 1;
}

struct litex_soc_ctrl_device {
	void __iomem *base;
};

/* Read litex CSR with Little Endian function */
static inline u32 litex_soc_ctrl_get_reg(void __iomem *reg_addr)
{
	return  (ioread32(reg_addr) << 24)	 |
		(ioread32(reg_addr + 0x4) << 16) |
		(ioread32(reg_addr + 0x8) << 8)  |
		 ioread32(reg_addr + 0xc);
}

/* Check byte order and set correct accessors */
static void litex_soc_ctrl_check_endianness(void __iomem *reg_addr)
{
	u32 reg;

	reg = litex_soc_ctrl_get_reg(reg_addr + SCRATCH_REG_OFF);

	if (reg == SCRATCH_REG_VALUE) {
		pr_info("Detected endianness: Little Endian");
		litex_read_reg = ioread32;
		litex_write_reg = iowrite32;
	} else {
		pr_info("Detected endianness: Big Endian");
		litex_read_reg = ioread32be;
		litex_write_reg = iowrite32be;
	}
}

static const struct of_device_id litex_soc_ctrl_of_match[] = {
	{.compatible = "litex,soc_controller"},
	{},
};

MODULE_DEVICE_TABLE(of, litex_soc_ctrl_of_match);

static int litex_soc_ctrl_probe(struct platform_device *pdev)
{
	struct device *dev;
	struct device_node *node;
	const struct of_device_id *id;
	struct litex_soc_ctrl_device *soc_ctrl_dev;
	struct resource *res;

	dev = &pdev->dev;
	node = dev->of_node;
	if (!node)
		return -ENODEV;

	id = of_match_node(litex_soc_ctrl_of_match, node);
	if (!id)
		return -ENODEV;

	soc_ctrl_dev = devm_kzalloc(dev, sizeof(*soc_ctrl_dev), GFP_KERNEL);
	if (IS_ERR_OR_NULL(soc_ctrl_dev))
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(res))
		return -EBUSY;

	soc_ctrl_dev->base = devm_of_iomap(dev, node, 0, &res->end);
	if (IS_ERR_OR_NULL(soc_ctrl_dev->base))
		return -EIO;

	litex_soc_ctrl_check_endianness(soc_ctrl_dev->base);

	return 0;
}

static int litex_soc_ctrl_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver litex_soc_ctrl_driver = {
	.driver = {
		.name = "litex-soc-controller",
		.of_match_table = of_match_ptr(litex_soc_ctrl_of_match)
	},
	.probe = litex_soc_ctrl_probe,
	.remove = litex_soc_ctrl_remove
};

module_platform_driver(litex_soc_ctrl_driver);
MODULE_DESCRIPTION("LiteX SoC Controller driver");
MODULE_AUTHOR("Antmicro <www.antmicro.com>");
MODULE_LICENSE("GPL v2");
