// SPDX-License-Identifier: GPL-2.0
/*
 * LiteSPI controller (LiteX) Driver
 *
 * Copyright (C) 2019 Antmicro Ltd. <www.antmicro.com>
 */

#include <linux/device.h>
#include <linux/litex.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#define DRIVER_NAME "litespi"

#define LITESPI_OFF_CTRL	0x00
#define LITESPI_OFF_STAT	0x08
#define LITESPI_OFF_MOSI	0x0c
#define LITESPI_OFF_MISO	0x10
#define LITESPI_OFF_CS		0x14

#define LITESPI_SZ_CTRL		2
#define LITESPI_SZ_STAT		1
#define LITESPI_SZ_CS		1

#define LITESPI_CTRL_SHIFT_BPW	8
#define LITESPI_CTRL_START_BIT	0

struct litespi_hw {
	struct spi_master *master;
	void __iomem *base_addr;
	struct mutex bus_mutex;
};

static inline void litespi_wait_xfer_end(struct litespi_hw *hw)
{
	while (!litex_get_reg(hw->base_addr + LITESPI_OFF_STAT,
			      LITESPI_SZ_STAT))
		cpu_relax();
}

static void litespi_rxtx(struct litespi_hw *hw, struct spi_transfer *t)
{
	u32 val;
	u8 bytes;
	int i;

	/*
	 * Calculate number of bytes necessary for given bits_per_word value.
	 * Incrementing by 7 forces non-multiplies of 8 to be incremented by 1
	 * after bit shifting, without changing the result for multiplies of 8.
	 */
	bytes = (t->bits_per_word + 7) >> 3;

	for (i = 0; i < t->len; i += bytes) {
		if (t->tx_buf) {
			memcpy(&val, t->tx_buf, bytes);
			litex_set_reg(hw->base_addr + LITESPI_OFF_MOSI, bytes,
				      val);
			t->tx_buf += bytes;
		}

		val = litex_get_reg(hw->base_addr + LITESPI_OFF_CTRL,
				    LITESPI_SZ_CTRL);
		litex_set_reg(hw->base_addr + LITESPI_OFF_CTRL, LITESPI_SZ_CTRL,
			      val | BIT(LITESPI_CTRL_START_BIT));
		litespi_wait_xfer_end(hw);

		if (t->rx_buf) {
			val = litex_get_reg(hw->base_addr + LITESPI_OFF_MISO,
					    bytes);
			memcpy(t->rx_buf, &val, bytes);
			t->rx_buf += bytes;
		}
	}
}

static int litespi_xfer_one(struct spi_master *master, struct spi_message *m)
{
	struct litespi_hw *hw = spi_master_get_devdata(master);
	struct spi_transfer *t;

	mutex_lock(&hw->bus_mutex);

	/* setup chip select */
	litex_set_reg(hw->base_addr + LITESPI_OFF_CS, LITESPI_SZ_CS,
		      BIT(m->spi->chip_select));

	list_for_each_entry(t, &m->transfers, transfer_list) {
		litespi_rxtx(hw, t);
		m->actual_length += t->len;
	}

	m->status = 0;
	spi_finalize_current_message(master);

	mutex_unlock(&hw->bus_mutex);

	return 0;
}

static int litespi_setup(struct spi_device *spi)
{
	struct litespi_hw *hw = spi_master_get_devdata(spi->master);

	/* lock and wait for transfer finish */
	mutex_lock(&hw->bus_mutex);
	litespi_wait_xfer_end(hw);

	/* set word size and clear CS bits */
	litex_set_reg(hw->base_addr + LITESPI_OFF_CTRL, LITESPI_SZ_CTRL,
		      spi->bits_per_word << LITESPI_CTRL_SHIFT_BPW);

	mutex_unlock(&hw->bus_mutex);

	return 0;
}

static int litespi_probe(struct platform_device *pdev)
{
	struct litespi_hw *hw;
	struct spi_master *master;
	struct device_node *np;
	struct resource *res;
	int ret;
	u32 val;

	if (!litex_check_accessors())
		return -EPROBE_DEFER;

	/* check if device tree exists */
	np = pdev->dev.of_node;
	if (!np)
		return -ENODEV;

	/* allocate memory for master */
	master = spi_alloc_master(&pdev->dev, sizeof(*hw));
	if (!master)
		return -ENOMEM;

	hw = spi_master_get_devdata(master);
	hw->master = master;
	mutex_init(&hw->bus_mutex);

	hw->master->dev.of_node = pdev->dev.of_node;
	hw->master->setup = litespi_setup;
	hw->master->transfer_one_message = litespi_xfer_one;
	hw->master->mode_bits =	SPI_MODE_0;
	hw->master->flags = SPI_CONTROLLER_MUST_RX | SPI_CONTROLLER_MUST_TX;

	/* get bits per word property */
	ret = of_property_read_u32(np, "litespi,max-bpw", &val);
	if (ret || val > 32)
		return -EINVAL;

	hw->master->bits_per_word_mask = SPI_BPW_RANGE_MASK(1, val);

	/* get sck frequency */
	ret = of_property_read_u32(np, "litespi,sck-frequency", &val);
	if (ret)
		return -EINVAL;

	hw->master->min_speed_hz = val;
	hw->master->max_speed_hz = val;

	/* get num cs */
	ret = of_property_read_u32(np, "litespi,num-cs", &val);
	if (ret)
		return -EINVAL;

	hw->master->num_chipselect = val;

	/* get base address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hw->base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (!hw->base_addr)
		return -ENXIO;

	/* register controller */
	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret < 0) {
		spi_master_put(master);
		return ret;
	}

	return 0;
}

static const struct of_device_id litespi_match[] = {
	{ .compatible = "litex,litespi" },
	{}
};
MODULE_DEVICE_TABLE(of, litespi_match);

static struct platform_driver litespi_driver = {
	.probe = litespi_probe,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(litespi_match)
	}
};
module_platform_driver(litespi_driver)

MODULE_AUTHOR("Antmicro Ltd <www.antmicro.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
