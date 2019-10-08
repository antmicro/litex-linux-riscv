// SPDX-License-Identifier: GPL-2.0
 /*
 * Copyright (c) 2020 Antmicro <www.antmicro.com>
 * 
 * Author: Jakub Cebulski
 */

#include <linux/spi/spi.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/module.h>

#include <linux/litex.h>
#include <linux/io.h>

#include <linux/types.h>
#include <linux/signal.h>

#define SPIFLASH4X0_LABEL "spiflash"

#define MEMORY_SIZE 16777216L

#define TIMEOUT_SPI 2000
#define TIMEOUT_FLASH 20000

#define SUBSECTOR_SIZE 4096L

#define WIP 0x1
#define WEN_SET 0x2
#define MANUFACTURER_ID 0x20

#define SPI_CSR_BASE spi->csr_base

#define SPI_DATA_IN ((SPI_CSR_BASE) + 0x1C)
#define SPI_DATA_OUT ((SPI_CSR_BASE) + 0x3c)
#define SPI_IN_LEN ((SPI_CSR_BASE) + 0x18)
#define SPI_OUT_LEN ((SPI_CSR_BASE) + 0x14)
#define QSPI ((SPI_CSR_BASE) + 0x10)
#define COMMAND_QUEUED ((SPI_CSR_BASE) + 0x0c)

#define READ_STATUS_REGISTER 0x05
#define WRITE_ENABLE 0x06
#define READ_EVC_REG 0x65
#define WRITE_EVC_REG 0x61
#define READ_ID 0x9F
#define SUBSECTOR_ERASE 0x20

#define ID_BYTE_1 0x20
#define ID_BYTE_2 0xBA

struct litex_qspi_flash {
	struct spi_nor nor;
	struct device *dev;
	void __iomem *mem_base;
	void __iomem *csr_base;
};

static int wait_while_spi_busy(struct spi_nor *nor)
{
	long timeout = TIMEOUT_SPI;
	struct litex_qspi_flash *spi = nor->priv;

	while (litex_get_reg((u8 *)COMMAND_QUEUED, 1)) {
		if (timeout < 0)
			return -ETIMEDOUT;

		timeout--;
	}

	return 0;
}

static int spi_tranceive(struct spi_nor *nor, int write_len,
			 int read_len, u8 *read_buf,
			 u8 *write_buf)
{
	int i;
	struct litex_qspi_flash *spi = nor->priv;

	if (wait_while_spi_busy(nor))
		return -EBUSY;

	for(i = 0; i < write_len; i++)
		litex_set_reg(SPI_DATA_IN + i*4, 1, write_buf[i]);

	litex_set_reg(SPI_IN_LEN, 1, read_len);
	litex_set_reg(SPI_OUT_LEN, 1, write_len);

	if (wait_while_spi_busy(nor))
		return -EBUSY;

	for(i = 0; i < read_len; i++)
		read_buf[i] = litex_get_reg(SPI_DATA_OUT + i*4, 1);

	return 0;
}

static int read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	u8 spi_buf[8] = {opcode};

	wait_while_spi_busy(nor);

	spi_tranceive(nor, 1, len, buf, spi_buf);

	return 0;
}

static int write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	u8 spi_buf[8] = {opcode};
	int i;

	for(i = 0; i < len; i++)
	        spi_buf[i + 1] = buf[i];
	wait_while_spi_busy(nor);

	spi_tranceive(nor, len, 0, NULL, spi_buf);

	return 0;
}

static int wait_while_flash_busy(struct spi_nor *nor)
{
	u8 command[1] = {READ_STATUS_REGISTER};
	u8 receive[1] = {0x00};
	unsigned long timeout = TIMEOUT_FLASH;

	do{
		if (!timeout){
		  return -EBUSY;
		}
		timeout--;
		spi_tranceive(nor, 1, 1, receive, command);
	}while (receive[0] & WIP);

	return 0;
}

static int write_protection(struct spi_nor *nor, bool enable)
{
	u8 spi_buf[8];
	u8 receive[1];
	int ret;

	ret = wait_while_flash_busy(nor);
	if (ret)
		return ret;

	spi_buf[0] = WRITE_ENABLE;
	spi_tranceive(nor, 1, 0, NULL, spi_buf);

	spi_buf[0] = READ_STATUS_REGISTER;

	ret = wait_while_spi_busy(nor);
	if (ret)
		return ret;

	spi_tranceive(nor, 1, 1, receive, spi_buf);

	if (!(receive[0] & WEN_SET)) {
		return 1;
	}

	return 0;

}

static ssize_t flash_write(struct spi_nor *nor, loff_t addr,
			size_t data_size,
			const u_char *data)
{
	u32 buf;
	u8 flash_offset;
	u8 miss;
	u8 min;
	int data_left = data_size;
	u8 *buf8 = (u8 *)&buf;
	u8 *data8 = (u8 *)data;
	struct litex_qspi_flash *spi = nor->priv;
	int ret;
	
	if ((addr + data_size) > MEMORY_SIZE || addr < 0)
		return -EFAULT;

	while (data_left > 0) {
		buf = 0xFFFFFFFF;
		flash_offset = addr & 0x3;
		miss = 4 - flash_offset;

		ret = wait_while_spi_busy(nor);
		if (ret)
			return ret;

		if (write_protection(nor, false))
			return data_size-data_left;

		min = (data_left < miss ? data_left : miss);
		memcpy(buf8 + flash_offset, data8, min);

		iowrite32(buf, spi->mem_base + addr - flash_offset);
		ret = wait_while_flash_busy(nor);
		if (ret)
			return ret;

		data8 += min;
		addr += min;
		data_left -= min;
	}
		ret = wait_while_flash_busy(nor);
		if (ret)
			return ret;

	return data_size - data_left;
}

static ssize_t flash_read(struct spi_nor *nor, loff_t addr,
				 size_t data_size, u_char *data)
{
	u32 buf;
	u32 data_left = data_size;
	u8 flash_offset;
	u8 miss;
	u8 min;
	u8 *buf8 = (u8 *)&buf;
	u8 *data8 = (u8 *)data;
	struct litex_qspi_flash *spi = nor->priv;

	if ((addr + data_size) > MEMORY_SIZE || addr < 0)
		return -EFAULT;

	while (data_left > 0) {
		flash_offset = addr & 0x3;
		miss = 4 - flash_offset;
		min = (data_left < miss) ? data_left : miss;

		if (wait_while_flash_busy(nor))
			return data_size - data_left;

		buf = ioread32(spi->mem_base + addr - flash_offset);

		memcpy(data8, (buf8 + flash_offset), min);
		addr += min;
		data8 += min;
		data_left -= min;
	}

	return data_size - data_left;
}

static ssize_t flash_erase(struct spi_nor *nor, loff_t addr)
{

	u8 send[4] = {0, 0, 0, 0};

	if ((addr) > MEMORY_SIZE || addr < 0)
		return -EFAULT;

	if (wait_while_flash_busy(nor))
		return -EBUSY;

	if (write_protection(nor, 0))
		return -EIO;

	if (wait_while_flash_busy(nor))
		return -EBUSY;

	send[0] = SUBSECTOR_ERASE;
	send[1] = (addr) >> 16;
	send[2] = (addr) >> 8;
	send[3] = (addr) >> 0;
	spi_tranceive(nor, 4, 0, NULL, send);

	if (wait_while_spi_busy(nor))
		return -EBUSY;

	return 0;
}

int spi_flash_litex_init(struct platform_device *pdev)
{
	struct litex_qspi_flash *spi;
	struct spi_nor *nor;
	struct resource *res;
	struct device_node *node1;
	resource_size_t size = 0;
	int ret;

	const struct spi_nor_hwcaps hwcaps = {
		.mask = SNOR_HWCAPS_READ |
			SNOR_HWCAPS_READ_FAST |
			SNOR_HWCAPS_PP,
	};

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No DT found\n");
		return -EINVAL;
	}

	spi = devm_kzalloc(&pdev->dev, sizeof(*spi), GFP_KERNEL);
	node1 = of_get_next_available_child(pdev->dev.of_node, NULL);
	if (IS_ERR_OR_NULL(node1)) {
		dev_err(&pdev->dev, "no SPI flash device to configure\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, spi);
	spi->dev = &pdev->dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	spi->csr_base = devm_of_iomap(&pdev->dev, node1, 1, &size);
	spi->mem_base = devm_of_iomap(&pdev->dev, node1, 0, &size);

	if (IS_ERR_OR_NULL(spi->mem_base))
		return -ENODEV;

	nor = &spi->nor;
	nor->dev = spi->dev;
	nor->priv = spi;
	spi_nor_set_flash_node(nor, node1);
	nor->read_reg = read_reg;
	nor->write_reg = write_reg;
	nor->read = flash_read;
	nor->write = flash_write;
	nor->erase = flash_erase;
	nor->read_proto = SNOR_HWCAPS_READ_4_4_4;
	nor->write_proto = SNOR_HWCAPS_PP_4_4_4;
	nor->mtd.name = "spiflash";
	nor->addr_width = 3;
	nor->erase_opcode = SPINOR_OP_BE_4K;

	ret = spi_nor_scan(nor, NULL, &hwcaps);

	if (ret)
		return ret;

	ret = mtd_device_register(&nor->mtd, NULL, 0);
	if (ret) {
		dev_err(&pdev->dev, "Fail to register device\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id litex_of_match[] = {
	{ .compatible = "litex,spiflash_quad" }
};

MODULE_DEVICE_TABLE(of, litex_of_match);


static struct platform_driver litex_spi_flash_driver = {
	.probe	= spi_flash_litex_init,
	.driver = {
		.name = "litex-spiflash",
		.of_match_table = of_match_ptr(litex_of_match)
	},
};
module_platform_driver(litex_spi_flash_driver);

MODULE_DESCRIPTION("LiteX SPI Flash driver");
MODULE_AUTHOR("Antmicro <www.antmicro.com>");
MODULE_LICENSE("GPL v2");
