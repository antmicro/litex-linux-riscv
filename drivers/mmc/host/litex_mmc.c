// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Antmicro <www.antmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <linux/fs.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/core.h>
#include <linux/clk.h>
#include <linux/genhd.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/genhd.h>
#include <linux/types.h>
#include <linux/blkdev.h>
#include <linux/pm_runtime.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/types.h>
#include <linux/delay.h>

struct sdclk_regs {
	u32 mmcm_reset[1];
	u32 mmcm_read[1];
	u32 mmcm_write[1];
	u32 mmcm_drdy[1];
	u32 mmcm_adr[1];
	u32 mmcm_dat_w[2];
	u32 mmcm_dat_r[2];
};

struct sdcore_regs {
	u32 argument[4];
	u32 command[4];
	u32 issue_cmd[1];
	u32 response[17];
	u32 cmdevt[4];
	u32 dataevt[4];
	u32 blocksize[2];
	u32 blockcount[4];
	u32 datatimeout[4];
	u32 cmdtimeout[4];
	u32 datawcrcclear[1];
	u32 datawcrcvalids[4];
	u32 datawcrcerrors[4];
};

struct sddatareader_regs {
	u32 reset[1];
	u32 start[1];
	u32 done[1];
	u32 errors[4];
};

struct sddatawriter_regs {
	u32 reset[1];
	u32 start[1];
	u32 done[1];
};

struct sdtimer_regs {
	u32 load[4];
	u32 reload[4];
	u32 en[1];
	u32 update_value[1];
	u32 value[4];
	u32 ev_status[1];
	u32 ev_pending[1];
	u32 ev_enable[1];
};

#define CSR_SDCLK_BASE 0xf0003000L
#define CSR_SDCLK_MMCM_RESET_OFFSET 0x00L
#define CSR_SDCLK_MMCM_RESET_SIZE 1
#define CSR_SDCLK_MMCM_READ_OFFSET 0x04L
#define CSR_SDCLK_MMCM_READ_SIZE 1
#define CSR_SDCLK_MMCM_WRITE_OFFSET 0x08L
#define CSR_SDCLK_MMCM_WRITE_SIZE 1
#define CSR_SDCLK_MMCM_DRDY_OFFSET 0x0cL
#define CSR_SDCLK_MMCM_DRDY_SIZE 1
#define CSR_SDCLK_MMCM_ADR_OFFSET 0x10L
#define CSR_SDCLK_MMCM_ADR_SIZE 1
#define CSR_SDCLK_MMCM_DAT_W_OFFSET 0x14L
#define CSR_SDCLK_MMCM_DAT_W_SIZE 2
#define CSR_SDCLK_MMCM_DAT_R_OFFSET 0x1cL
#define CSR_SDCLK_MMCM_DAT_R_SIZE 2

#define SYSTEM_CLOCK_FREQUENCY (u32)1e8
#define SD_CLOCK_FREQUENCY (u32)25 //In MHz
#define MAX_CCNT ((1 << 16) - 1)
#define MAX_NR_SG 1
#define RW_THRESHOLD 32

#define SDCARD_CTRL_DATA_TRANSFER_NONE  0
#define SDCARD_CTRL_DATA_TRANSFER_READ  (1 << 5)
#define SDCARD_CTRL_DATA_TRANSFER_WRITE (2 << 5)

#define SDCARD_CTRL_RESPONSE_NONE  0
#define SDCARD_CTRL_RESPONSE_SHORT 1
#define SDCARD_CTRL_RESPONSE_LONG  2

#define SD_SPEED_SDR12  0
#define SD_SPEED_SDR25  1
#define SD_SPEED_SDR50  2
#define SD_SPEED_SDR104 3
#define SD_SPEED_DDR50  4

#define SD_OK 0
#define SD_CRCERROR 1
#define SD_TIMEOUT 2
#define SD_WRITEERROR 3

#define SD_GROUP_COMMANDSYSTEM  1
#define SD_GROUP_DRIVERSTRENGTH 2
#define SD_GROUP_POWERLIMIT     3

#define SD_DRIVER_STRENGTH_B 0
#define SD_DRIVER_STRENGTH_A 1
#define SD_DRIVER_STRENGTH_C 2
#define SD_DRIVER_STRENGTH_D 3

struct litex_mmc_host {
	struct mmc_host *mmc;
	struct platform_device *dev;

	struct {
		__iomem struct sdclk_regs *sdclk;
		__iomem struct sdcore_regs *sdcore;
		__iomem struct sddatareader_regs *sddatareader;
		__iomem struct sddatawriter_regs *sddatawriter;
		__iomem struct sdtimer_regs *sdtimer;
	} regs;
	__iomem u8 *sdread;
	__iomem u8 *sdwrite;

	u32 resp[4];
	u16 rca;

	unsigned clock;
	unsigned char bus_width;
	bool is_bus_width_set;
	bool app_cmd;
};

struct litex_mmc_config {
	int (*get_cd)(int module);
	int (*get_ro)(int module);
	void (*set_power)(int module, bool on);
	u32 max_freq;
	u32 caps;
	u8 nr_sg;
};

static void sdclk_mmcm_write(unsigned int adr, unsigned int data)
{
	volatile u8 *ptr = (volatile u8 *)CSR_SDCLK_BASE;
	iowrite32(adr, ptr + CSR_SDCLK_MMCM_ADR_OFFSET);

	iowrite32(data >> 8, ptr + CSR_SDCLK_MMCM_DAT_W_OFFSET);
	iowrite32(data, ptr + CSR_SDCLK_MMCM_DAT_W_OFFSET + 0x04);
	iowrite32(1, ptr + CSR_SDCLK_MMCM_WRITE_OFFSET);

	while (!ioread32(ptr + CSR_SDCLK_MMCM_DRDY_OFFSET))
		;
}

//**********************************************
//LITESDCARD CODE FOR CLK CONFIG FROM ENJOY-DIGITAL REPO

static void sdclk_set_config(unsigned int m, unsigned int d)
{
	/* clkfbout_mult = m */
	if (m % 2)
		sdclk_mmcm_write(0x14, 0x1000 | ((m / 2) << 6) | (m / 2 + 1));
	else
		sdclk_mmcm_write(0x14, 0x1000 | ((m / 2) << 6) | m / 2);
	/* divclk_divide = d */
	if (d == 1)
		sdclk_mmcm_write(0x16, 0x1000);
	else if (d % 2)
		sdclk_mmcm_write(0x16, ((d / 2) << 6) | (d / 2 + 1));
	else
		sdclk_mmcm_write(0x16, ((d / 2) << 6) | d / 2);
	/* clkout0_divide = 10 */
	sdclk_mmcm_write(0x8, 0x1000 | (5 << 6) | 5);
	/* clkout1_divide = 2 */
	sdclk_mmcm_write(0xa, 0x1000 | (1 << 6) | 1);
}

/* FIXME: add vco frequency check */
static void sdclk_get_config(unsigned int freq, unsigned int *best_m,
			     unsigned int *best_d)
{
	unsigned int ideal_m, ideal_d;
	unsigned int bm, bd;
	unsigned int m, d;
	unsigned int diff_current;
	unsigned int diff_tested;

	ideal_m = freq;
	ideal_d = 10000;

	bm = 1;
	bd = 0;
	for (d = 1; d <= 128; d++)
		for (m = 2; m <= 128; m++) {
			/* common denominator is d*bd*ideal_d */
			diff_current = abs(d * ideal_d * bm - d * bd * ideal_m);
			diff_tested = abs(bd * ideal_d * m - d * bd * ideal_m);
			if (diff_tested < diff_current) {
				bm = m;
				bd = d;
			}
		}
	*best_m = bm;
	*best_d = bd;
}

void sdclk_set_clk(unsigned int freq)
{
	unsigned int clk_m, clk_d;

	sdclk_get_config(1000 * freq, &clk_m, &clk_d);
	sdclk_set_config(clk_m, clk_d);
}

//**********************************************


//TODO: FIX HACK
static void litex_mmc_read_rsp(struct litex_mmc_host *host, u32 resp[4])
{
	static const u8 reg_size = sizeof(host->regs.sdcore->response)/4;
	unsigned offset;
	unsigned i;
	u8 value;
	u32 resp_tmp[5];

	resp[0] = resp[1] = resp[2] = resp[3] = 0;
	resp_tmp[0] = resp_tmp[1] = resp_tmp[2] = resp_tmp[3] = resp_tmp[4] = 0;

	for (i = 0; i < reg_size; i++) {
		offset = reg_size - 1 - i;
		value = ioread32(&host->regs.sdcore->response[offset]) & 0xFF;
		resp_tmp[4 - i/4] |= value << (8 * (i%4));
	}
	resp[0] = resp_tmp[1];
	resp[1] = resp_tmp[2];
	resp[2] = resp_tmp[3];
	resp[3] = resp_tmp[4];
}

static void _write_reg(struct litex_mmc_host *host, u32 data, __iomem u32 *reg,
		      u8 reg_size)
{
	u8 i;
	for (i = 0; i < reg_size; i++) {
		iowrite32(data >> (reg_size - 1 - i) * 8, &reg[i]);
	}
}
#define write_reg(host, data, reg) _write_reg(host, data, reg, sizeof(reg)/4)

static u32 _read_reg(struct litex_mmc_host *host, __iomem const u32 *reg,
		     u8 reg_size)
{
	u32 value = 0;
	u8 i;
	for (i = 0; i < reg_size; i++) {
		value <<= 8;
		value |= ioread32(&reg[i]) & 0xFF;
	}
	return value;
}
#define read_reg(host, reg) _read_reg(host, reg, sizeof(reg)/4)

static void litex_issue_cmd(struct litex_mmc_host *host) {
	write_reg(host, 1, host->regs.sdcore->issue_cmd);
}

static inline u32 read_cmdevt(struct litex_mmc_host *host) {
	return read_reg(host, host->regs.sdcore->cmdevt);
}

static inline u32 read_dataevt(struct litex_mmc_host *host) {
	return read_reg(host, host->regs.sdcore->dataevt);
}

static int sdcard_wait_cmd_done(struct litex_mmc_host *host) {
	u32 cmdevt;
	// TODO: add timeout
	while (1) {
		cmdevt = read_cmdevt(host);
		if (cmdevt & 0x1) {
			if (cmdevt & 0x4) {
				return SD_TIMEOUT;
			}
			if (cmdevt & 0x8) {
				return SD_CRCERROR;
			}
			return SD_OK;
		}
	}
}

static int sdcard_wait_data_done(struct litex_mmc_host *host) {
	u32 dataevt;
	while (1) {
		dataevt = read_dataevt(host);
		if (dataevt & 0x1) {
			if (dataevt & 0x4) {
				return SD_TIMEOUT;
			}
			if (dataevt & 0x8) {
				return SD_CRCERROR;
			}
			return SD_OK;
		}
	}
}

static int send_cmd(struct litex_mmc_host *host, u8 cmd, u32 arg,
		    u8 response_len, u8 transfer) {
	int status = SD_OK;
	unsigned done = 0;

	if (transfer == SDCARD_CTRL_DATA_TRANSFER_READ) {
		write_reg(host, 1, host->regs.sddatareader->reset);
		write_reg(host, 1, host->regs.sddatareader->start);
	} else if (transfer == SDCARD_CTRL_DATA_TRANSFER_WRITE) {
		write_reg(host, 1, host->regs.sddatawriter->reset);
		write_reg(host, 1, host->regs.sddatawriter->start);
	}
	write_reg(host, arg, host->regs.sdcore->argument);
	write_reg(host, cmd << 8 | transfer | response_len,
		  host->regs.sdcore->command);
	litex_issue_cmd(host);

	status = sdcard_wait_cmd_done(host);
	if (response_len != SDCARD_CTRL_RESPONSE_NONE) {
		litex_mmc_read_rsp(host, host->resp);
	}

	if(status != SD_OK) {
		return status;
	}

	if (!host->app_cmd && cmd == SD_SEND_RELATIVE_ADDR) {
		host->rca = (host->resp[3] >> 16) & 0xffff;
	}

	if (transfer) {
		status = sdcard_wait_data_done(host);

		if(status != SD_OK) {
			return status;
		}


		if (transfer == SDCARD_CTRL_DATA_TRANSFER_READ) {
			while((done & 1) == 0) {
				done = read_reg(host, host->regs.sddatareader->done);
			}
		} else if (transfer == SDCARD_CTRL_DATA_TRANSFER_WRITE) {
			while((done & 1) == 0) {
				done = read_reg(host, host->regs.sddatawriter->done);
			}
		}
	}

	return status;
}


// CMD55
static inline int send_app_cmd(struct litex_mmc_host *host) {
	return send_cmd(host, MMC_APP_CMD, host->rca << 16,
			SDCARD_CTRL_RESPONSE_SHORT,
			SDCARD_CTRL_DATA_TRANSFER_NONE);
}

// ACMD6
static inline int send_app_set_bus_width_cmd(struct litex_mmc_host *host, u32 width) {
	return send_cmd(host, SD_APP_SET_BUS_WIDTH, width,
			SDCARD_CTRL_RESPONSE_SHORT,
			SDCARD_CTRL_DATA_TRANSFER_NONE);
}

static void litex_mmc_initsd(struct litex_mmc_host *host)
{
	write_reg(host, 1 << 19, host->regs.sdcore->cmdtimeout);
	write_reg(host, 1 << 19, host->regs.sdcore->datatimeout);

	return;
}

/*
 Return values for the get_ro callback should be:
*   0 for a read/write card
*   1 for a read-only card
*   -ENOSYS when not supported (equal to NULL callback)
*   or a negative errno value when something bad happened
*/

static int litex_get_ro(struct mmc_host *mmc)
{
	struct platform_device *pdev = to_platform_device(mmc->parent);
	struct litex_mmc_config *config = pdev->dev.platform_data;
	int ro;

	return 0;

	if (config && config->get_ro) {
		ro = config->get_ro(pdev->id);
	}
	ro = mmc_gpio_get_ro(mmc);
	return ro;
}

static int litex_get_cd(struct mmc_host *mmc)
{
	//int ret = mmc_gpio_get_cd(mmc);
	struct platform_device *pdev = to_platform_device(mmc->parent);
	struct litex_mmc_config *config = pdev->dev.platform_data;
	if (config && config->get_cd) {
	}//??
	return 1; //Hack - should return ret
}

/*
 * Send request to a card. Command, data transfer, things like this.
 * Call mmc_request_done() when finished.
 */
static void litex_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct litex_mmc_host *host = mmc_priv(mmc);
	struct platform_device *pdev = to_platform_device(mmc->parent);
	struct device *dev = &pdev->dev;
	int status;

	struct mmc_data *data = mrq->data;
	struct mmc_command *cmd = mrq->cmd;
	int retries = cmd->retries;

	u32 response_len = SDCARD_CTRL_RESPONSE_NONE;
	u32 transfer = SDCARD_CTRL_DATA_TRANSFER_NONE;

	u8 data_buf[512];

	int i;

	if (cmd->flags & MMC_RSP_136) {
		response_len = SDCARD_CTRL_RESPONSE_LONG;
	} else if (cmd->flags & MMC_RSP_PRESENT) {
		response_len = SDCARD_CTRL_RESPONSE_SHORT;
	}

	if (data) {
		if (mrq->data->flags & MMC_DATA_READ) {
			transfer = SDCARD_CTRL_DATA_TRANSFER_READ;
		} else if (mrq->data->flags & MMC_DATA_WRITE) {
			transfer = SDCARD_CTRL_DATA_TRANSFER_WRITE;

			sg_copy_to_buffer(data->sg, 1, data_buf, data->sg->length);
			for (i = 0; i < data->sg->length; ++i) {
				iowrite8(data_buf[i], &host->sdwrite[i]);
			}
		} else {
			dev_warn(dev, "Data present without read or write flag.\n");
		}

		// This is a good time (i.e. last moment) to finally set bus
		// width. Ofc if not set yet.
		if (!host->is_bus_width_set) {
			if (!host->app_cmd) {
				send_app_cmd(host);
			}
			status = send_app_set_bus_width_cmd(host, host->bus_width);
			if (status == SD_OK) {
				host->is_bus_width_set = true;
			}
			if (host->app_cmd) {
				send_app_cmd(host);
			}
		}

		write_reg(host, data->blksz, host->regs.sdcore->blocksize);
		// We have 512B buffer. See if we can control clock and refill
		// the buffer during transmission.
		write_reg(host, 1, host->regs.sdcore->blockcount);
	}

	do {
		status = send_cmd(host, cmd->opcode, cmd->arg, response_len, transfer);
	} while(retries-- > 0 && status != SD_OK);

	switch(status) {
		case SD_TIMEOUT:        cmd->error = -ETIMEDOUT; break;
		case SD_CRCERROR:       cmd->error = -EILSEQ; break;
		case SD_WRITEERROR:     cmd->error = -EINVAL; break;
		default:                cmd->error = 0; break;
	}

	if (response_len == SDCARD_CTRL_RESPONSE_SHORT) {
		// FIXME: reverse response reading
		cmd->resp[0] = host->resp[3];
		cmd->resp[1] = host->resp[2];
	} else if (SDCARD_CTRL_RESPONSE_LONG) {
		cmd->resp[0] = host->resp[0];
		cmd->resp[1] = host->resp[1];
		cmd->resp[2] = host->resp[2];
		cmd->resp[3] = host->resp[3];
	}

	if (status == SD_OK && transfer == SDCARD_CTRL_DATA_TRANSFER_READ) {
		u8 data_buf[512];
		int i;

		data->bytes_xfered = data->blksz <= 512 ? data->blksz : 512;
		for (i = 0; i < data->bytes_xfered; ++i) {
			data_buf[i] = ioread8(host->sdread+i);
		}
		sg_copy_from_buffer(data->sg, 1, data_buf, data->bytes_xfered);
	} else if (status == SD_OK && transfer == SDCARD_CTRL_DATA_TRANSFER_WRITE) {
		// XXX: needed?
		data->bytes_xfered = data->blksz <= 512 ? data->blksz : 512;
	}

	host->app_cmd = (cmd->opcode == MMC_APP_CMD && status == SD_OK);

	mmc_request_done(mmc, mrq);
}

static void litex_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct litex_mmc_host *host = mmc_priv(mmc);

	//switch (ios->power_mode) {
	//case MMC_POWER_UP:
	//	return;
	//}

	if (ios->clock != host->clock) {
		sdclk_set_clk((ios->clock / (u32)1e6));
		host->clock = ios->clock;
	}

	// Technically there should be some code forsetting requested bus width.
	// We can't send "send_bus_width" command to the card at this moment
	// because the card is not in the right state (the command is only accepted
	// in "trans" state). Requested width should be stored and send when it is
	// possible (see litex_request), but litesdcard supports only 4 bit bus
	// width, so it is known what must be set.
}

static const struct mmc_host_ops litex_mmc_ops = {

	.get_cd = litex_get_cd,
	.get_ro = litex_get_ro,
	.request = litex_request,
	.set_ios = litex_set_ios,
};

static int litex_mmc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct litex_mmc_host *host;
	struct device_node *node;
	struct mmc_host *mmc;
	int ret;
	int i;

	static const size_t resource_ptr_offsets[] = {
		offsetof(struct litex_mmc_host, regs.sdclk),
		offsetof(struct litex_mmc_host, regs.sdcore),
		offsetof(struct litex_mmc_host, regs.sddatareader),
		offsetof(struct litex_mmc_host, regs.sddatawriter),
		offsetof(struct litex_mmc_host, sdread),
		offsetof(struct litex_mmc_host, sdwrite),
		offsetof(struct litex_mmc_host, regs.sdtimer),
	};

	node = pdev->dev.of_node;
	if (!node) {
		return -ENODEV;
	}

	host = devm_kzalloc(&pdev->dev, sizeof(struct litex_mmc_host),
			    GFP_KERNEL);
	if (!host) {
		return -ENOMEM;
	}

	mmc = mmc_alloc_host(sizeof(struct litex_mmc_host), &pdev->dev);
	if (!mmc) {
		return -ENOMEM;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;
	// Initial state
	host->clock = 0;
	// litesdcard only supports 4-bit bus width
	host->bus_width = MMC_BUS_WIDTH_4;
	host->is_bus_width_set = false;
	host->app_cmd = false;

	for (i = 0; i < ARRAY_SIZE(resource_ptr_offsets); ++i) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res) {
			dev_err(&pdev->dev, "Resource %d missing\n", i);
			ret = -ENODEV;
			goto err_exit;
		}
		*(void **)((char *)host + resource_ptr_offsets[i]) =
			devm_ioremap_resource(&pdev->dev, res);
		if (!(*(void **)((char *)host + resource_ptr_offsets[i]))) {
			dev_err(&pdev->dev, "Couldn't map resource %d\n", i);
			ret = -ENOMEM;
			goto err_exit;
		}
	}

	ret = mmc_of_parse(mmc);
	if (ret) {
		goto err_exit;
	}

	sdclk_set_clk(SD_CLOCK_FREQUENCY);
	litex_mmc_initsd(host);

	mmc->caps = MMC_CAP_WAIT_WHILE_BUSY | MMC_CAP_DRIVER_TYPE_D;
	mmc->caps2 = MMC_CAP2_NO_SDIO | MMC_CAP2_FULL_PWR_CYCLE | MMC_CAP2_NO_WRITE_PROTECT;
	mmc->ops = &litex_mmc_ops;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

	mmc->max_segs = MAX_NR_SG;
	mmc->max_seg_size = 512;
	mmc->max_blk_size = 512;
	mmc->max_blk_count = 1;
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;

	mmc->f_min = SD_CLOCK_FREQUENCY  * 1e6;
	mmc->f_max = SD_CLOCK_FREQUENCY  * 1e6;

	platform_set_drvdata(pdev, host);

	ret = mmc_add_host(mmc);
	if (ret < 0) {
		goto err_exit;
	}

	return 0;

err_exit:
	mmc_free_host(mmc);
	return ret;
}

static int litex_mmc_remove(struct platform_device *pdev)
{
	struct litex_mmc_host *host = dev_get_drvdata(&pdev->dev);

	mmc_remove_host(host->mmc);
	mmc_free_host(host->mmc);

	return 0;
}

static const struct of_device_id litex_match[] = {
	{ .compatible =
		"litex,mmc" },
		{}
	};

MODULE_DEVICE_TABLE(of, litex_match);

static struct platform_driver litex_mmc_driver = {
	.driver =
		{
			.name = "litex-mmc",
			.of_match_table = of_match_ptr(litex_match),
		},
	.probe = litex_mmc_probe,
	.remove = litex_mmc_remove,
};

module_platform_driver(litex_mmc_driver);

MODULE_DESCRIPTION("LiteX SDCard driver");
MODULE_AUTHOR("Antmicro <www.antmicro.com>");
MODULE_LICENSE("GPL v2");
