// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2020 Antmicro <www.antmicro.com>
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
#include <linux/litex.h>

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
	u32 carddet[1];
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

#define MAX_NR_SG 1

#define SDCARD_CTRL_DATA_TRANSFER_NONE  0
#define SDCARD_CTRL_DATA_TRANSFER_READ  (1 << 5)
#define SDCARD_CTRL_DATA_TRANSFER_WRITE (2 << 5)

#define SDCARD_CTRL_RESPONSE_NONE  0
#define SDCARD_CTRL_RESPONSE_SHORT 1
#define SDCARD_CTRL_RESPONSE_LONG  2

#define SD_OK 0
#define SD_CRCERROR 1
#define SD_TIMEOUT 2
#define SD_WRITEERROR 3

#define write_reg(reg, data) litex_set_reg(reg, sizeof(reg)/4, data)

#define read_reg(reg) litex_get_reg(reg, sizeof(reg)/4)

struct litex_mmc_host {
	struct mmc_host *mmc;
	struct platform_device *dev;

	struct {
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

void sdclk_set_clk(struct device *dev, unsigned int clk_freq)
{
	struct clk *clkout0 = devm_clk_get(dev, NULL);
	dev_info(dev, "Setting clk freq to: %d\n", clk_freq);
	if(IS_ERR(clkout0)) {
		dev_err(dev, "Could not get clk: CLKOUT0\n");
		return;
	}
	clk_set_rate(clkout0, clk_freq);
}


//TODO: Max length of response should be 16 bytes, but due to a bug in litesdcard,
//LONG response is 17 bytes long, we are skipping here the last byte
static void litex_mmc_read_rsp(struct litex_mmc_host *host, u32 resp[4])
{
	static const u8 reg_size = sizeof(host->regs.sdcore->response) / 4;
	unsigned offset;
	unsigned i;
	u8 value;

	resp[0] = resp[1] = resp[2] = resp[3] = 0;

	for (i = 0; i < reg_size - 1; i++) {
		offset = reg_size - 1 - i;
		value = ioread32(&host->regs.sdcore->response[offset]) & 0xFF;
		resp[3 - i / 4] |= value << (8 * (i % 4));
	}
}

static void litex_issue_cmd(struct litex_mmc_host *host) {
	write_reg(host->regs.sdcore->issue_cmd, 1);
}

static int sdcard_wait_done(struct litex_mmc_host *host, u32 *reg) {
	u32 dataevt;
	//TODO: add timeout
	while (1) {
		dataevt = litex_get_reg(reg, sizeof(reg));
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
	__iomem const u32 * read_mem;

	if (transfer == SDCARD_CTRL_DATA_TRANSFER_READ) {
		write_reg(host->regs.sddatareader->reset, 1);
		write_reg(host->regs.sddatareader->start, 1);
	} else if (transfer == SDCARD_CTRL_DATA_TRANSFER_WRITE) {
		write_reg(host->regs.sddatawriter->reset, 1);
		write_reg(host->regs.sddatawriter->start, 1);
	}
	write_reg(host->regs.sdcore->argument, arg);
	write_reg(host->regs.sdcore->command, cmd << 8 | transfer | response_len);
	litex_issue_cmd(host);

	status = sdcard_wait_done(host, host->regs.sdcore->cmdevt);
	if (response_len != SDCARD_CTRL_RESPONSE_NONE) {
		litex_mmc_read_rsp(host, host->resp);
	}

	if(status != SD_OK) {
		return status;
	}

	if (!host->app_cmd && cmd == SD_SEND_RELATIVE_ADDR) {
		host->rca = (host->resp[3] >> 16) & 0xffff;
	}

	if (!transfer) {
		return status;
	}
	status = sdcard_wait_done(host, host->regs.sdcore->dataevt);

	if(status != SD_OK) {
		return status;
	}

        read_mem = transfer == SDCARD_CTRL_DATA_TRANSFER_READ ?
		host->regs.sddatareader->done : host->regs.sddatawriter->done;

	while((done & 1) == 0) {
		done = read_reg((void*)read_mem);
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
	write_reg(host->regs.sdcore->cmdtimeout, 1<<19);
	write_reg(host->regs.sdcore->datatimeout, 1<<19);
}

/*
 Return values for the get_ro callback should be:
*   0 for a read/write card
*   1 for a read-only card
*   -ENOSYS when not supported (equal to NULL callback)
*   or a negative errno value when something bad happened
*/
//TODO: Not sure about this, is this check is enough?
static int litex_get_ro(struct mmc_host *mmc)
{
	struct platform_device *pdev = to_platform_device(mmc->parent);
	struct litex_mmc_config *config = pdev->dev.platform_data;
	int ro;

	if (config && config->get_ro) {
		ro = config->get_ro(pdev->id);
	} else {
		ro = mmc_gpio_get_ro(mmc);
	}

	return ro;
}

static int litex_get_cd(struct mmc_host *mmc)
{
	struct litex_mmc_host *host = mmc_priv(mmc);
	int gpio_cd = mmc_gpio_get_cd(mmc);

	if (!mmc_card_is_removable(host->mmc))
		return 1;

	if(gpio_cd >= 0)
		return !!gpio_cd;

	return !read_reg(host->regs.sdcore->carddet);
}

static int litex_set_bus_width(struct litex_mmc_host *host) {
	int status;
	// Check if last cmd was app cmd, if not, send app cmd
	if (!host->app_cmd) {
		send_app_cmd(host);
	}
	status = send_app_set_bus_width_cmd(host, host->bus_width);
	if (status == SD_OK) {
		host->is_bus_width_set = true;
	}
	// If last command was app cmd, redo it for this command
	if (host->app_cmd) {
		send_app_cmd(host);
	}
	return status;
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
			int write_length = min(data->sg->length, (u32)512);
			transfer = SDCARD_CTRL_DATA_TRANSFER_WRITE;

			sg_copy_to_buffer(data->sg, 1, data_buf, write_length);
			for (i = 0; i < write_length; ++i) {
				iowrite8(data_buf[i], &host->sdwrite[i]);
			}
		} else {
			dev_warn(dev, "Data present without read or write flag.\n");
		}

		// This is a good time (i.e. last moment) to finally set bus
		// width. Ofc if not set yet.
		if (!host->is_bus_width_set) {
			if(litex_set_bus_width(host) != SD_OK) {
				dev_warn(dev, "Could not set bus width!\n");
			}
		}

		write_reg(host->regs.sdcore->blocksize, data->blksz);
		// We have 512B buffer. See if we can control clock and refill
		// the buffer during transmission.
		write_reg(host->regs.sdcore->blockcount, 1);
	}

	do {
		status = send_cmd(host, cmd->opcode, cmd->arg, response_len, transfer);
	} while(retries-- > 0 && status != SD_OK);

	switch(status) {
		case SD_OK:		cmd->error = 0; break;
		case SD_TIMEOUT:        cmd->error = -ETIMEDOUT; break;
		case SD_CRCERROR:       cmd->error = -EILSEQ; break;
		case SD_WRITEERROR:     cmd->error = -EINVAL; break;
		default:                cmd->error = -EINVAL; break;
	}

	if (response_len == SDCARD_CTRL_RESPONSE_SHORT) {
		cmd->resp[0] = host->resp[3];
		cmd->resp[1] = host->resp[2];
	} else if (SDCARD_CTRL_RESPONSE_LONG) {
		cmd->resp[0] = host->resp[0];
		cmd->resp[1] = host->resp[1];
		cmd->resp[2] = host->resp[2];
		cmd->resp[3] = host->resp[3];
	}
	
	if (status == SD_OK) {
		if (transfer == SDCARD_CTRL_DATA_TRANSFER_READ) {
			data->bytes_xfered = min(data->blksz, (u32)512);
			for (i = 0; i < data->bytes_xfered; ++i) {
				data_buf[i] = ioread8(host->sdread+i);
			}
			sg_copy_from_buffer(data->sg, 1, data_buf, data->bytes_xfered);
		} else if (transfer == SDCARD_CTRL_DATA_TRANSFER_WRITE) {
			data->bytes_xfered = min(data->blksz, (u32)512);
		}
	}

	host->app_cmd = (cmd->opcode == MMC_APP_CMD && status == SD_OK);

	mmc_request_done(mmc, mrq);
}

static void litex_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct litex_mmc_host *host;
	
	host = mmc_priv(mmc);

	if (ios->clock != host->clock) {
		sdclk_set_clk(&host->dev->dev, ios->clock);
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
	struct clk *clkout0;
	int ret;
	int i;

	static const size_t resource_ptr_offsets[] = {
		offsetof(struct litex_mmc_host, regs.sdcore),
		offsetof(struct litex_mmc_host, regs.sddatareader),
		offsetof(struct litex_mmc_host, regs.sddatawriter),
		offsetof(struct litex_mmc_host, sdread),
		offsetof(struct litex_mmc_host, sdwrite),
		offsetof(struct litex_mmc_host, regs.sdtimer),
	};

	clkout0 = devm_clk_get(&pdev->dev, NULL);
	if(IS_ERR(clkout0)) {
		return -EPROBE_DEFER;
	}
	clk_prepare_enable(clkout0);

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
	host->dev = pdev;
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

	mmc->f_min = 0; // Min clk freq is 0
	mmc->f_max = 50 * 1e6; // Max clk freq is 50Mhz

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
	{ .compatible = "litex,mmc" },
		{},
};

MODULE_DEVICE_TABLE(of, litex_match);

static struct platform_driver litex_mmc_driver = {
	.driver = {
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
