/*
MMC子系统介绍

MMC代码分布

MMC子系统代码主要在drivers/mmc目录下，共有三个目录：

         Card：存放闪存卡(块设备)的相关驱动，如MMC/SD卡设备驱动，SDIOUART；

         Host：针对不同主机端的SDHC、MMC控制器的驱动，这部分需要由驱动工程师来完成；

         Core：整个MMC的核心层，这部分完成不同协议和规范的实现，为host层和设备驱动层提供接口函数。 

Linux MMC子系统主要分成三个部分：

MMC核心层：完成不同协议和规范的实现，为host层和设备驱动层提供接口函数。MMC核心层由三个部分组成：MMC，SD和SDIO，分别为三类设备驱动提供接口函数；

Host 驱动层：针对不同主机端的SDHC、MMC控制器的驱动；

Client 驱动层：针对不同客户端的设备驱动程序。如SD卡、T-flash卡、SDIO接口的GPS和wi-fi等设备驱动。

*/
//需要添加sd的平台设备信息
static void __init smdk4x12_machine_init(void)
{
	//...
	//初始化相关平台信息

	#ifdef CONFIG_EXYNOS4_DEV_DWMCI
	exynos_dwmci_set_platdata(&exynos_dwmci_pdata);
#endif

	//注册i2c的平台设备信息

	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
	//...
}

static struct i2c_board_info smdk4x12_i2c_devs0[] __initdata = {
	&exynos_device_dwmci,
};


static struct dw_mci_board exynos_dwmci_pdata __initdata = {
	.num_slots		= 1,
	.quirks			= DW_MCI_QUIRK_BROKEN_CARD_DETECTION | DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz			= 100 * 1000 * 1000,
	.caps			= MMC_CAP_UHS_DDR50 | MMC_CAP_1_8V_DDR |
				MMC_CAP_8_BIT_DATA | MMC_CAP_CMD23,
	.fifo_depth		= 0x80,
	.detect_delay_ms	= 200,
	.hclk_name		= "dwmci",
	.cclk_name		= "sclk_dwmci",
	.cfg_gpio		= exynos_dwmci_cfg_gpio,
};

static void exynos_dwmci_cfg_gpio(int width)
{
	unsigned int gpio;

	for (gpio = EXYNOS4_GPK0(0); gpio < EXYNOS4_GPK0(2); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
	}

	switch (width) {
	case MMC_BUS_WIDTH_8:
		for (gpio = EXYNOS4_GPK1(3); gpio <= EXYNOS4_GPK1(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(4));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
		}
	case MMC_BUS_WIDTH_4:
		for (gpio = EXYNOS4_GPK0(3); gpio <= EXYNOS4_GPK0(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
		}
		break;
	case MMC_BUS_WIDTH_1:
		gpio = EXYNOS4_GPK0(3);
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
	default:
		break;
	}
}

void __init exynos_dwmci_set_platdata(struct dw_mci_board *pd)
{
	struct dw_mci_board *npd;

	npd = s3c_set_platdata(pd, sizeof(struct dw_mci_board),
			&exynos_device_dwmci);

	if (!npd->init)
		npd->init = exynos_dwmci_init;
	if (!npd->get_bus_wd)
		npd->get_bus_wd = exynos_dwmci_get_bus_wd;
	if (!npd->set_io_timing)
		npd->set_io_timing = exynos_dwmci_set_io_timing;
}

struct platform_device exynos_device_dwmci = {
	.name		= "dw_mmc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(exynos_dwmci_resource),
	.resource	= exynos_dwmci_resource,
	.dev		= {
		.dma_mask		= &exynos_dwmci_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data	= &exynos_dwci_pdata,
	},
};

static struct resource exynos_dwmci_resource[] = {
	[0] = DEFINE_RES_MEM(EXYNOS4_PA_DWMCI, SZ_4K),
	[1] = DEFINE_RES_IRQ(EXYNOS4_IRQ_DWMCI),
};

static struct dw_mci_board exynos_dwci_pdata = {
	.num_slots			= 1,
	.quirks				= DW_MCI_QUIRK_BROKEN_CARD_DETECTION,
	.bus_hz				= 80 * 1000 * 1000,
	.detect_delay_ms	= 200,
	.init				= exynos_dwmci_init,
	.get_bus_wd			= exynos_dwmci_get_bus_wd,
};

static u64 exynos_dwmci_dmamask = DMA_BIT_MASK(32);

/*-----------------------------------------------------------------------------------
samsung Dw_mmc-pltfm.c分析
-----------------------------------------------------------------------------------*/
static int dw_mci_pltfm_probe(struct platform_device *pdev)
{
	struct dw_mci *host;
	struct resource	*regs;
	int ret;

	host = kzalloc(sizeof(struct dw_mci), GFP_KERNEL);//在内核模块中申请分配内存
	if (!host)
		return -ENOMEM;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);//申请设备资源
	if (!regs) {
		ret = -ENXIO;
		goto err_free;
	}

	 /* 
	 通过平台设备platform_device获得IRQ 
 	 platform_get_irq其实是调用platform_get_resource(dev, IORESOURCE_IRQ, num) 
	 */  
	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0) {
		ret = host->irq;
		goto err_free;
	}

	host->dev = pdev->dev;
	host->irq_flags = 0;
	host->pdata = pdev->dev.platform_data;
	ret = -ENOMEM;
	host->regs = ioremap(regs->start, resource_size(regs));
	if (!host->regs)
		goto err_free;
	/*使用platform_set_drvdata（）将其保存到platform_device中，
	在需要使用的时候再使用platform_get_drvdata（）来获取它。*/
	platform_set_drvdata(pdev, host);
	ret = dw_mci_probe(host);//非常重要的函数在dw_mmc.c中定义，后面详细分析
	if (ret)
		goto err_out;
	return ret;
err_out:
	iounmap(host->regs);
err_free:
	kfree(host);
	return ret;
}
/*-----------------------------------------------------------------------------------
samsung Dw_mmc.c分析
-----------------------------------------------------------------------------------*/
int dw_mci_probe(struct dw_mci *host)
{
	int width, i = 0, ret = 0;
	u32 fifo_size;
	struct dw_mci_board *brd = NULL;

	if (!host->pdata || !host->pdata->init) {
		dev_err(&host->dev,
			"Platform data must supply init function\n");
		return -ENODEV;
	}

	if (!host->pdata->select_slot && host->pdata->num_slots > 1) {
		dev_err(&host->dev,
			"Platform data must supply select_slot function\n");
		return -ENODEV;
	}

	if (!host->pdata->bus_hz) {
		dev_err(&host->dev,
			"Platform data must supply bus speed\n");
		return -ENODEV;
	}

	host->hclk = clk_get(&host->dev, host->pdata->hclk_name);
	if (IS_ERR(host->hclk)) {
		dev_err(&host->dev,
				"failed to get hclk\n");
		ret = PTR_ERR(host->hclk);
		goto err_freehost;
	}
	clk_enable(host->hclk);

	host->cclk = clk_get(&host->dev, host->pdata->cclk_name);
	if (IS_ERR(host->cclk)) {
		dev_err(&host->dev,
				"failed to get cclk\n");
		ret = PTR_ERR(host->cclk);
		goto err_free_hclk;
	}
	clk_enable(host->cclk);

	if (host->pdata->cfg_gpio)
		host->pdata->cfg_gpio(MMC_BUS_WIDTH_8);

	host->pdata->bus_hz = 66 * 1000 * 1000;

	host->bus_hz = host->pdata->bus_hz;
	host->quirks = host->pdata->quirks;

	/* Set Phase Shift Register */
	if (soc_is_exynos4212() || soc_is_exynos4412()) {
		brd = host->pdata;
		brd->sdr_timing = 0x00010001;
		brd->ddr_timing = 0x00010001;
	}

	spin_lock_init(&host->lock);
	INIT_LIST_HEAD(&host->queue);

	/*
	 * Get the host data width - this assumes that HCON has been set with
	 * the correct values.
	 探测函数中赋值的收发函数，这里根据数据类型,总共有三种，包括16位、32位和64位的收发，程序类似，我们以16位为例讲述。
	 */
	i = (mci_readl(host, HCON) >> 7) & 0x7;
	if (!i) {
		host->push_data = dw_mci_push_data16;
		host->pull_data = dw_mci_pull_data16;
		width = 16;
		host->data_shift = 1;
	} else if (i == 2) {
		host->push_data = dw_mci_push_data64;
		host->pull_data = dw_mci_pull_data64;
		width = 64;
		host->data_shift = 3;
	} else {
		/* Check for a reserved value, and warn if it is */
		WARN((i != 1),
		     "HCON reports a reserved host data width!\n"
		     "Defaulting to 32-bit access.\n");
		host->push_data = dw_mci_push_data32;
		host->pull_data = dw_mci_pull_data32;
		width = 32;
		host->data_shift = 2;
	}

	/* Reset all blocks SD复位函数*/
	if (!mci_wait_reset(&host->dev, host))
		return -ENODEV;

	/*DW的SD卡驱动中关于操作DAM部分
	由于这里是探测函数，在设备定义中也没有对pdata->dma_ops成员进行初始化*/
	host->dma_ops = host->pdata->dma_ops;
	dw_mci_init_dma(host);

	/* Clear the interrupts for the host controller */
	mci_writel(host, RINTSTS, 0xFFFFFFFF);
	mci_writel(host, INTMASK, 0); /* disable all mmc interrupt first */

	/* Put in max timeout */
	mci_writel(host, TMOUT, 0xFFFFFFFF);

	/*
	 * FIFO threshold settings  RxMark  = fifo_size / 2 - 1,
	 *                          Tx Mark = fifo_size / 2 DMA Size = 8
	 */
	if (!host->pdata->fifo_depth) {
		/*
		 * Power-on value of RX_WMark is FIFO_DEPTH-1, but this may
		 * have been overwritten by the bootloader, just like we're
		 * about to do, so if you know the value for your hardware, you
		 * should put it in the platform data.
		 */
		fifo_size = mci_readl(host, FIFOTH);
		fifo_size = 1 + ((fifo_size >> 16) & 0xfff);
	} else {
		fifo_size = host->pdata->fifo_depth;
	}
	host->fifo_depth = fifo_size;
	host->fifoth_val = ((0x2 << 28) | ((fifo_size/2 - 1) << 16) |
			((fifo_size/2) << 0));
	mci_writel(host, FIFOTH, host->fifoth_val);

	/* disable clock to CIU */
	mci_writel(host, CLKENA, 0);
	mci_writel(host, CLKSRC, 0);

	/*
	 tasklet就象一个内核定时器, 在一个"软中断"的上下文中执行(以原子模式),常用
	 在硬件中断处理中，使得可以使得复杂的任务安全地延后到以后的时间处理。task_init
	 建立一个tasklet，然后调用函数 tasklet_schedule将这个tasklet放在 tasklet_vec
	 链表的头部，并唤醒后台线程 ksoftirqd。
	*/
	tasklet_init(&host->tasklet, dw_mci_tasklet_func, (unsigned long)host);
	host->card_workqueue = alloc_workqueue("dw-mci-card",
			WQ_MEM_RECLAIM | WQ_NON_REENTRANT, 1);
	if (!host->card_workqueue)
		goto err_dmaunmap;
	INIT_WORK(&host->card_work, dw_mci_work_routine_card);
	ret = request_irq(host->irq, dw_mci_interrupt, host->irq_flags, "dw-mci", host);
	if (ret)
		goto err_workqueue;

	if (host->pdata->num_slots)
		host->num_slots = host->pdata->num_slots;
	else
		host->num_slots = ((mci_readl(host, HCON) >> 1) & 0x1F) + 1;

	/*
	 * Enable interrupts for command done, data over, data empty, card det,
	 * receive ready and error such as transmit, receive timeout, crc error
	 */
	mci_writel(host, RINTSTS, 0xFFFFFFFF);
	mci_writel(host, INTMASK, SDMMC_INT_CMD_DONE | SDMMC_INT_DATA_OVER |
		   SDMMC_INT_TXDR | SDMMC_INT_RXDR |
		   DW_MCI_ERROR_FLAGS | SDMMC_INT_CD);
	mci_writel(host, CTRL, SDMMC_CTRL_INT_ENABLE); /* Enable mci interrupt */

	/* We need at least one slot to succeed */
	for (i = 0; i < host->num_slots; i++) {
		/*在dw_mci_init_slot 中就调用mmc_alloc_host/mmc_add_host 向kernel 添加这个mmc host。
	在mmc_alloc_host 中最重要的就是    INIT_DELAYED_WORK(&host->detect, mmc_rescan);，后面
	就可以通过mmc_rescan 来发现这个host上连接的设备*/
		ret = dw_mci_init_slot(host, i);
		if (ret) {
			ret = -ENODEV;
			goto err_init_slot;
		}
	}

	/*
	 * In 2.40a spec, Data offset is changed.
	 * Need to check the version-id and set data-offset for DATA register.
	 */
	host->verid = SDMMC_GET_VERID(mci_readl(host, VERID));
	dev_info(&host->dev, "Version ID is %04x\n", host->verid);

	if (host->verid < DW_MMC_240A)
		host->data_offset = DATA_OFFSET;
	else
		host->data_offset = DATA_240A_OFFSET;

	dev_info(&host->dev, "DW MMC controller at irq %d, "
		 "%d bit host data width, "
		 "%u deep fifo\n",
		 host->irq, width, fifo_size);
	if (host->quirks & DW_MCI_QUIRK_IDMAC_DTO)
		dev_info(&host->dev, "Internal DMAC interrupt fix enabled.\n");

	return 0;

err_init_slot:
	/* De-init any initialized slots */
	while (i > 0) {
		if (host->slot[i])
			dw_mci_cleanup_slot(host->slot[i], i);
		i--;
	}
	free_irq(host->irq, host);

err_workqueue:
	destroy_workqueue(host->card_workqueue);

err_dmaunmap:
	if (host->use_dma && host->dma_ops->exit)
		host->dma_ops->exit(host);
	dma_free_coherent(&host->dev, PAGE_SIZE,
			  host->sg_cpu, host->sg_dma);

	if (host->vmmc) {
		regulator_disable(host->vmmc);
		regulator_put(host->vmmc);
	}
	clk_disable(host->cclk);
	clk_put(host->cclk);
err_free_hclk:
	clk_disable(host->hclk);
	clk_put(host->hclk);
err_freehost:
	return ret;
}

/*接下来分析下探测函数中赋值的收发函数，这里根据数据类型，总共有三种，包括16位、32位和64位的收发，程序类似，我们以16位为例讲述。
host->push_data = dw_mci_push_data16;
host->pull_data = dw_mci_pull_data16;
发函数：*/
static void dw_mci_push_data16(struct dw_mci *host, void *buf, int cnt)
{
    u16 *pdata = (u16 *)buf;
    WARN_ON(cnt % 2 != 0);
    cnt = cnt >> 1;    //接收到的长度是以字节为单位的，所以16位的除以2,如果是32位的就右移两位，除以4,以此类推
    while (cnt > 0) {
        mci_writew(host, DATA, *pdata++);
        cnt--;
    }
}
//收函数：
static void dw_mci_pull_data16(struct dw_mci *host, void *buf, int cnt)
{
    u16 *pdata = (u16 *)buf;
    WARN_ON(cnt % 2 != 0);
    cnt = cnt >> 1;    //接收到的长度是以字节为单位的，所以16位的除以2,如果是32位的就右移两位，除以4,以此类推
    while (cnt > 0) {
        *pdata++ = mci_readw(host, DATA);
        cnt--;
    }
}

//分析下SD复位函数mci_wait_reset：
static bool mci_wait_reset(struct device *dev, struct dw_mci *host)
{
    unsigned long timeout = jiffies + msecs_to_jiffies(500);
    unsigned int ctrl;
    mci_writel(host, CTRL, (SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET |
                SDMMC_CTRL_DMA_RESET));    //设置控制寄存器
    /* wait till resets clear */
    do {
        ctrl = mci_readl(host, CTRL);
        if (!(ctrl & (SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET |
                  SDMMC_CTRL_DMA_RESET)))
            return true;
    } while (time_before(jiffies, timeout));    //等待超时时间内
    dev_err(dev, "Timeout resetting block (ctrl %#x)\n", ctrl);
    return false;
}

static void dw_mci_init_dma(struct dw_mci *host)
{
	/* Alloc memory for sg translation */
	host->sg_cpu = dma_alloc_coherent(&host->dev, PAGE_SIZE,
					  &host->sg_dma, GFP_KERNEL);//DMA内存申请
	if (!host->sg_cpu) {
		dev_err(&host->dev, "%s: could not alloc DMA memory\n",
			__func__);
		goto no_dma;
	}

	/* Determine which DMA interface to use */
#ifdef CONFIG_MMC_DW_IDMAC
	host->dma_ops = &dw_mci_idmac_ops;//DMA操作函数集合
	dev_info(&host->dev, "Using internal DMA controller.\n");
#endif
		//......
}

static int __init dw_mci_init_slot(struct dw_mci *host, unsigned int id)
{
	struct mmc_host *mmc;
	struct dw_mci_slot *slot;

	mmc = mmc_alloc_host(sizeof(struct dw_mci_slot), &host->dev);//core中申请host
	if (!mmc)
		return -ENOMEM;

	slot = mmc_priv(mmc);// 将dw_mci_slot作为mmc_host的私有数据，mmc_host->private = dw_mci_slot
	slot->id = id;
	slot->mmc = mmc;
	slot->host = host;

	mmc->ops = &dw_mci_ops;//mmc操作函数集
	mmc->f_min = DIV_ROUND_UP(host->bus_hz, 510);
	mmc->f_max = host->bus_hz;

	if (host->pdata->get_ocr)
		mmc->ocr_avail = host->pdata->get_ocr(id);
	else
		mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

	/*
	 * Start with slot power disabled, it will be enabled when a card
	 * is detected.
	 */
	if (host->pdata->setpower)
		host->pdata->setpower(id, 0);

	if (host->pdata->caps)
		mmc->caps = host->pdata->caps;

	if (host->pdata->caps2)
		mmc->caps2 = host->pdata->caps2;

	if (host->pdata->get_bus_wd)
		if (host->pdata->get_bus_wd(slot->id) >= 4)
			mmc->caps |= MMC_CAP_4_BIT_DATA;

	if (host->pdata->quirks & DW_MCI_QUIRK_HIGHSPEED)
		mmc->caps |= MMC_CAP_SD_HIGHSPEED | MMC_CAP_MMC_HIGHSPEED;

	if (mmc->caps2 & MMC_CAP2_POWEROFF_NOTIFY)
		mmc->power_notify_type = MMC_HOST_PW_NOTIFY_SHORT;
	else
		mmc->power_notify_type = MMC_HOST_PW_NOTIFY_NONE;

	if (host->pdata->blk_settings) {
		mmc->max_segs = host->pdata->blk_settings->max_segs;
		mmc->max_blk_size = host->pdata->blk_settings->max_blk_size;
		mmc->max_blk_count = host->pdata->blk_settings->max_blk_count;
		mmc->max_req_size = host->pdata->blk_settings->max_req_size;
		mmc->max_seg_size = host->pdata->blk_settings->max_seg_size;
	} else {
		/* Useful defaults if platform data is unset. */
#ifdef CONFIG_MMC_DW_IDMAC
		mmc->max_segs = host->ring_size;
		mmc->max_blk_size = 65536;
		mmc->max_blk_count = host->ring_size;
		mmc->max_seg_size = 0x1000;
		mmc->max_req_size = mmc->max_seg_size * mmc->max_blk_count;
#else
		mmc->max_segs = 64;
		mmc->max_blk_size = 65536; /* BLKSIZ is 16 bits */
		mmc->max_blk_count = 512;
		mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
		mmc->max_seg_size = mmc->max_req_size;
#endif /* CONFIG_MMC_DW_IDMAC */
	}
	//......

	host->slot[id] = slot;
	mmc_add_host(mmc);//初始化host
	//.......
}

static const struct mmc_host_ops dw_mci_ops = {
	.request		= dw_mci_request,
	.pre_req		= dw_mci_pre_req,
	.post_req		= dw_mci_post_req,
	.set_ios		= dw_mci_set_ios,
	.get_ro			= dw_mci_get_ro,
	.get_cd			= dw_mci_get_cd,
	.enable_sdio_irq	= dw_mci_enable_sdio_irq,
};


//mmc_rescan 里面既要检测 sd 卡，又要检测 mmc 卡的，我们就照着一个往下走，假定有个人插入了 MMC 卡，那就应该走下面这几行：

       err = mmc_send_op_cond(host, 0, &ocr);
       if (!err) {
              if (mmc_attach_mmc(host, ocr))
                     mmc_power_off(host);
              goto out;
       }

//mmc_send_op_cond 这个函数据说是读了一下卡的什么值，这个值是什么意义我也不清楚，这就像检测 FLASH 时读 FLASH 的 ID 一样，网卡也是这样的，不用管这个值的意义了，只要知道它能标识是一个 MMC 卡插入就行了。如果取这个值没有错误的话就得进 mmc_attach_mmc 了：

	/*
  * Starting point for MMC card init.
  */

int mmc_attach_mmc(struct mmc_host *host, u32 ocr)  // core/mmc.c

{
       int err;
……
       mmc_attach_bus_ops(host);         // 这个与总线的电源管理有关，暂时跳过
……
       /*

         * Detect and init the card.

         */
       err = mmc_init_card(host, host->ocr, NULL);
       if (err)
              goto err;
……
       mmc_release_host(host);
       err = mmc_add_card(host->card);
       if (err)
              goto remove_card;
       return 0;
remove_card:
……
err:
……
       return err;

}
//还是找几个关键函数来看 mmc_init_card 从函数名来看就是初始化一个 card ，这个 card 
//就用 struct mmc_card 结构来描述，然后又调用 mmc_add_card 将卡设备添加到了内核，先来看 mmc_init_card 都做了些什么事情：
static int mmc_init_card(struct mmc_host *host, u32 ocr,
       struct mmc_card *oldcard)
{
       struct mmc_card *card;
       int err;
       u32 cid[4];
       unsigned int max_dtr;

……
              /*

                * Allocate card structure.

                */
              card = mmc_alloc_card(host, &mmc_type);

              if (IS_ERR(card)) {

                     err = PTR_ERR(card);

                     goto err;

              }
              card->type = MMC_TYPE_MMC;
              card->rca = 1;
              memcpy(card->raw_cid, cid, sizeof(card->raw_cid));
……
              host->card = card;
       return 0;
free_card:
……
err:
……
    return err;

}
//将与硬件操作相关的全部删掉，最后对我们有用的也就这几行了 mmc_alloc_card 申请了一个 struct mmc_card 结构，
//然后给 card->type 赋上 MMC_TYPE_MMC ，最后将 card 又赋给了 host->card ，这和具体硬件还是挺像的，因为一个主控制器一般就插一个卡，有卡时 host->card 有值，没有卡时 host->card 自己就是 NULL 了。
//       钻进 mmc_alloc_card 里面来看看：
/*
  * Allocate and initialise a new MMC card structure.
  */
struct mmc_card *mmc_alloc_card(struct mmc_host *host, struct device_type *type)
{
       struct mmc_card *card;
       card = kzalloc(sizeof(struct mmc_card), GFP_KERNEL);

       if (!card)
              return ERR_PTR(-ENOMEM);
       card->host = host;
       device_initialize(&card->dev);
       card->dev.parent = mmc_classdev(host);
       card->dev.bus = &mmc_bus_type;
       card->dev.release = mmc_release_card;
       card->dev.type = type;
       return card;
}

/*Struct mmc_card 结构里面包含了一个 struct device 结构， mmc_alloc_card 不但申请了内存，而且
	 还填充了 struct device 中的几个成员，尤其 card->dev.bus = &mmc_bus_type; 这一句要重点对待。
   申请一个 mmc_card 结构，并简单初始化后， mmc_init_card 的使命就完成了，然后再调用
   mmc_add_card 将这个 card 设备添加到内核。 mmc_add_card 其实很简单，就是调用 device_add 将
    card->dev 添加到内核当中去。
   知道总线模型这个东西的人都明白，理到 device_add 里面总线就应该有动作了，具体是哪个
   总线呢？那就得看你调用 device_add 时送的那个 dev 里面指定的是哪个总线了，我们送的 
   card->dev ，那么 card->dev.bus 具体指向什么呢？很明现是那个 mmc_bus_type ：
*/
static struct bus_type mmc_bus_type = {
       .name             = "mmc",
       .dev_attrs       = mmc_dev_attrs,
       .match           = mmc_bus_match,
       .uevent           = mmc_bus_uevent,
       .probe            = mmc_bus_probe,
       .remove          = mmc_bus_remove,
       .suspend  = mmc_bus_suspend,
       .resume          = mmc_bus_resume,
};

//在 device_add 里面，设备对应的总线会拿着你这个设备和挂在这个总线上的所有驱动程序去
//匹配（ match ），此时会调用 match 函数，如果匹配到了就会调用总线的 probe 函数或驱动
//的 probe 函数，那我们看一下这里的 mmc_bus_match 是如何进行匹配的：
static int mmc_bus_match(struct device *dev, struct device_driver *drv)
{
       return 1;
}

//看来 match 永远都能成功，那就去执行 probe 吧：

static int mmc_bus_probe(struct device *dev)
{
       struct mmc_driver *drv = to_mmc_driver(dev->driver);
       struct mmc_card *card = dev_to_mmc_card(dev);
       return drv->probe(card);
}

/*这里就有点麻烦了，在这个函数里面又调用了一下 drv->probe() ，那这个 drv 是什么呢？
上面有： struct mmc_driver *drv = to_mmc_driver(dev->driver);

match 函数总是返回 1 ，那看来只要是挂在这条总线上的 driver 都有可能跑到这里来了，
事实的确也是这样的，不过好在挂在这条总线上的 driver 只有一个，它是这样定义的：
*/
static struct mmc_driver mmc_driver = {
       .drv        = {
              .name      = "mmcblk",
       },
       .probe            = mmc_blk_probe,
       .remove          = mmc_blk_remove,
       .suspend      = mmc_blk_suspend,
       .resume          = mmc_blk_resume,
};

//看到这里时， card/core/host 几个已经全部被扯进来了，边看 mmc_driver 中的几个函数，他们几个如何联系起来也就慢慢明白了。那我们继续吧。
/*第三阶段：

前面已经看到了，在总线的 probe 里面调用了 drv->probe, 而这个函数就对应的是 mmc_blk_probe ，
具体这个 mmc_driver 是怎么挂到 mmc_bus 上的，自己去看 mmc_blk_init() ，就几行代码，应该不难。
*/
static int mmc_blk_probe(struct mmc_card *card) // 来自 card/block.c
{
       struct mmc_blk_data *md;
       int err;
……
       md = mmc_blk_alloc(card);
       if (IS_ERR(md))
              return PTR_ERR(md);
……
       add_disk(md->disk);
       return 0;
  out:
     mmc_blk_put(md);
       return err;

}

//还是捡重要的函数看，一看到这个函数最后调用了 add_disk ，你应该可以想到些什么吧？如果你不知道我在说些什么，那我估计你没有看过 LDD3 ，或者看了也是走马观花了。我来告诉你：如果看到 add_disk ，那说明前面一定会有 alloc_disk 和初始化队列的动作，在 mmc_blk_probe 时面没有体现出来，那就看 mmc_blk_alloc(card) 那一行：
static struct mmc_blk_data *mmc_blk_alloc(struct mmc_card *card)
{
       struct mmc_blk_data *md;
       int devidx, ret;
       
       devidx = find_first_zero_bit(dev_use, MMC_NUM_MINORS);
       if (devidx >= MMC_NUM_MINORS)
              return ERR_PTR(-ENOSPC);
       __set_bit(devidx, dev_use);

       md = kzalloc(sizeof(struct mmc_blk_data), GFP_KERNEL);
       if (!md) {
              ret = -ENOMEM;
              goto out;
       }

       /*
         * Set the read-only status based on the supported commands
         * and the write protect switch.
         */
       md->read_only = mmc_blk_readonly(card);
       md->disk = alloc_disk(1 << MMC_SHIFT);
       if (md->disk == NULL) {
              ret = -ENOMEM;
             goto err_kfree;
       }
       spin_lock_init(&md->lock);
       md->usage = 1;
       ret = mmc_init_queue(&md->queue, card, &md->lock);
       if (ret)
              goto err_putdisk;
       md->queue.issue_fn = mmc_blk_issue_rq;
       md->queue.data = md;
       md->disk->major   = MMC_BLOCK_MAJOR;
       md->disk->first_minor = devidx << MMC_SHIFT;
       md->disk->fops = &mmc_bdops;
       md->disk->private_data = md;
       md->disk->queue = md->queue.queue;
       md->disk->driverfs_dev = &card->dev;
       /*
         * As discussed on lkml, GENHD_FL_REMOVABLE should:
         *
         * - be set for removable media with permanent block devices
         * - be unset for removable block devices with permanent media
         *
         * Since MMC block devices clearly fall under the second
         * case, we do not set GENHD_FL_REMOVABLE.  Userspace
         * should use the block device creation/destruction hotplug
         * messages to tell when the card is present.
         */
       sprintf(md->disk->disk_name, "mmcblk%d", devidx);
       blk_queue_logical_block_size(md->queue.queue, 512);
       if (!mmc_card_sd(card) && mmc_card_blockaddr(card)) {
              /*
                * The EXT_CSD sector count is in number or 512 byte
                * sectors.
                */
              set_capacity(md->disk, card->ext_csd.sectors);
       } else {
              /*
                * The CSD capacity field is in units of read_blkbits.
                * set_capacity takes units of 512 bytes.
                */
              set_capacity(md->disk,
                     card->csd.capacity << (card->csd.read_blkbits - 9));
       }
       return md;
  err_putdisk:
       put_disk(md->disk);
  err_kfree:
       kfree(md);
  out:
       return ERR_PTR(ret);
}

/*看到这个函数的代码，我们自然就回忆起了块设备驱动的整个套路了：
1.       分配、初始化请求队列，并绑定请求队列和请求函数。
2.       分配，初始化 gendisk ，给 gendisk 的 major ， fops ， queue 等成员赋值，最后添加 gendisk 。
3.       注册块设备驱动。
我们看看 MMC 卡驱动程序有没有按这个套路走，
1 、 mmc_init_queue 初始了队列，并将 mmc_blk_issue_rq; 函数绑定成请求函数；
2 、 alloc_disk 分配了 gendisk 结构，并初始化了 major ， fops ，和 queue ；
3 、最后调用 add_disk 将块设备加到 KERNEL 中去。
到这里虽然 mmc_blk_probe 已经结束了，但我们别停下来。记得 LDD3 上在讲 sbull 实例时说过， 
add_disk 的调用标志着一个块设备驱动将被激活，所以在这之前必须把其它所有准备工作全部做好，
作者为什么会这样说是有理由的，因为在 add_disk 里面 kernel 会去调用你绑定到队列中的请求函数，
目的是去你的块设备上读分区表。而且是在 add_disk 内部就要做的，而不是 add_disk 返回后再做，
具体为什么会这样，去看 add_disk 的代码实现就知道了。
既然要调用请求函数去读，那我们就来看看请求函数： mmc_blk_issue_rq*/
static int mmc_blk_issue_rq(struct mmc_queue *mq, struct request *req)
{
       struct mmc_blk_data *md = mq->data;
       struct mmc_card *card = md->queue.card;
       struct mmc_blk_request brq;
       int ret = 1, disable_multi = 0;

       do {
              mmc_wait_for_req(card->host, &brq.mrq);
              /*
                * A block was successfully transferred.
                */
              spin_lock_irq(&md->lock);
              ret = __blk_end_request(req, 0, brq.data.bytes_xfered);
              spin_unlock_irq(&md->lock);
       } while (ret);
       return 1;

}

/*这个函数实在太长了，好在我们不用全部看，大部分读数据的准备代码和出错处理的代码已经被我删掉了，只要知道读数据都是在这里完成的就够了。看不懂这个函数的，拿上 LDD3 找个人少的地方，将 sbull 研究透了也就明白这个函数了。不过这个函数里涉及的东西还挺不少，“散列表”，“回弹”都在这里出现了，有时间慢慢去研究吧。

       在块设备驱动当中你只需要抓住请求队列和请求函数就可以了，具体那些 block_device_operations 里面赋值的函数可不像字符设备驱动里面那么受关注了。

 

       分析到这里， MMC/SD 卡的驱动整个构架基本也就很明析了，说简单了就是做了两件事：

1.       卡的检测；

2.       卡数据的读取。

最后再将这两个过程大概串一下：

1.       卡的检测：

S3cmci_probe(host/s3cmci.c)

       Mmc_alloc_host(core/core.c)

              Mmc_rescan(core/core.c)

                     Mmc_attach_mmc(core/mmc.c)

                            Mmc_init_card(core/mmc.c)

                            mmc_add_card(core/bus.c)

                                   device_add

                                          mmc_bus_match(core/bus.c)

                                          mmc_bus_probe(core/bus.c)

                                                 mmc_blk_probe(card/block.c)

                                                        alloc_disk/add_disk

2.       读写数据：

mmc_blk_issue_rq （ card/block.c ）

       mmc_wait_for_req(core/core.c)

              mmc_start_request(core/core.c)

                     host->ops->request(host, mrq)   // s3cmci 中 s3cmci_request

 

MMC/SD 卡的驱动分析完了，是不是有些复杂，不过这样设计的目的是为了分层，让具体平台的驱动编写更加省事。
*/










