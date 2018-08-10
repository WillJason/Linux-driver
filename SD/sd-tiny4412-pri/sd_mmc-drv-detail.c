/*三星平台SD/MMC驱动主要有两个文件sdhci.c和sdhci-s3c.c,核心驱动在后者里面。*/
//需要添加sd的平台设备信息
static void __init smdk4x12_machine_init(void)
{
	//...
	//初始化相关平台信息

	s3c_sdhci2_set_platdata(&smdk4x12_hsmmc2_pdata);
	s3c_sdhci3_set_platdata(&smdk4x12_hsmmc3_pdata);
	

	//注册i2c的平台设备信息

	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
	//...
}

static struct s3c_sdhci_platdata smdk4x12_hsmmc2_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
};
static struct s3c_sdhci_platdata smdk4x12_hsmmc3_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_PERMANENT,
};

static struct i2c_board_info smdk4x12_i2c_devs0[] __initdata = {
	&s3c_device_hsmmc2,
	&s3c_device_hsmmc3,
}


void s3c_sdhci2_set_platdata(struct s3c_sdhci_platdata *pd)
{
	s3c_sdhci_set_platdata(pd, &s3c_hsmmc2_def_platdata);
}

struct platform_device s3c_device_hsmmc2 = {
	.name		= "s3c-sdhci",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(s3c_hsmmc2_resource),
	.resource	= s3c_hsmmc2_resource,
	.dev		= {
		.dma_mask		= &samsung_device_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data		= &s3c_hsmmc2_def_platdata,
	},
};
static struct resource s3c_hsmmc2_resource[] = {
	[0] = DEFINE_RES_MEM(S3C_PA_HSMMC2, SZ_4K),
	[1] = DEFINE_RES_IRQ(IRQ_HSMMC2),
};

struct s3c_sdhci_platdata s3c_hsmmc2_def_platdata = {
	.max_width	= 4,
	.host_caps	= (MMC_CAP_4_BIT_DATA |
			   MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
};

/*-----------------------------------------------------------------------------------
samsung sdhci-s3c.c分析
-----------------------------------------------------------------------------------*/
static int __devinit sdhci_s3c_probe(struct platform_device *pdev)
{
	struct s3c_sdhci_platdata *pdata;
	struct sdhci_s3c_drv_data *drv_data;
	struct device *dev = &pdev->dev;
	struct sdhci_host *host;
	struct sdhci_s3c *sc;
	struct resource *res;
	int ret, irq, ptr, clks;

	if (!pdev->dev.platform_data) {
		dev_err(dev, "no device data specified\n");
		return -ENOENT;
	}

	//platform_get_irq（）调用platform_get_resource， 会返回一个start, 即可用的中断号。
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "no irq specified\n");
		return irq;
	}

	/* 实现mmc_host和sdhci_host的分配 */
	host = sdhci_alloc_host(dev, sizeof(struct sdhci_s3c));
	if (IS_ERR(host)) {
		dev_err(dev, "sdhci_alloc_host() failed\n");
		return PTR_ERR(host);
	}

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		ret = -ENOMEM;
		goto err_io_clk;
	}
	memcpy(pdata, pdev->dev.platform_data, sizeof(*pdata));

	/*调用到platform_get_device_id(pdev)，通过id_entry中的driver_data判断匹配的到底是cbpmci_driver_ids中的哪一组ID，
比如上面例子中的"cbp-sdmmc"中匹配的是第二组，这样就可以再Probe的判断到底是什么平台了*/
	drv_data = sdhci_s3c_get_driver_data(pdev);
	
	//sc = host->private；
	//host->private已经被sdhci_alloc_host函数alloc了额外的sizeof(struct sdhci_s3c)给private成员 
	sc = sdhci_priv(host);

	sc->host = host;
	sc->pdev = pdev;
	sc->pdata = pdata;
	sc->ext_cd_gpio = pdata->ext_cd_gpio;

	/*使用platform_set_drvdata（）将其保存到platform_device中，
在需要使用的时候再使用platform_get_drvdata（）来获取它。*/
	platform_set_drvdata(pdev, host);

	sc->clk_io = clk_get(dev, "hsmmc");//获取时钟源
	if (IS_ERR(sc->clk_io)) {
		dev_err(dev, "failed to get io clock\n");
		ret = PTR_ERR(sc->clk_io);
		goto err_io_clk;
	}

	/* enable the local io clock and keep it running for the moment. */
	clk_enable(sc->clk_io);//使能

	for (clks = 0, ptr = 0; ptr < MAX_BUS_CLK; ptr++) {
		struct clk *clk;
		char name[14];

		snprintf(name, 14, "mmc_busclk.%d", ptr);
		clk = clk_get(dev, name);
		if (IS_ERR(clk)) {
			continue;
		}

		clks++;
		sc->clk_bus[ptr] = clk;

		/*
		 * save current clock index to know which clock bus
		 * is used later in overriding functions.
		 */
		sc->cur_clk = ptr;

		clk_enable(clk);

		dev_info(dev, "clock source %d: %s (%ld Hz)\n",
			 ptr, name, clk_get_rate(clk));
	}

	if (clks == 0) {
		dev_err(dev, "failed to find any bus clocks\n");
		ret = -ENOENT;
		goto err_no_busclks;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);////申请设备资源
	/*映射寄存器地址到内核虚拟地址空间，申请中断和配置接口*/
	host->ioaddr = devm_request_and_ioremap(&pdev->dev, res);
	if (!host->ioaddr) {
		dev_err(dev, "failed to map registers\n");
		ret = -ENXIO;
		goto err_req_regs;
	}

	/* Ensure we have minimal gpio selected CMD/CLK/Detect */
	if (pdata->cfg_gpio)
		pdata->cfg_gpio(pdev, pdata->max_width);

	host->hw_name = "samsung-hsmmc";
	host->ops = &sdhci_s3c_ops;
	host->quirks = 0;
	host->irq = irq;

	/* Setup quirks for the controller */
	host->quirks |= SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC;
	host->quirks |= SDHCI_QUIRK_NO_HISPD_BIT;
	if (drv_data)
		host->quirks |= drv_data->sdhci_quirks;

#ifndef CONFIG_MMC_SDHCI_S3C_DMA

	/* we currently see overruns on errors, so disable the SDMA
	 * support as well. */
	host->quirks |= SDHCI_QUIRK_BROKEN_DMA;

#endif /* CONFIG_MMC_SDHCI_S3C_DMA */

	/* It seems we do not get an DATA transfer complete on non-busy
	 * transfers, not sure if this is a problem with this specific
	 * SDHCI block, or a missing configuration that needs to be set. */
	host->quirks |= SDHCI_QUIRK_NO_BUSY_IRQ;

	/* This host supports the Auto CMD12 */
	host->quirks |= SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12;

	/* Samsung SoCs need BROKEN_ADMA_ZEROLEN_DESC */
	host->quirks |= SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC;

	if (pdata->cd_type == S3C_SDHCI_CD_NONE ||
	    pdata->cd_type == S3C_SDHCI_CD_PERMANENT)
		host->quirks |= SDHCI_QUIRK_BROKEN_CARD_DETECTION;

	if (pdata->cd_type == S3C_SDHCI_CD_GPIO)
		host->quirks |= SDHCI_QUIRK_BROKEN_CARD_DETECTION;

	if (pdata->cd_type == S3C_SDHCI_CD_PERMANENT)
		host->mmc->caps = MMC_CAP_NONREMOVABLE;

	switch (pdata->max_width) {
	case 8:
		host->mmc->caps |= MMC_CAP_8_BIT_DATA;
	case 4:
		host->mmc->caps |= MMC_CAP_4_BIT_DATA;
		break;
	}

	if (pdata->pm_caps)
		host->mmc->pm_caps |= pdata->pm_caps;

	host->quirks |= (SDHCI_QUIRK_32BIT_DMA_ADDR |
			 SDHCI_QUIRK_32BIT_DMA_SIZE);

	/* HSMMC on Samsung SoCs uses SDCLK as timeout clock */
	host->quirks |= SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK;

	/*
	 * If controller does not have internal clock divider,
	 * we can use overriding functions instead of default.
	 */
	if (host->quirks & SDHCI_QUIRK_NONSTANDARD_CLOCK) {
		sdhci_s3c_ops.set_clock = sdhci_cmu_set_clock;
		sdhci_s3c_ops.get_min_clock = sdhci_cmu_get_min_clock;
		sdhci_s3c_ops.get_max_clock = sdhci_cmu_get_max_clock;
	}

	/* It supports additional host capabilities if needed */
	if (pdata->host_caps)
		host->mmc->caps |= pdata->host_caps;

	if (pdata->host_caps2)
		host->mmc->caps2 |= pdata->host_caps2;

	pm_runtime_enable(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, 50);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_suspend_ignore_children(&pdev->dev, 1);
	
	
	//通过sdhci_add_host函数来注册sdhci_host
	// sdhci_add_host调用了mmc_add_host将这个控制器设备添加到内核中
	//将host添加入mmc管理，其本质即是添加host的class_dev
	ret = sdhci_add_host(host);
	/*
	这里我们着重分析加入MMC子系统的详细过程：sdhci_add_host(struct sdhci_host *host)
	此函数的主要功能为，转换sdhci_host结构体到mmc_host内核标准结构体，并挂接上相关标准函数，加入MMC子系统中
	
	1. 设置MMC操作接口：mmc->ops = &sdhci_ops; 【后面需要详解】  
	2. 添加两个tasklet并启动计时器：
	tasklet_init(&host->card_tasklet, sdhci_tasklet_card, (unsigned long)host);【用于MMC插槽上的状态变化】
	tasklet_init(&host->finish_tasklet, sdhci_tasklet_finish, (unsigned long)host);【用于命令传输完成后的处理】
	setup_timer(&host->timer, sdhci_timeout_timer, (unsigned long)host);【用于等待硬件中断】
	3. 中断映射：
	ret = request_irq(host->irq, sdhci_irq, IRQF_SHARED, mmc_hostname(mmc), host);【中断处理】
	4. 初始化MMC设备，包括复位控制器：
	sdhci_init(host, 0);
	5. 加入MMC子系统：
	mmc_add_host(mmc);
	6. 使能设备探测功能：
	sdhci_enable_card_detection(host);
	*/
	if (ret) {
		dev_err(dev, "sdhci_add_host() failed\n");
		pm_runtime_forbid(&pdev->dev);
		pm_runtime_get_noresume(&pdev->dev);
		goto err_req_regs;
	}

	/* The following two methods of card detection might call
	   sdhci_s3c_notify_change() immediately, so they can be called
	   only after sdhci_add_host(). Setup errors are ignored. */
	if (pdata->cd_type == S3C_SDHCI_CD_EXTERNAL && pdata->ext_cd_init)
		pdata->ext_cd_init(&sdhci_s3c_notify_change);
	if (pdata->cd_type == S3C_SDHCI_CD_GPIO &&
	    gpio_is_valid(pdata->ext_cd_gpio))
		sdhci_s3c_setup_card_detect_gpio(sc);

	return 0;

 err_req_regs:
	for (ptr = 0; ptr < MAX_BUS_CLK; ptr++) {
		if (sc->clk_bus[ptr]) {
			clk_disable(sc->clk_bus[ptr]);
			clk_put(sc->clk_bus[ptr]);
		}
	}

 err_no_busclks:
	clk_disable(sc->clk_io);
	clk_put(sc->clk_io);

 err_io_clk:
	sdhci_free_host(host);

	return ret;
}



struct sdhci_host *sdhci_alloc_host(struct device *dev,
    size_t priv_size)
{
    struct mmc_host *mmc;
    struct sdhci_host *host;

    WARN_ON(dev == NULL);

/* 实现mmc_host和sdhci_host的分配 */
    mmc = mmc_alloc_host(sizeof(struct sdhci_host) + priv_size, dev);   // 分配一个struct mmc_host
    // 分配mmc_host的同时也分配了sizeof(struct sdhci_host) + priv_size的私有数据空间，这部分就是作为sdhci_host及其私有数据使用的。
    // 具体参考《mmc core——host模块说明》
    if (!mmc)
        return ERR_PTR(-ENOMEM);

/* 实现mmc_host和sdhci_host的关联操作 */
    host = mmc_priv(mmc);   // 将sdhci_host作为mmc_host的私有数据，mmc_host->private = sdhci_host
    host->mmc = mmc;   // 关联sdhci_host和mmc_host，sdhci_host->mmc = mmc_host

/* sdhci_host的锁的初始化工作 */
    spin_lock_init(&host->lock);   // 初始化sdhci_host 的占有锁
    mutex_init(&host->ios_mutex);   // 初始化sdhci_host 设置io setting的互斥锁

    return host;   // 将struct sdhci_host 返回
}
/*
综上， 
mmc_host->private = sdhci_host 
sdhci_host->mmc = mmc_host
*/

int sdhci_add_host(struct sdhci_host *host)
{
	/*重点的几个部分*/
	sdhci_reset(host, SDHCI_RESET_ALL);
	mmc->ops = &sdhci_ops;   // 设置mmc_host的操作集为sdhci_ops
	host->vmmc = regulator_get(mmc_dev(mmc), "vmmc");
	tasklet_init(&host->card_tasklet, sdhci_tasklet_card, (unsigned long)host); // host上发生card插入或者拔出时调用
	tasklet_init(&host->finish_tasklet, sdhci_tasklet_finish, (unsigned long)host); // 完成一个request时调用的tasklet
	ret = request_irq(host->irq, sdhci_irq, IRQF_SHARED,mmc_hostname(mmc), host);
	sdhci_init(host, 0);    // 软初始化host
	sdhci_enable_card_detection(host);    // 开始使能card插入状态的检测
}

















