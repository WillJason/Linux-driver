/*
硬件平台：itop4412
系统：linux-4.14.2


*/
//对应的设备驱动源文件: drivers/mmc/host/sdhci-s3c.c
//sdhci-s3c.c源文件里设备树相关的主要内容:
static struct platform_driver sdhci_s3c_driver = {
	.probe		= sdhci_s3c_probe,
	.remove		= sdhci_s3c_remove,
	.id_table	= sdhci_s3c_driver_ids,
	.driver		= {
		.name	= "s3c-sdhci",
		.of_match_table = of_match_ptr(sdhci_s3c_dt_match),
		.pm	= &sdhci_s3c_pmops,
	},
};
static const struct of_device_id sdhci_s3c_dt_match[] = {
	{ .compatible = "samsung,s3c6410-sdhci", },
	{ .compatible = "samsung,exynos4210-sdhci",
		.data = (void *)EXYNOS4_SDHCI_DRV_DATA },
	{},
};
MODULE_DEVICE_TABLE(of, sdhci_s3c_dt_match);
//匹配上后， 设备驱动里sdhci_s3c_probe函数就会被触发调用. 在probe函数里就会取出设备树里提供的硬件资源.
static int sdhci_s3c_probe(struct platform_device *pdev)
{
	struct s3c_sdhci_platdata *pdata;
	struct sdhci_s3c_drv_data *drv_data;
	struct device *dev = &pdev->dev;
	struct sdhci_host *host;
	struct sdhci_s3c *sc;
	struct resource *res;
	int ret, irq, ptr, clks;

	if (!pdev->dev.platform_data && !pdev->dev.of_node) {
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
	//sc = host->private；
//host->private已经被sdhci_alloc_host函数alloc了额外的sizeof(struct sdhci_s3c)给private成员 
	sc = sdhci_priv(host);

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		ret = -ENOMEM;
		goto err_pdata_io_clk;
	}

	if (pdev->dev.of_node) {
		/*
		1.读取设备结点np的属性名为propname，类型为8、16、32、64位整型数组的属性。
		对于32位处理器来讲，最常用的是of_property_read_u32_array()。
		2.通过给定的设备节点和属性名字得到value。
		3.从of node中得到GPIO的number.
		*/
		ret = sdhci_s3c_parse_dt(&pdev->dev, host, pdata);
		if (ret)
			goto err_pdata_io_clk;
	} else {
		memcpy(pdata, pdev->dev.platform_data, sizeof(*pdata));
		sc->ext_cd_gpio = -1; /* invalid gpio number */
	}

	/*调用到platform_get_device_id(pdev)，通过id_entry中的driver_data判断匹配的到底是cbpmci_driver_ids中的哪一组ID，
比如上面例子中的"cbp-sdmmc"中匹配的是第二组，这样就可以再Probe的判断到底是什么平台了*/
	drv_data = sdhci_s3c_get_driver_data(pdev);//获取设备ID表结构以及平台设备资源

	sc->host = host;
	sc->pdev = pdev;
	sc->pdata = pdata;
	sc->cur_clk = -1;

	/*使用platform_set_drvdata（）将其保存到platform_device中，
在需要使用的时候再使用platform_get_drvdata（）来获取它。*/
	platform_set_drvdata(pdev, host);

	sc->clk_io = devm_clk_get(dev, "hsmmc");//获取时钟源
	if (IS_ERR(sc->clk_io)) {
		dev_err(dev, "failed to get io clock\n");
		ret = PTR_ERR(sc->clk_io);
		goto err_pdata_io_clk;
	}

	/* enable the local io clock and keep it running for the moment. */
	clk_prepare_enable(sc->clk_io);

	for (clks = 0, ptr = 0; ptr < MAX_BUS_CLK; ptr++) {
		char name[14];

		snprintf(name, 14, "mmc_busclk.%d", ptr);
		sc->clk_bus[ptr] = devm_clk_get(dev, name);
		if (IS_ERR(sc->clk_bus[ptr]))
			continue;

		clks++;
		sc->clk_rates[ptr] = clk_get_rate(sc->clk_bus[ptr]);

		dev_info(dev, "clock source %d: %s (%ld Hz)\n",
				ptr, name, sc->clk_rates[ptr]);
	}

	if (clks == 0) {
		dev_err(dev, "failed to find any bus clocks\n");
		ret = -ENOENT;
		goto err_no_busclks;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);//申请设备资源
	/*映射寄存器地址到内核虚拟地址空间，申请中断和配置接口*/
	host->ioaddr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(host->ioaddr)) {
		ret = PTR_ERR(host->ioaddr);
		goto err_req_regs;
	}

	/* Ensure we have minimal gpio selected CMD/CLK/Detect */
	if (pdata->cfg_gpio)
		pdata->cfg_gpio(pdev, pdata->max_width);

	host->hw_name = "samsung-hsmmc";
	host->ops = &sdhci_s3c_ops;
	host->quirks = 0;
	host->quirks2 = 0;
	host->irq = irq;

	/* Setup quirks for the controller */
	host->quirks |= SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC;
	host->quirks |= SDHCI_QUIRK_NO_HISPD_BIT;
	if (drv_data) {
		host->quirks |= drv_data->sdhci_quirks;
		sc->no_divider = drv_data->no_divider;
	}

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
	if (sc->no_divider) {
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

	ret = mmc_of_parse(host->mmc);
	if (ret)
		goto err_req_regs;

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
		goto err_req_regs;
	}

#ifdef CONFIG_PM
	if (pdata->cd_type != S3C_SDHCI_CD_INTERNAL)
		clk_disable_unprepare(sc->clk_io);
#endif
	return 0;

 err_req_regs:
	pm_runtime_disable(&pdev->dev);

 err_no_busclks:
	clk_disable_unprepare(sc->clk_io);

 err_pdata_io_clk:
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

//设备树里的设备节点:
sdhci_2: sdhci@12530000 {
		compatible = "samsung,exynos4210-sdhci";
		reg = <0x12530000 0x100>;
		interrupts = <GIC_SPI 75 IRQ_TYPE_LEVEL_HIGH>;
		clocks = <&clock CLK_SDMMC2>, <&clock CLK_SCLK_MMC2>;
		clock-names = "hsmmc", "mmc_busclk.2";
		status = "disabled";
	};
&sdhci_2 {
	bus-width = <4>;
	pinctrl-0 = <&sd2_clk &sd2_cmd &sd2_bus4>;
	pinctrl-names = "default";
	cd-gpio = <&gpx0 7 GPIO_ACTIVE_LOW>;
	cap-sd-highspeed;
	/*vmmc-supply = <&ldo23_reg>;
	vqmmc-supply = <&ldo17_reg>;*/
	status = "okay";
};


//重启系统后，可以查看到:
~ # fdisk -l                                                                    

Disk /dev/mmcblk1: 3909 MB, 3909091328 bytes
226 heads, 33 sectors/track, 1023 cylinders
Units = cylinders of 7458 * 512 = 3818496 bytes

        Device Boot      Start         End      Blocks  Id System
/dev/mmcblk1p1             653        1020     1372272   c Win95 FAT32 (LBA)
/dev/mmcblk1p2               6         287     1051578  83 Linux
/dev/mmcblk1p3             288         569     1051578  83 Linux
/dev/mmcblk1p4             570         652      309507  83 Linux

Partition table entries are not in disk order

Disk /dev/mmcblk1boot1: 4 MB, 4194304 bytes
4 heads, 16 sectors/track, 128 cylinders
Units = cylinders of 64 * 512 = 32768 bytes

Disk /dev/mmcblk1boot1 doesn't contain a valid partition table

Disk /dev/mmcblk1boot0: 4 MB, 4194304 bytes
4 heads, 16 sectors/track, 128 cylinders
Units = cylinders of 64 * 512 = 32768 bytes

Disk /dev/mmcblk1boot0 doesn't contain a valid partition table

Disk /dev/mmcblk0: 16.3 GB, 16358834176 bytes
255 heads, 63 sectors/track, 1988 cylinders
Units = cylinders of 16065 * 512 = 8225280 bytes

        Device Boot      Start         End      Blocks  Id System
/dev/mmcblk0p1             210        1988    14285824   c Win95 FAT32 (LBA)
/dev/mmcblk0p2               3         133     1048576  83 Linux
/dev/mmcblk0p3             133         171      307200  83 Linux
/dev/mmcblk0p4             171         210      307200  83 Linux

















