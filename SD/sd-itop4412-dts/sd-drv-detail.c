/*
Ӳ��ƽ̨��itop4412
ϵͳ��linux-4.14.2


*/
//��Ӧ���豸����Դ�ļ�: drivers/mmc/host/sdhci-s3c.c
//sdhci-s3c.cԴ�ļ����豸����ص���Ҫ����:
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
//ƥ���Ϻ� �豸������sdhci_s3c_probe�����ͻᱻ��������. ��probe������ͻ�ȡ���豸�����ṩ��Ӳ����Դ.
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

	//platform_get_irq��������platform_get_resource�� �᷵��һ��start, �����õ��жϺš�
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "no irq specified\n");
		return irq;
	}

	/* ʵ��mmc_host��sdhci_host�ķ��� */
	host = sdhci_alloc_host(dev, sizeof(struct sdhci_s3c));
	if (IS_ERR(host)) {
		dev_err(dev, "sdhci_alloc_host() failed\n");
		return PTR_ERR(host);
	}
	//sc = host->private��
//host->private�Ѿ���sdhci_alloc_host����alloc�˶����sizeof(struct sdhci_s3c)��private��Ա 
	sc = sdhci_priv(host);

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		ret = -ENOMEM;
		goto err_pdata_io_clk;
	}

	if (pdev->dev.of_node) {
		/*
		1.��ȡ�豸���np��������Ϊpropname������Ϊ8��16��32��64λ������������ԡ�
		����32λ��������������õ���of_property_read_u32_array()��
		2.ͨ���������豸�ڵ���������ֵõ�value��
		3.��of node�еõ�GPIO��number.
		*/
		ret = sdhci_s3c_parse_dt(&pdev->dev, host, pdata);
		if (ret)
			goto err_pdata_io_clk;
	} else {
		memcpy(pdata, pdev->dev.platform_data, sizeof(*pdata));
		sc->ext_cd_gpio = -1; /* invalid gpio number */
	}

	/*���õ�platform_get_device_id(pdev)��ͨ��id_entry�е�driver_data�ж�ƥ��ĵ�����cbpmci_driver_ids�е���һ��ID��
�������������е�"cbp-sdmmc"��ƥ����ǵڶ��飬�����Ϳ�����Probe���жϵ�����ʲôƽ̨��*/
	drv_data = sdhci_s3c_get_driver_data(pdev);//��ȡ�豸ID��ṹ�Լ�ƽ̨�豸��Դ

	sc->host = host;
	sc->pdev = pdev;
	sc->pdata = pdata;
	sc->cur_clk = -1;

	/*ʹ��platform_set_drvdata�������䱣�浽platform_device�У�
����Ҫʹ�õ�ʱ����ʹ��platform_get_drvdata��������ȡ����*/
	platform_set_drvdata(pdev, host);

	sc->clk_io = devm_clk_get(dev, "hsmmc");//��ȡʱ��Դ
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

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);//�����豸��Դ
	/*ӳ��Ĵ�����ַ���ں������ַ�ռ䣬�����жϺ����ýӿ�*/
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

	//ͨ��sdhci_add_host������ע��sdhci_host
	// sdhci_add_host������mmc_add_host������������豸��ӵ��ں���
	//��host�����mmc�����䱾�ʼ������host��class_dev
	ret = sdhci_add_host(host);
	/*
	�����������ط�������MMC��ϵͳ����ϸ���̣�sdhci_add_host(struct sdhci_host *host)
	�˺�������Ҫ����Ϊ��ת��sdhci_host�ṹ�嵽mmc_host�ں˱�׼�ṹ�壬���ҽ�����ر�׼����������MMC��ϵͳ��
	
	1. ����MMC�����ӿڣ�mmc->ops = &sdhci_ops; ��������Ҫ��⡿  
	2. �������tasklet��������ʱ����
	tasklet_init(&host->card_tasklet, sdhci_tasklet_card, (unsigned long)host);������MMC����ϵ�״̬�仯��
	tasklet_init(&host->finish_tasklet, sdhci_tasklet_finish, (unsigned long)host);�������������ɺ�Ĵ���
	setup_timer(&host->timer, sdhci_timeout_timer, (unsigned long)host);�����ڵȴ�Ӳ���жϡ�
	3. �ж�ӳ�䣺
	ret = request_irq(host->irq, sdhci_irq, IRQF_SHARED, mmc_hostname(mmc), host);���жϴ���
	4. ��ʼ��MMC�豸��������λ��������
	sdhci_init(host, 0);
	5. ����MMC��ϵͳ��
	mmc_add_host(mmc);
	6. ʹ���豸̽�⹦�ܣ�
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

/* ʵ��mmc_host��sdhci_host�ķ��� */
    mmc = mmc_alloc_host(sizeof(struct sdhci_host) + priv_size, dev);   // ����һ��struct mmc_host
    // ����mmc_host��ͬʱҲ������sizeof(struct sdhci_host) + priv_size��˽�����ݿռ䣬�ⲿ�־�����Ϊsdhci_host����˽������ʹ�õġ�
    // ����ο���mmc core����hostģ��˵����
    if (!mmc)
        return ERR_PTR(-ENOMEM);

/* ʵ��mmc_host��sdhci_host�Ĺ������� */
    host = mmc_priv(mmc);   // ��sdhci_host��Ϊmmc_host��˽�����ݣ�mmc_host->private = sdhci_host
    host->mmc = mmc;   // ����sdhci_host��mmc_host��sdhci_host->mmc = mmc_host

/* sdhci_host�����ĳ�ʼ������ */
    spin_lock_init(&host->lock);   // ��ʼ��sdhci_host ��ռ����
    mutex_init(&host->ios_mutex);   // ��ʼ��sdhci_host ����io setting�Ļ�����

    return host;   // ��struct sdhci_host ����
}
/*
���ϣ� 
mmc_host->private = sdhci_host 
sdhci_host->mmc = mmc_host
*/

int sdhci_add_host(struct sdhci_host *host)
{
	/*�ص�ļ�������*/
	sdhci_reset(host, SDHCI_RESET_ALL);
	mmc->ops = &sdhci_ops;   // ����mmc_host�Ĳ�����Ϊsdhci_ops
	host->vmmc = regulator_get(mmc_dev(mmc), "vmmc");
	tasklet_init(&host->card_tasklet, sdhci_tasklet_card, (unsigned long)host); // host�Ϸ���card������߰γ�ʱ����
	tasklet_init(&host->finish_tasklet, sdhci_tasklet_finish, (unsigned long)host); // ���һ��requestʱ���õ�tasklet
	ret = request_irq(host->irq, sdhci_irq, IRQF_SHARED,mmc_hostname(mmc), host);
	sdhci_init(host, 0);    // ���ʼ��host
	sdhci_enable_card_detection(host);    // ��ʼʹ��card����״̬�ļ��
}

//�豸������豸�ڵ�:
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


//����ϵͳ�󣬿��Բ鿴��:
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

















