/*����ƽ̨SD/MMC������Ҫ�������ļ�sdhci.c��sdhci-s3c.c,���������ں������档*/
//��Ҫ���sd��ƽ̨�豸��Ϣ
static void __init smdk4x12_machine_init(void)
{
	//...
	//��ʼ�����ƽ̨��Ϣ

	s3c_sdhci2_set_platdata(&smdk4x12_hsmmc2_pdata);
	s3c_sdhci3_set_platdata(&smdk4x12_hsmmc3_pdata);
	

	//ע��i2c��ƽ̨�豸��Ϣ

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
samsung sdhci-s3c.c����
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

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		ret = -ENOMEM;
		goto err_io_clk;
	}
	memcpy(pdata, pdev->dev.platform_data, sizeof(*pdata));

	/*���õ�platform_get_device_id(pdev)��ͨ��id_entry�е�driver_data�ж�ƥ��ĵ�����cbpmci_driver_ids�е���һ��ID��
�������������е�"cbp-sdmmc"��ƥ����ǵڶ��飬�����Ϳ�����Probe���жϵ�����ʲôƽ̨��*/
	drv_data = sdhci_s3c_get_driver_data(pdev);
	
	//sc = host->private��
	//host->private�Ѿ���sdhci_alloc_host����alloc�˶����sizeof(struct sdhci_s3c)��private��Ա 
	sc = sdhci_priv(host);

	sc->host = host;
	sc->pdev = pdev;
	sc->pdata = pdata;
	sc->ext_cd_gpio = pdata->ext_cd_gpio;

	/*ʹ��platform_set_drvdata�������䱣�浽platform_device�У�
����Ҫʹ�õ�ʱ����ʹ��platform_get_drvdata��������ȡ����*/
	platform_set_drvdata(pdev, host);

	sc->clk_io = clk_get(dev, "hsmmc");//��ȡʱ��Դ
	if (IS_ERR(sc->clk_io)) {
		dev_err(dev, "failed to get io clock\n");
		ret = PTR_ERR(sc->clk_io);
		goto err_io_clk;
	}

	/* enable the local io clock and keep it running for the moment. */
	clk_enable(sc->clk_io);//ʹ��

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

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);////�����豸��Դ
	/*ӳ��Ĵ�����ַ���ں������ַ�ռ䣬�����жϺ����ýӿ�*/
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

















