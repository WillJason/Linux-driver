/*
MMC��ϵͳ����

MMC����ֲ�

MMC��ϵͳ������Ҫ��drivers/mmcĿ¼�£���������Ŀ¼��

         Card��������濨(���豸)�������������MMC/SD���豸������SDIOUART��

         Host����Բ�ͬ�����˵�SDHC��MMC���������������ⲿ����Ҫ����������ʦ����ɣ�

         Core������MMC�ĺ��Ĳ㣬�ⲿ����ɲ�ͬЭ��͹淶��ʵ�֣�Ϊhost����豸�������ṩ�ӿں����� 

Linux MMC��ϵͳ��Ҫ�ֳ��������֣�

MMC���Ĳ㣺��ɲ�ͬЭ��͹淶��ʵ�֣�Ϊhost����豸�������ṩ�ӿں�����MMC���Ĳ�������������ɣ�MMC��SD��SDIO���ֱ�Ϊ�����豸�����ṩ�ӿں�����

Host �����㣺��Բ�ͬ�����˵�SDHC��MMC��������������

Client �����㣺��Բ�ͬ�ͻ��˵��豸����������SD����T-flash����SDIO�ӿڵ�GPS��wi-fi���豸������

*/
//��Ҫ���sd��ƽ̨�豸��Ϣ
static void __init smdk4x12_machine_init(void)
{
	//...
	//��ʼ�����ƽ̨��Ϣ

	#ifdef CONFIG_EXYNOS4_DEV_DWMCI
	exynos_dwmci_set_platdata(&exynos_dwmci_pdata);
#endif

	//ע��i2c��ƽ̨�豸��Ϣ

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
samsung Dw_mmc-pltfm.c����
-----------------------------------------------------------------------------------*/
static int dw_mci_pltfm_probe(struct platform_device *pdev)
{
	struct dw_mci *host;
	struct resource	*regs;
	int ret;

	host = kzalloc(sizeof(struct dw_mci), GFP_KERNEL);//���ں�ģ������������ڴ�
	if (!host)
		return -ENOMEM;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);//�����豸��Դ
	if (!regs) {
		ret = -ENXIO;
		goto err_free;
	}

	 /* 
	 ͨ��ƽ̨�豸platform_device���IRQ 
 	 platform_get_irq��ʵ�ǵ���platform_get_resource(dev, IORESOURCE_IRQ, num) 
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
	/*ʹ��platform_set_drvdata�������䱣�浽platform_device�У�
	����Ҫʹ�õ�ʱ����ʹ��platform_get_drvdata��������ȡ����*/
	platform_set_drvdata(pdev, host);
	ret = dw_mci_probe(host);//�ǳ���Ҫ�ĺ�����dw_mmc.c�ж��壬������ϸ����
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
samsung Dw_mmc.c����
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
	 ̽�⺯���и�ֵ���շ����������������������,�ܹ������֣�����16λ��32λ��64λ���շ����������ƣ�������16λΪ��������
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

	/* Reset all blocks SD��λ����*/
	if (!mci_wait_reset(&host->dev, host))
		return -ENODEV;

	/*DW��SD�������й��ڲ���DAM����
	����������̽�⺯�������豸������Ҳû�ж�pdata->dma_ops��Ա���г�ʼ��*/
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
	 tasklet����һ���ں˶�ʱ��, ��һ��"���ж�"����������ִ��(��ԭ��ģʽ),����
	 ��Ӳ���жϴ����У�ʹ�ÿ���ʹ�ø��ӵ�����ȫ���Ӻ��Ժ��ʱ�䴦��task_init
	 ����һ��tasklet��Ȼ����ú��� tasklet_schedule�����tasklet���� tasklet_vec
	 �����ͷ���������Ѻ�̨�߳� ksoftirqd��
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
		/*��dw_mci_init_slot �о͵���mmc_alloc_host/mmc_add_host ��kernel ������mmc host��
	��mmc_alloc_host ������Ҫ�ľ���    INIT_DELAYED_WORK(&host->detect, mmc_rescan);������
	�Ϳ���ͨ��mmc_rescan ���������host�����ӵ��豸*/
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

/*������������̽�⺯���и�ֵ���շ���������������������ͣ��ܹ������֣�����16λ��32λ��64λ���շ����������ƣ�������16λΪ��������
host->push_data = dw_mci_push_data16;
host->pull_data = dw_mci_pull_data16;
��������*/
static void dw_mci_push_data16(struct dw_mci *host, void *buf, int cnt)
{
    u16 *pdata = (u16 *)buf;
    WARN_ON(cnt % 2 != 0);
    cnt = cnt >> 1;    //���յ��ĳ��������ֽ�Ϊ��λ�ģ�����16λ�ĳ���2,�����32λ�ľ�������λ������4,�Դ�����
    while (cnt > 0) {
        mci_writew(host, DATA, *pdata++);
        cnt--;
    }
}
//�պ�����
static void dw_mci_pull_data16(struct dw_mci *host, void *buf, int cnt)
{
    u16 *pdata = (u16 *)buf;
    WARN_ON(cnt % 2 != 0);
    cnt = cnt >> 1;    //���յ��ĳ��������ֽ�Ϊ��λ�ģ�����16λ�ĳ���2,�����32λ�ľ�������λ������4,�Դ�����
    while (cnt > 0) {
        *pdata++ = mci_readw(host, DATA);
        cnt--;
    }
}

//������SD��λ����mci_wait_reset��
static bool mci_wait_reset(struct device *dev, struct dw_mci *host)
{
    unsigned long timeout = jiffies + msecs_to_jiffies(500);
    unsigned int ctrl;
    mci_writel(host, CTRL, (SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET |
                SDMMC_CTRL_DMA_RESET));    //���ÿ��ƼĴ���
    /* wait till resets clear */
    do {
        ctrl = mci_readl(host, CTRL);
        if (!(ctrl & (SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET |
                  SDMMC_CTRL_DMA_RESET)))
            return true;
    } while (time_before(jiffies, timeout));    //�ȴ���ʱʱ����
    dev_err(dev, "Timeout resetting block (ctrl %#x)\n", ctrl);
    return false;
}

static void dw_mci_init_dma(struct dw_mci *host)
{
	/* Alloc memory for sg translation */
	host->sg_cpu = dma_alloc_coherent(&host->dev, PAGE_SIZE,
					  &host->sg_dma, GFP_KERNEL);//DMA�ڴ�����
	if (!host->sg_cpu) {
		dev_err(&host->dev, "%s: could not alloc DMA memory\n",
			__func__);
		goto no_dma;
	}

	/* Determine which DMA interface to use */
#ifdef CONFIG_MMC_DW_IDMAC
	host->dma_ops = &dw_mci_idmac_ops;//DMA������������
	dev_info(&host->dev, "Using internal DMA controller.\n");
#endif
		//......
}

static int __init dw_mci_init_slot(struct dw_mci *host, unsigned int id)
{
	struct mmc_host *mmc;
	struct dw_mci_slot *slot;

	mmc = mmc_alloc_host(sizeof(struct dw_mci_slot), &host->dev);//core������host
	if (!mmc)
		return -ENOMEM;

	slot = mmc_priv(mmc);// ��dw_mci_slot��Ϊmmc_host��˽�����ݣ�mmc_host->private = dw_mci_slot
	slot->id = id;
	slot->mmc = mmc;
	slot->host = host;

	mmc->ops = &dw_mci_ops;//mmc����������
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
	mmc_add_host(mmc);//��ʼ��host
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


//mmc_rescan �����Ҫ��� sd ������Ҫ��� mmc ���ģ����Ǿ�����һ�������ߣ��ٶ��и��˲����� MMC �����Ǿ�Ӧ���������⼸�У�

       err = mmc_send_op_cond(host, 0, &ocr);
       if (!err) {
              if (mmc_attach_mmc(host, ocr))
                     mmc_power_off(host);
              goto out;
       }

//mmc_send_op_cond ���������˵�Ƕ���һ�¿���ʲôֵ�����ֵ��ʲô������Ҳ�������������� FLASH ʱ�� FLASH �� ID һ��������Ҳ�������ģ����ù����ֵ�������ˣ�ֻҪ֪�����ܱ�ʶ��һ�� MMC ����������ˡ����ȡ���ֵû�д���Ļ��͵ý� mmc_attach_mmc �ˣ�

	/*
  * Starting point for MMC card init.
  */

int mmc_attach_mmc(struct mmc_host *host, u32 ocr)  // core/mmc.c

{
       int err;
����
       mmc_attach_bus_ops(host);         // ��������ߵĵ�Դ�����йأ���ʱ����
����
       /*

         * Detect and init the card.

         */
       err = mmc_init_card(host, host->ocr, NULL);
       if (err)
              goto err;
����
       mmc_release_host(host);
       err = mmc_add_card(host->card);
       if (err)
              goto remove_card;
       return 0;
remove_card:
����
err:
����
       return err;

}
//�����Ҽ����ؼ��������� mmc_init_card �Ӻ������������ǳ�ʼ��һ�� card ����� card 
//���� struct mmc_card �ṹ��������Ȼ���ֵ��� mmc_add_card �����豸��ӵ����ںˣ������� mmc_init_card ������Щʲô���飺
static int mmc_init_card(struct mmc_host *host, u32 ocr,
       struct mmc_card *oldcard)
{
       struct mmc_card *card;
       int err;
       u32 cid[4];
       unsigned int max_dtr;

����
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
����
              host->card = card;
       return 0;
free_card:
����
err:
����
    return err;

}
//����Ӳ��������ص�ȫ��ɾ���������������õ�Ҳ���⼸���� mmc_alloc_card ������һ�� struct mmc_card �ṹ��
//Ȼ��� card->type ���� MMC_TYPE_MMC ����� card �ָ����� host->card ����;���Ӳ������ͦ��ģ���Ϊһ����������һ��Ͳ�һ�������п�ʱ host->card ��ֵ��û�п�ʱ host->card �Լ����� NULL �ˡ�
//       ��� mmc_alloc_card ������������
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

/*Struct mmc_card �ṹ���������һ�� struct device �ṹ�� mmc_alloc_card �����������ڴ棬����
	 ������� struct device �еļ�����Ա������ card->dev.bus = &mmc_bus_type; ��һ��Ҫ�ص�Դ���
   ����һ�� mmc_card �ṹ�����򵥳�ʼ���� mmc_init_card ��ʹ��������ˣ�Ȼ���ٵ���
   mmc_add_card ����� card �豸��ӵ��ںˡ� mmc_add_card ��ʵ�ܼ򵥣����ǵ��� device_add ��
    card->dev ��ӵ��ں˵���ȥ��
   ֪������ģ������������˶����ף��� device_add �������߾�Ӧ���ж����ˣ��������ĸ�
   �����أ��Ǿ͵ÿ������ device_add ʱ�͵��Ǹ� dev ����ָ�������ĸ������ˣ������͵� 
   card->dev ����ô card->dev.bus ����ָ��ʲô�أ����������Ǹ� mmc_bus_type ��
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

//�� device_add ���棬�豸��Ӧ�����߻�����������豸�͹�����������ϵ�������������ȥ
//ƥ�䣨 match ������ʱ����� match ���������ƥ�䵽�˾ͻ�������ߵ� probe ����������
//�� probe �����������ǿ�һ������� mmc_bus_match ����ν���ƥ��ģ�
static int mmc_bus_match(struct device *dev, struct device_driver *drv)
{
       return 1;
}

//���� match ��Զ���ܳɹ����Ǿ�ȥִ�� probe �ɣ�

static int mmc_bus_probe(struct device *dev)
{
       struct mmc_driver *drv = to_mmc_driver(dev->driver);
       struct mmc_card *card = dev_to_mmc_card(dev);
       return drv->probe(card);
}

/*������е��鷳�ˣ���������������ֵ�����һ�� drv->probe() ������� drv ��ʲô�أ�
�����У� struct mmc_driver *drv = to_mmc_driver(dev->driver);

match �������Ƿ��� 1 ���ǿ���ֻҪ�ǹ������������ϵ� driver ���п����ܵ��������ˣ�
��ʵ��ȷҲ�������ģ��������ڹ������������ϵ� driver ֻ��һ����������������ģ�
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

//��������ʱ�� card/core/host �����Ѿ�ȫ�����������ˣ��߿� mmc_driver �еļ������������Ǽ��������ϵ����Ҳ�����������ˡ������Ǽ����ɡ�
/*�����׶Σ�

ǰ���Ѿ������ˣ������ߵ� probe ��������� drv->probe, ����������Ͷ�Ӧ���� mmc_blk_probe ��
������� mmc_driver ����ô�ҵ� mmc_bus �ϵģ��Լ�ȥ�� mmc_blk_init() ���ͼ��д��룬Ӧ�ò��ѡ�
*/
static int mmc_blk_probe(struct mmc_card *card) // ���� card/block.c
{
       struct mmc_blk_data *md;
       int err;
����
       md = mmc_blk_alloc(card);
       if (IS_ERR(md))
              return PTR_ERR(md);
����
       add_disk(md->disk);
       return 0;
  out:
     mmc_blk_put(md);
       return err;

}

//���Ǽ���Ҫ�ĺ�������һ������������������� add_disk ����Ӧ�ÿ����뵽Щʲô�ɣ�����㲻֪������˵Щʲô�����ҹ�����û�п��� LDD3 �����߿���Ҳ������ۻ��ˡ����������㣺������� add_disk ����˵��ǰ��һ������ alloc_disk �ͳ�ʼ�����еĶ������� mmc_blk_probe ʱ��û�����ֳ������ǾͿ� mmc_blk_alloc(card) ��һ�У�
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

/*������������Ĵ��룬������Ȼ�ͻ������˿��豸������������·�ˣ�
1.       ���䡢��ʼ��������У�����������к���������
2.       ���䣬��ʼ�� gendisk ���� gendisk �� major �� fops �� queue �ȳ�Ա��ֵ�������� gendisk ��
3.       ע����豸������
���ǿ��� MMC ������������û�а������·�ߣ�
1 �� mmc_init_queue ��ʼ�˶��У����� mmc_blk_issue_rq; �����󶨳���������
2 �� alloc_disk ������ gendisk �ṹ������ʼ���� major �� fops ���� queue ��
3 �������� add_disk �����豸�ӵ� KERNEL ��ȥ��
��������Ȼ mmc_blk_probe �Ѿ������ˣ������Ǳ�ͣ�������ǵ� LDD3 ���ڽ� sbull ʵ��ʱ˵���� 
add_disk �ĵ��ñ�־��һ�����豸�������������������֮ǰ�������������׼������ȫ�����ã�
����Ϊʲô������˵�������ɵģ���Ϊ�� add_disk ���� kernel ��ȥ������󶨵������е���������
Ŀ����ȥ��Ŀ��豸�϶��������������� add_disk �ڲ���Ҫ���ģ������� add_disk ���غ�������
����Ϊʲô��������ȥ�� add_disk �Ĵ���ʵ�־�֪���ˡ�
��ȻҪ����������ȥ���������Ǿ��������������� mmc_blk_issue_rq*/
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

/*�������ʵ��̫���ˣ��������ǲ���ȫ�������󲿷ֶ����ݵ�׼������ͳ�����Ĵ����Ѿ�����ɾ���ˣ�ֻҪ֪�������ݶ�����������ɵľ͹��ˡ���������������ģ����� LDD3 �Ҹ����ٵĵط����� sbull �о�͸��Ҳ��������������ˡ���������������漰�Ķ�����ͦ���٣���ɢ�б������ص���������������ˣ���ʱ������ȥ�о��ɡ�

       �ڿ��豸����������ֻ��Ҫץס������к��������Ϳ����ˣ�������Щ block_device_operations ���渳ֵ�ĺ����ɲ����ַ��豸����������ô�ܹ�ע�ˡ�

 

       ��������� MMC/SD ���������������ܻ���Ҳ�ͺ������ˣ�˵���˾������������£�

1.       ���ļ�⣻

2.       �����ݵĶ�ȡ��

����ٽ����������̴�Ŵ�һ�£�

1.       ���ļ�⣺

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

2.       ��д���ݣ�

mmc_blk_issue_rq �� card/block.c ��

       mmc_wait_for_req(core/core.c)

              mmc_start_request(core/core.c)

                     host->ops->request(host, mrq)   // s3cmci �� s3cmci_request

 

MMC/SD ���������������ˣ��ǲ�����Щ���ӣ�����������Ƶ�Ŀ����Ϊ�˷ֲ㣬�þ���ƽ̨��������д����ʡ�¡�
*/










