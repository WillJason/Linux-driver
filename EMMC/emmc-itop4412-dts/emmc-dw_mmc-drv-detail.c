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


Ӳ��ƽ̨��itop4412
ϵͳ��linux-4.14.2


*/
//��Ӧ���豸����Դ�ļ�: drivers/mmc/host/Dw_mmc-exynos.c
//Dw_mmc-exynos.cԴ�ļ����豸����ص���Ҫ����:
static struct platform_driver dw_mci_exynos_pltfm_driver = {
	.probe		= dw_mci_exynos_probe,
	.remove		= dw_mci_exynos_remove,
	.driver		= {
		.name		= "dwmmc_exynos",
		.of_match_table	= dw_mci_exynos_match,
		.pm		= &dw_mci_exynos_pmops,
	},
};
static const struct of_device_id dw_mci_exynos_match[] = {
	{ .compatible = "samsung,exynos4412-dw-mshc",
			.data = &exynos_drv_data, },
	{},
};
MODULE_DEVICE_TABLE(of, dw_mci_exynos_match);
//ƥ���Ϻ� �豸������dw_mci_exynos_probe�����ͻᱻ��������. ��probe������ͻ�ȡ���豸�����ṩ��Ӳ����Դ.
static int dw_mci_exynos_probe(struct platform_device *pdev)
{
	const struct dw_mci_drv_data *drv_data;
	const struct of_device_id *match;
	int ret;

	match = of_match_node(dw_mci_exynos_match, pdev->dev.of_node);//��ȡ�豸ID��ṹ
	drv_data = match->data;

	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	ret = dw_mci_pltfm_register(pdev, drv_data);//��Ҫ�ĺ�������Dw_mmc-pltfm.c�ж��壬������ϸ����
	if (ret) {
		pm_runtime_disable(&pdev->dev);
		pm_runtime_set_suspended(&pdev->dev);
		pm_runtime_put_noidle(&pdev->dev);

		return ret;
	}

	return 0;
}

/*-----------------------------------------------------------------------------------
samsung Dw_mmc-pltfm.c����
-----------------------------------------------------------------------------------*/
int dw_mci_pltfm_register(struct platform_device *pdev,
			  const struct dw_mci_drv_data *drv_data)
{
	struct dw_mci *host;
	struct resource	*regs;

	//���ں�ģ������������ڴ�
	host = devm_kzalloc(&pdev->dev, sizeof(struct dw_mci), GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	 /* 
	 ͨ��ƽ̨�豸platform_device���IRQ 
 	 platform_get_irq��ʵ�ǵ���platform_get_resource(dev, IORESOURCE_IRQ, num) 
	 */
	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0)
		return host->irq;

	host->drv_data = drv_data;
	host->dev = &pdev->dev;
	host->irq_flags = 0;
	host->pdata = pdev->dev.platform_data;

	//�����豸��Դ
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(host->regs))
		return PTR_ERR(host->regs);

	/* Get registers' physical base address */
	host->phy_regs = regs->start;

	/*ʹ��platform_set_drvdata�������䱣�浽platform_device�У�
	����Ҫʹ�õ�ʱ����ʹ��platform_get_drvdata��������ȡ����*/
	platform_set_drvdata(pdev, host);
	return dw_mci_probe(host);//�ǳ���Ҫ�ĺ�����dw_mmc.c�ж��壬������ϸ����
}

/*-----------------------------------------------------------------------------------
samsung Dw_mmc.c����
-----------------------------------------------------------------------------------*/
int dw_mci_probe(struct dw_mci *host)
{
	const struct dw_mci_drv_data *drv_data = host->drv_data;
	int width, i, ret = 0;
	u32 fifo_size;

	if (!host->pdata) {
		host->pdata = dw_mci_parse_dt(host);//ʹ���豸��������ȡƽ̨��Դ
		if (PTR_ERR(host->pdata) == -EPROBE_DEFER) {
			return -EPROBE_DEFER;
		} else if (IS_ERR(host->pdata)) {
			dev_err(host->dev, "platform data not available\n");
			return -EINVAL;
		}
	}

	host->biu_clk = devm_clk_get(host->dev, "biu");
	if (IS_ERR(host->biu_clk)) {
		dev_dbg(host->dev, "biu clock not available\n");
	} else {
		ret = clk_prepare_enable(host->biu_clk);
		if (ret) {
			dev_err(host->dev, "failed to enable biu clock\n");
			return ret;
		}
	}

	host->ciu_clk = devm_clk_get(host->dev, "ciu");
	if (IS_ERR(host->ciu_clk)) {
		dev_dbg(host->dev, "ciu clock not available\n");
		host->bus_hz = host->pdata->bus_hz;
	} else {
		ret = clk_prepare_enable(host->ciu_clk);
		if (ret) {
			dev_err(host->dev, "failed to enable ciu clock\n");
			goto err_clk_biu;
		}

		if (host->pdata->bus_hz) {
			ret = clk_set_rate(host->ciu_clk, host->pdata->bus_hz);
			if (ret)
				dev_warn(host->dev,
					 "Unable to set bus rate to %uHz\n",
					 host->pdata->bus_hz);
		}
		host->bus_hz = clk_get_rate(host->ciu_clk);
	}

	if (!host->bus_hz) {
		dev_err(host->dev,
			"Platform data must supply bus speed\n");
		ret = -ENODEV;
		goto err_clk_ciu;
	}

	if (!IS_ERR(host->pdata->rstc)) {
		reset_control_assert(host->pdata->rstc);
		usleep_range(10, 50);
		reset_control_deassert(host->pdata->rstc);
	}

	if (drv_data && drv_data->init) {
		ret = drv_data->init(host);
		if (ret) {
			dev_err(host->dev,
				"implementation specific init failed\n");
			goto err_clk_ciu;
		}
	}

	setup_timer(&host->cmd11_timer,
		    dw_mci_cmd11_timer, (unsigned long)host);

	setup_timer(&host->cto_timer,
		    dw_mci_cto_timer, (unsigned long)host);

	setup_timer(&host->dto_timer,
		    dw_mci_dto_timer, (unsigned long)host);

	spin_lock_init(&host->lock);
	spin_lock_init(&host->irq_lock);
	INIT_LIST_HEAD(&host->queue);

	/*
	 * Get the host data width - this assumes that HCON has been set with
	 * the correct values.
	 ̽�⺯���и�ֵ���շ����������������������,�ܹ������֣�����16λ��32λ��64λ���շ����������ƣ�
	 ������16λΪ��������
	 */
	i = SDMMC_GET_HDATA_WIDTH(mci_readl(host, HCON));
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

	/* Reset all blocks ��λ����*/
	if (!dw_mci_ctrl_reset(host, SDMMC_CTRL_ALL_RESET_FLAGS)) {
		ret = -ENODEV;
		goto err_clk_ciu;
	}

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
	host->fifoth_val =
		SDMMC_SET_FIFOTH(0x2, fifo_size / 2 - 1, fifo_size / 2);
	mci_writel(host, FIFOTH, host->fifoth_val);

	/* disable clock to CIU */
	mci_writel(host, CLKENA, 0);
	mci_writel(host, CLKSRC, 0);

	/*
	 * In 2.40a spec, Data offset is changed.
	 * Need to check the version-id and set data-offset for DATA register.
	 */
	host->verid = SDMMC_GET_VERID(mci_readl(host, VERID));
	dev_info(host->dev, "Version ID is %04x\n", host->verid);

	if (host->data_addr_override)
		host->fifo_reg = host->regs + host->data_addr_override;
	else if (host->verid < DW_MMC_240A)
		host->fifo_reg = host->regs + DATA_OFFSET;
	else
		host->fifo_reg = host->regs + DATA_240A_OFFSET;

	/*
	 tasklet����һ���ں˶�ʱ��, ��һ��"���ж�"����������ִ��(��ԭ��ģʽ),����
	 ��Ӳ���жϴ����У�ʹ�ÿ���ʹ�ø��ӵ�����ȫ���Ӻ��Ժ��ʱ�䴦��task_init
	 ����һ��tasklet��Ȼ����ú��� tasklet_schedule�����tasklet���� tasklet_vec
	 �����ͷ���������Ѻ�̨�߳� ksoftirqd��
	*/
	tasklet_init(&host->tasklet, dw_mci_tasklet_func, (unsigned long)host);
	ret = devm_request_irq(host->dev, host->irq, dw_mci_interrupt,
			       host->irq_flags, "dw-mci", host);
	if (ret)
		goto err_dmaunmap;

	/*
	 * Enable interrupts for command done, data over, data empty,
	 * receive ready and error such as transmit, receive timeout, crc error
	 */
	mci_writel(host, INTMASK, SDMMC_INT_CMD_DONE | SDMMC_INT_DATA_OVER |
		   SDMMC_INT_TXDR | SDMMC_INT_RXDR |
		   DW_MCI_ERROR_FLAGS);
	/* Enable mci interrupt */
	mci_writel(host, CTRL, SDMMC_CTRL_INT_ENABLE);

	dev_info(host->dev,
		 "DW MMC controller at irq %d,%d bit host data width,%u deep fifo\n",
		 host->irq, width, fifo_size);

	/* We need at least one slot to succeed */
	/*��dw_mci_init_slot �о͵���mmc_alloc_host/mmc_add_host ��kernel ������mmc host��
	��mmc_alloc_host ������Ҫ�ľ���    INIT_DELAYED_WORK(&host->detect, mmc_rescan);������
	�Ϳ���ͨ��mmc_rescan ���������host�����ӵ��豸*/
	ret = dw_mci_init_slot(host);
	if (ret) {
		dev_dbg(host->dev, "slot %d init failed\n", i);
		goto err_dmaunmap;
	}

	/* Now that slots are all setup, we can enable card detect */
	dw_mci_enable_cd(host);

	return 0;

err_dmaunmap:
	if (host->use_dma && host->dma_ops->exit)
		host->dma_ops->exit(host);

	if (!IS_ERR(host->pdata->rstc))
		reset_control_assert(host->pdata->rstc);

err_clk_ciu:
	clk_disable_unprepare(host->ciu_clk);

err_clk_biu:
	clk_disable_unprepare(host->biu_clk);

	return ret;
}
//��ȡƽ̨��Ϣ����
static struct dw_mci_board *dw_mci_parse_dt(struct dw_mci *host)
{
	struct dw_mci_board *pdata;
	struct device *dev = host->dev;
	const struct dw_mci_drv_data *drv_data = host->drv_data;
	int ret;
	u32 clock_frequency;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	/* find reset controller when exist */
	pdata->rstc = devm_reset_control_get_optional_exclusive(dev, "reset");
	if (IS_ERR(pdata->rstc)) {
		if (PTR_ERR(pdata->rstc) == -EPROBE_DEFER)
			return ERR_PTR(-EPROBE_DEFER);
	}

	/* find out number of slots supported */
	if (!device_property_read_u32(dev, "num-slots", &pdata->num_slots))
		dev_info(dev, "'num-slots' was deprecated.\n");

	if (device_property_read_u32(dev, "fifo-depth", &pdata->fifo_depth))
		dev_info(dev,
			 "fifo-depth property not found, using value of FIFOTH register as default\n");

	device_property_read_u32(dev, "card-detect-delay",
				 &pdata->detect_delay_ms);

	device_property_read_u32(dev, "data-addr", &host->data_addr_override);

	if (device_property_present(dev, "fifo-watermark-aligned"))
		host->wm_aligned = true;

	if (!device_property_read_u32(dev, "clock-frequency", &clock_frequency))
		pdata->bus_hz = clock_frequency;

	if (drv_data && drv_data->parse_dt) {
		ret = drv_data->parse_dt(host);
		if (ret)
			return ERR_PTR(ret);
	}

	return pdata;
}
/*
��Ҫ����Ѿ��������漰����ϸ�����Ѿ���
emmc-tiny4412-pri/emmc-dw_mmc-drv-detail.c�ļ�����ϸ�����ˣ����ٽ���-*2018-08-27-ShenYang-Fine Day
*/


//�豸������豸�ڵ�:
				mmc@12550000 {
                num-slots = <1>;
                broken-cd;
                non-removable;
                card-detect-delay = <200>;
                clock-frequency = <400000000>;
                samsung,dw-mshc-ciu-div = <0>;
                samsung,dw-mshc-sdr-timing = <2 3>;
                samsung,dw-mshc-ddr-timing = <1 2>;
                pinctrl-0 = <&sd4_clk &sd4_cmd &sd4_bus4 &sd4_bus8>;
                pinctrl-names = "default";
                status = "okay";
                bus-width = <8>;
                cap-mmc-highspeed;
        };

//����ϵͳ�󣬿��Բ鿴��:

...................

[    1.925575] mmc_host mmc1: Bus speed (slot 0) = 50000000Hz (slot req 52000000Hz, actual 50000000HZ div = 0)
[    1.950983] mmc1: new DDR MMC card at address 0001
[    1.960543] mmcblk1: mmc1:0001 4YMD3R 3.64 GiB 
[    1.970526] mmcblk1boot0: mmc1:0001 4YMD3R partition 1 4.00 MiB
[    1.980586] mmcblk1boot1: mmc1:0001 4YMD3R partition 2 4.00 MiB
[    1.986114] mmcblk1rpmb: mmc1:0001 4YMD3R partition 3 512 KiB
[    1.999034]  mmcblk1: p1 p2 p3 p4
[    2.004630] mmc0: Problem switching card into high-speed mode!
[    2.011718] mmc0: new SDHC card at address 21e7
[    2.019858] input: gpio-keys as /devices/platform/gpio-keys/input/input0
[    2.025876] mmcblk0: mmc0:21e7 APPSD 15.2 GiB 
[    2.030267]  mmcblk0: p1 p2 p3 p4

...................

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

Partition table entries are not in disk order


























