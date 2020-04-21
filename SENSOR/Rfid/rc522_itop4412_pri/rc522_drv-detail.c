/*
Ӳ��ƽ̨��itop4412
ϵͳ��linux-3.0.5
	MF RC522 �������Ƚ��ĵ��ƺͽ�������ȫ��������13.56MHz ���������͵ı�
���ǽӴ�ʽͨ�ŷ�ʽ��Э�顣֧�� ISO14443A �Ķ��Ӧ�á����ڲ����������ֿ���
����д��������ISO 14443A/MIFARE����Ӧ�����ͨ�ţ����������ĵ�·����������
���ṩһ����̶���Ч�Ľ���ͽ����·�����ڴ���ISO14443A ���ݵ�Ӧ�����źš�
���ֲ��ִ���ISO14443A ֡�ʹ����⣨��ż &CRC�������⣬����֧�ֿ���CRYPTO1 
�����㷨��������֤MIFARE ϵ�в�Ʒ��MFRC522 ֧��MIFARE?�����ٵķǽӴ�ʽͨ�ţ�
˫�����ݴ������ʸߴ�424kbit/s��

SPI��linux�з�Ϊ3��ֱ�Ϊ�������������������Ĳ�����������������
����������弶�����ܹ��ǣ�
			����������������spi_s3c64xx.c--------------�����Ҫ����SPI������ص���������������
     
     	���Ĳ����� ��spi.c---------------------------����ļ���ʵ����spi��ע���ע���ĺ����ȣ�����
     		�������������������������������������á�
     
      ��������  ��rc522.c----------------------�ں��е�һ���ǽӴ�ʽ��д��оƬ�����������
      
 	spi_s3c24xx.c����û�������豸�ڵ㣬rc522.c���������Ϊ����user space��kernel space����spi��
�����裬����ͨ����������spi�豸�Ķ�д��

rc522����λ��drivers/spiĿ¼�������ļ�Ϊdrivers/spi/rc522.c��
*/
//������һ��SPI���ߵ�������������ص�ƽ̨�豸ע�����Ϣ
////arch/arm/mach-exynos/dev-spi.c
static struct resource exynos_spi2_resource[] = {
	[0] = {
		.start = EXYNOS_PA_SPI2,
		.end   = EXYNOS_PA_SPI2 + 0x100 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = DMACH_SPI2_TX,
		.end   = DMACH_SPI2_TX,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.start = DMACH_SPI2_RX,
		.end   = DMACH_SPI2_RX,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.start = IRQ_SPI2,
		.end   = IRQ_SPI2,
		.flags = IORESOURCE_IRQ,
	},
};
static struct s3c64xx_spi_info exynos_spi2_pdata = {
	.cfg_gpio = exynos_spi_cfg_gpio,
	.fifo_lvl_mask = 0x7f,
	.rx_lvl_offset = 15,
	.high_speed = 1,
	.clk_from_cmu = true,
	.tx_st_done = 25,
};
struct platform_device exynos_device_spi2 = {
	.name		  = "s3c64xx-spi",
	.id		  = 2,
	.num_resources	  = ARRAY_SIZE(exynos_spi2_resource),
	.resource	  = exynos_spi2_resource,
	.dev = {
		.dma_mask		= &spi_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data = &exynos_spi2_pdata,
	},
};
//���������Ҫ����spi�ӿ�Ӳ��������I2C��s3c_i2c1_set_platdata��ͬ������ں��������
void __init exynos_spi_set_info(int cntrlr, int src_clk_nr, int num_cs)
{
	struct s3c64xx_spi_info *pd;

	/* Reject invalid configuration */
	if (!num_cs || src_clk_nr < 0
			|| src_clk_nr > EXYNOS_SPI_SRCCLK_SCLK) {
		printk(KERN_ERR "%s: Invalid SPI configuration\n", __func__);
		return;
	}
	switch (cntrlr) {
	case 0:
		pd = &exynos_spi0_pdata;
		break;
	case 1:
		pd = &exynos_spi1_pdata;
		break;
	case 2:
		pd = &exynos_spi2_pdata;
		break;
	default:
		printk(KERN_ERR "%s: Invalid SPI controller(%d)\n",
							__func__, cntrlr);
		return;
	}
	pd->num_cs = num_cs;
	pd->src_clk_nr = src_clk_nr;
	pd->src_clk_name = spi_src_clks[src_clk_nr];
}
//��Ϣ��ӵ���ں���
static struct platform_device *smdk4x12_devices[] __initdata = {
{
		#ifdef CONFIG_S3C64XX_DEV_SPI
			&exynos_device_spi2,
		#endif
}
static void __init smdk4x12_machine_init(void)
{
		#ifdef CONFIG_S3C64XX_DEV_SPI
			unsigned int gpio;
			struct clk *sclk = NULL;
			struct clk *prnt = NULL;
			struct device *spi2_dev = &exynos_device_spi2.dev;
		#endif
		
		platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
		
		sclk = clk_get(spi2_dev, "dout_spi2");
		if (IS_ERR(sclk))
			dev_err(spi2_dev, "failed to get sclk for SPI-2\n");
		prnt = clk_get(spi2_dev, "mout_mpll_user");
		if (IS_ERR(prnt))
			dev_err(spi2_dev, "failed to get prnt\n");
		if (clk_set_parent(sclk, prnt))
			printk(KERN_ERR "Unable to set parent %s of clock %s.\n",
					prnt->name, sclk->name);
	
		clk_set_rate(sclk, 800 * 1000 * 1000);
		clk_put(sclk);
		clk_put(prnt);
	
		if (!gpio_request(EXYNOS4_GPC1(2), "SPI_CS2")) {
			gpio_direction_output(EXYNOS4_GPC1(2), 1);
			s3c_gpio_cfgpin(EXYNOS4_GPC1(2), S3C_GPIO_SFN(1));
			s3c_gpio_setpull(EXYNOS4_GPC1(2), S3C_GPIO_PULL_UP);
			exynos_spi_set_info(2, EXYNOS_SPI_SRCCLK_SCLK,
				ARRAY_SIZE(spi2_csi));
		}
		for (gpio = EXYNOS4_GPC1(1); gpio < EXYNOS4_GPC1(5); gpio++)
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV3);
}
//������һ��rc522����ע�����Ϣ
//�����ں����rc522������������ѡ��:
Symbol: SPI_RC522 [=y]                                              
  Type  : tristate                                               
  Prompt: RC522 Module driver support                            
	  Defined at drivers/spi/Kconfig:470                            
	  Depends on: SPI [=y] && SPI_MASTER [=y] && EXPERIMENTAL [=y]             
	  Location:                                                                                                                           
		  -> Device Drivers                                                                                                                     
		  -> SPI support (SPI [=y])   
//ע����Ϣ����
static struct s3c64xx_spi_csinfo spi2_csi[] = {
	[0] = {
		.line = EXYNOS4_GPC1(2),
		.set_level = gpio_set_value,
		.fb_delay = 0x2,
	},
};
static struct spi_board_info spi2_board_info[] __initdata = {
#ifdef CONFIG_SPI_RC522
        {
                .modalias = "rc522",
                .platform_data = NULL,
                .max_speed_hz = 10*1000*1000,
                .bus_num = 2,
                .chip_select = 0,
                .mode = SPI_MODE_0,
                .controller_data = &spi2_csi[0],
        }
#endif
};
//��Ϣͬ����ӵ���ں���
static void __init smdk4x12_machine_init(void)
{
	spi_register_board_info(spi2_board_info, ARRAY_SIZE(spi2_board_info));
}

//�豸��Ϣע�������ϣ���������������
//��������������
/*file:spi_s3c64xx.c*/
//�ڱ��Ŀ¼���Ѿ��з��������ﲻ���ظ�

//�����������
/*file:rc522.c */
//����û������id_table������ֻ�Ǽ򵥵ıȽ�driver.name

/*platform����ƥ���豸�����������ַ�����һ��ͨ��platform_driver�е�id_table��
���id_table�����ڣ���ֻ�Ǽ򵥵ıȽ��豸�е�name�ֶκ������е�name�ֶ��Ƿ���ͬ.
��ôͨ��id_table���ַ�ʽ��ʲô�ô��أ����ֻ�Ǽ򵥵ıȽ�name�ֶ��Ƿ���ͬ����ô
һ������ֻ��֧���ض���һ���豸�������ͨ��id_table�ķ�ʽ�أ�һ����������֧��
�ܶ���豸��������ֻ��name�ֶβ�ͬ���ѡ�
*/
static struct spi_driver rc522_spi_driver = {
	.driver = {
		.name =	"rc522",
		.owner =	THIS_MODULE,
	},
	.probe =	rc522_probe,
	.remove = __devexit_p(rc522_remove),
};
static int __init rc522_init(void)
{
	int status;
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, "rc522_test", &rc522_fops);//ע��
	rc522_class = class_create(THIS_MODULE, "rc522");
	status = spi_register_driver(&rc522_spi_driver);
}
module_init(rc522_init);
static void __exit rc522_exit(void)
{
	spi_unregister_driver(&rc522_spi_driver);
	class_destroy(rc522_class);
	unregister_chrdev(SPIDEV_MAJOR, rc522_spi_driver.driver.name);
}
module_exit(rc522_exit);
/*-----------------------------------------------------------------------------------
samsung rc522.c��������probe��������
-----------------------------------------------------------------------------------*/
static int __devinit rc522_probe(struct spi_device *spi)
{
	struct rc522_data *rc522;
	int status;
	unsigned long minor;

	/* Allocate driver data */
	//����ռ䣬kzallocʵ����kmalloc�Լ�memset����һ��������������������
	rc522 = kzalloc(sizeof(*rc522), GFP_KERNEL);
	if (!rc522)
		return -ENOMEM;

	/* reset */
	rc522_reset();//��EXYNOS4_GPK1(0)��λ����һ�������ش���

	/* Initialize the driver data */
	//����rc522�������ݣ�����SPI
	rc522->spi = spi;
	spin_lock_init(&rc522->spi_lock);
	mutex_init(&rc522->buf_lock);

	INIT_LIST_HEAD(&rc522->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	//�ҵ�һ��δʹ�õĴ��豸��
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		rc522->devt = MKDEV(SPIDEV_MAJOR, minor);//����mdev

		dev = device_create(rc522_class, &spi->dev, rc522->devt,
				    rc522, "rc522",
				    spi->master->bus_num, spi->chip_select);//����

		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&rc522->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if (status == 0)
		spi_set_drvdata(spi, rc522);
	else
		kfree(rc522);

	return status;
}
//ops����������rc552_init��ں�����ע��
static const struct file_operations rc522_fops = {
	.owner =	THIS_MODULE,
	.write = rc522_write,
	.read = rc522_read,
	.unlocked_ioctl = rc522_ioctl,
	.compat_ioctl = rc522_compat_ioctl,
	.open = rc522_open,
	.release = rc522_release,
	.llseek =	no_llseek,
};






































