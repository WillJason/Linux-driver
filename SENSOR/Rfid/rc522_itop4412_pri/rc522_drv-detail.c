/*
硬件平台：itop4412
系统：linux-3.0.5
	MF RC522 利用了先进的调制和解调概念，完全集成了在13.56MHz 下所有类型的被
动非接触式通信方式和协议。支持 ISO14443A 的多层应用。其内部发送器部分可驱
动读写器天线与ISO 14443A/MIFARE卡和应答机的通信，无需其它的电路。接收器部
分提供一个坚固而有效的解调和解码电路，用于处理ISO14443A 兼容的应答器信号。
数字部分处理ISO14443A 帧和错误检测（奇偶 &CRC）。此外，它还支持快速CRYPTO1 
加密算法，用于验证MIFARE 系列产品。MFRC522 支持MIFARE?更高速的非接触式通信，
双向数据传输速率高达424kbit/s。

SPI在linux中分为3层分别为主机控制器驱动、核心层驱动和外设驱动：
在我们这个板级驱动架构是：
			主机控制器驱动：spi_s3c64xx.c--------------这个主要处理SPI总线相关的主机控制器驱动
     
     	核心层驱动 ：spi.c---------------------------这个文件是实现了spi的注册和注销的函数等，连接
     		了主机控制器和外设驱动，起到了桥梁的作用。
     
      外设驱动  ：rc522.c----------------------内核中的一个非接触式读写卡芯片外设控制器。
      
 	spi_s3c24xx.c本身并没有生成设备节点，rc522.c的任务就是为了在user space或kernel space访问spi设
备而设，就是通过它来进行spi设备的读写。

rc522驱动位于drivers/spi目录。驱动文件为drivers/spi/rc522.c。
*/
//先来看一下SPI总线的主机控制器相关的平台设备注册的信息
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
//这个函数主要设置spi接口硬件，类似I2C的s3c_i2c1_set_platdata，同样在入口函数会调用
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
//信息添加到入口函数
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
//再来看一下rc522外设注册的信息
//首先内核里的rc522外设驱动配置选项:
Symbol: SPI_RC522 [=y]                                              
  Type  : tristate                                               
  Prompt: RC522 Module driver support                            
	  Defined at drivers/spi/Kconfig:470                            
	  Depends on: SPI [=y] && SPI_MASTER [=y] && EXPERIMENTAL [=y]             
	  Location:                                                                                                                           
		  -> Device Drivers                                                                                                                     
		  -> SPI support (SPI [=y])   
//注册信息设置
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
//信息同样添加到入口函数
static void __init smdk4x12_machine_init(void)
{
	spi_register_board_info(spi2_board_info, ARRAY_SIZE(spi2_board_info));
}

//设备信息注册分析完毕，接下来分析驱动
//主机控制器驱动
/*file:spi_s3c64xx.c*/
//在别的目录下已经有分析，这里不再重复

//外设驱动相关
/*file:rc522.c */
//这里没有设置id_table，所以只是简单的比较driver.name

/*platform总线匹配设备和驱动有两种方法：一是通过platform_driver中的id_table，
如果id_table不存在，则只是简单的比较设备中的name字段和驱动中的name字段是否相同.
那么通过id_table这种方式有什么好处呢，如果只是简单的比较name字段是否相同，那么
一个驱动只能支持特定的一个设备，而如果通过id_table的方式呢，一个驱动可以支持
很多个设备，而它们只是name字段不同而已。
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
	status = register_chrdev(SPIDEV_MAJOR, "rc522_test", &rc522_fops);//注册
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
samsung rc522.c外设驱动probe函数分析
-----------------------------------------------------------------------------------*/
static int __devinit rc522_probe(struct spi_device *spi)
{
	struct rc522_data *rc522;
	int status;
	unsigned long minor;

	/* Allocate driver data */
	//分配空间，kzalloc实现了kmalloc以及memset功能一个函数起到两个函数作用
	rc522 = kzalloc(sizeof(*rc522), GFP_KERNEL);
	if (!rc522)
		return -ENOMEM;

	/* reset */
	rc522_reset();//给EXYNOS4_GPK1(0)复位引脚一个上升沿触发

	/* Initialize the driver data */
	//设置rc522驱动数据，挂载SPI
	rc522->spi = spi;
	spin_lock_init(&rc522->spi_lock);
	mutex_init(&rc522->buf_lock);

	INIT_LIST_HEAD(&rc522->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	//找到一个未使用的次设备号
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		rc522->devt = MKDEV(SPIDEV_MAJOR, minor);//生成mdev

		dev = device_create(rc522_class, &spi->dev, rc522->devt,
				    rc522, "rc522",
				    spi->master->bus_num, spi->chip_select);//创建

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
//ops操作函数在rc552_init入口函数中注册
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






































