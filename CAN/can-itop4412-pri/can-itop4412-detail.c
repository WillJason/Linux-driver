/*
精英板的itop4412的CAN 驱动需要 SPI 总线支持。

CAN总线是一种在汽车上广泛采用的总线协议，被设计作为汽车环境中的微控制器通讯。
LZ理论知识有限，网上抄一句介绍的吧。如下：CAN(Controller Area Network)总线，即控制器局域网总线，
是一种有效支持分布式控制或实时控制的串行通信网络。由于其高性能、高可靠性、及独特的设计和适宜的
价格而广泛应用于工业现场控制、智能楼宇、医疗器械、交通工具以及传感器等领域，并已被公认为几种最
有前途的现场总线之一。CAN总线规范已经被国际标准化组织制订为国际标准ISO11898，
并得到了众多半导体器件厂商的支持。

电路相关的资料不在这里介绍了，可以查看博客等资料。

Socket CAN的设计克服了将CAN设备驱动实现为字符设备驱动带来的局限性，因为字符设备驱动直接操作控制器
硬件进行帧的收发、帧的排队以及一些高层传输协议只能在应用层实现。另外，字符设备驱动只能单进程进行
访问，不支持多进程同时操作。
SocketCAN的设计基于新的协议族PF_CAN.协议族PF_CAN一方面向应用程序提供Socket接口，另一方面它构建在Linux
网络体系结构中的网络层之上，从而可以更有效利用Linux网络子系统中的各种排队策略。CAN控制器被注册成一个
网络设备，这样的控制器收到的帧就可以传送给网络层，进而传送到CAN协议族部分（帧发送的过程的传递方向
与此相反）。同时，协议族模块提供传输层协议动态加载和卸载接口函数，更好地支持使用各种CAN传输层协议
（目前协议族中只包括两种CAN协议：CAN_RAW和CAN_BCM）。此外协议族还支持各种CAN帧的订阅，支持多个进程
同时进行SOCKET CAN通信，每个进程可以同时使用不同协议进行各种帧的收发。

CAN子系统：
can子系统实现协议族PF_CAN，主要包括三个C文件：af_can.c、raw.c和bcm.c。其中af_can.c是整个子系统的
核心管理文件，raw.c和bcm.c分别是raw和bcm协议的实现文件。CAN子系统与其他模块的关系如图：

						BSD Socket Layer
								|
						CAN子系统
			CAN_RAW        CAN_BCM
								|
				  Network	Layer		
				  			|
				  	CAN设备驱动
				  	
		af_can.c是Socket CAN的核心管理模块。can_creat()创建CAN通信所需的socket。当应用程序调用socket（）
创建socket时，就会简介调用此函数：can_proto_register(struct can_proto *)和can_proto_unregister(
struct can_proto *)可被用来动态加载和卸载CAN传输协议，传输协议在此用struct can_proto表示。
CAN_RAW和CAN_BCM协议分别定义如下：
	static const struct can_proto raw_can_proto = {
	.type       = SOCK_RAW,
	.protocol   = CAN_RAW,
	.ops        = &raw_ops,
	.prot       = &raw_proto,
};

static const struct can_proto bcm_can_proto = {
	.type       = SOCK_DGRAM,
	.protocol   = CAN_BCM,
	.ops        = &bcm_ops,
	.prot       = &bcm_proto,
};

can_rcv()是网络层用来接收包的操作函数，与包对应的操作函数通过定义struct packet_type来指明：
 *
 * af_can module init/exit functions
 *

static struct packet_type can_packet __read_mostly = {
	.type = cpu_to_be16(ETH_P_CAN),
	.dev  = NULL,
	.func = can_rcv,
};
	can_rcv()中调用can_rcv_filter()对收到的CAN帧进行过滤处理，只接收用户通过can_rx_register()订阅的
CAN帧；与can_rx_register()对应的函数can_rx_unregister（）用来取消用户订阅的CAN帧。
	raw.c和bcm.c分别是协议族里用来实现raw.socket和bcm.socket通信所需的文件。利用CAN_RAW和CAN_BCM协议
均能实现帧ID的订阅，并且利用CAN_BCM协议还能进行帧内容的过滤。
	其中，raw_rcv()和bcm_send_to_user()是传输层用来接收CAN帧的操作函数，当can_rcv()接收到用户订阅的帧
时，就会调用这两函数将帧放到接收队列中，然后raw_recvmsg()或bcm_recvmsg()就会通过调用
skb_recv_datagram()来从接收队列中取出帧，从而把帧进一步传送到上层；
	raw_sendmsg()和bcm_sendmsg()通过调用can_send()进行帧的发送，而can_send()会调用dev_queue_xmit()来
向CAN设备传输一个CAN帧，然后dev_queue_xmit()会间接调用CAN设备驱动hard_start_xmit()来真正实现帧的
发送。

Socket CAN驱动程序设计
	Linux驱动程序是Linux内核主要组成部分，其功能主要是操作硬件设备，为用户屏蔽设备的工作细节，并向
用户提供透明访问硬件设备的机制。Linux驱动程序支持3种类型的设备：字符设备、块设备和网络设备。
	独立的CAN控制器MCP2515实现CAN协议的物理层与数据链路层，本文的应用中将其作为网络设备处理。同时,
MCP2515通过SPI接口连接到主控器，其也被视为SPI从设备。因此，驱动程序不仅要操作CAN控制器，而且要能
很好地与Linux CAN子系统、网络子系统、Linux SPI核心进行交互。
CAN网络设备操作函数的定义如下：
	static const struct net_device_ops mcp251x_netdev_ops = {
	.ndo_open = mcp251x_open,
	.ndo_stop = mcp251x_stop,
	.ndo_start_xmit = mcp251x_hard_start_xmit,
};
结构体中部分成员的功能描述如下：
	mcp251x_open（）负责对CAN控制器进行复位初始化，并且调用netif_wake_queue()通知上层启动新的传输。
	mcp251x_stop（）使控制器停止工作并让其睡眠。
	mcp251x_hard_start_xmit（）负责将帧数据写到控制器的相应寄存器和发送缓存区中，并进行发送。
	
SPI从设备信息定义如下：
	#ifdef CONFIG_CAN_MCP251X
	{
		.modalias = "mcp2515",
		.platform_data = &mcp251x_info,
		
	#if defined(CONFIG_CPU_TYPE_SCP_ELITE) || defined(CONFIG_CPU_TYPE_POP_ELITE) || defined(CONFIG_CPU_TYPE_POP2G_ELITE) //add by dg 2015 08 10

         	.irq = IRQ_EINT(1),
	#elif defined(CONFIG_CPU_TYPE_SCP_SUPPER) || defined(CONFIG_CPU_TYPE_POP_SUPPER) || defined(CONFIG_CPU_TYPE_POP2G_SUPPER)
             .irq = IRQ_EINT(0),
	#endif
	
		.max_speed_hz = 10*1000*1000,
		.bus_num = 2,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.controller_data = &spi2_csi[0],
	}
	#endif
	
	static struct mcp251x_platform_data mcp251x_info = {
	.oscillator_frequency = 8000000,
	.board_specific_setup = mcp251x_ioSetup,
	};

	另外，驱动中还涉及几个比较重要函数。
	
*/
/*
	主机驱动与外设驱动分离的设计思想

12.4.1　主机驱动与外设驱动分离

Linux中SPI、I2C、USB等子系统都利用典型的把主机驱动和外设驱动分离的想法，主机端只负责产生总
线上的传输波形，而外设端通过标准的API让主机端以适当的波形访问自身。

这里涉及4个软件模块：

1）主机端的驱动。根据具体的I2C、SPI、USB等控制器的硬件手册，操作具体的I2C、SPI、USB等控制器，
产生总线的各种波形。

2）连接主机和外设的纽带。外设不直接调用主机端的驱动来产生波形，而是调一个标准的API。由这个标
准的API把这个波形的传输请求间接“转发”给了具体的主机端驱动。在这里，最好把关于波形的描述也以
某种数据结构标准化。

3）外设端的驱动。外设接在I2C、SPI、USB总线上，但外设本身可以是触摸屏、网卡、声卡或任意一种类
型的设备。在相关的i2c_driver、spi_driver、usb_driver这种xxx_driver的probe（）函数中去注册它
具体的类型。当这些外设要求I2C、SPI、USB等去访问它的时候，它调用“连接主机和外设的纽带”模块的
标准API。

4）板级逻辑。板级逻辑用来描述主机和外设是如何互联的。假设板子上有多个SPI控制器和多个SPI外设，
究竟谁接在谁上面管理互联关系，既不是主机端的责任，也不是外设端的责任，这属于板级逻辑的责任。
通常出现在arch/arm/mach-xxx下面或者arch/arm/boot/dts下面。

SPI在linux中分为3层分别为主机控制器驱动、核心层驱动和外设驱动：

在我们这个板级驱动架构是：
			主机控制器驱动：spi_s3c64xx.c--------------这个文件将s3c6410的硬件spi控制器驱动。
     
     	核心层驱动 ：spi.c---------------------------这个文件是实现了spi的注册和注销的函数等，连接
     		了主机控制器和外设驱动，起到了桥梁的作用。
     
      外设驱动  ：mcp251x.c----------------------内核中的一个CAN外设控制器。
      
 	spi_s3c24xx.c本身并没有生成设备节点，mcp251x.c的任务就是为了在user space或kernel space访问spi设
备而设，就是通过它来进行spi设备的读写。

*/
////需要添加lcd的平台设备信息
/*file:mach-itop4412.c*/
static void __init smdk4x12_machine_init(void)
{
	//#ifndef CONFIG_CAN_MCP251X 如果使用了CAN，那么我们不可以使能同管脚的i2c 6接口
	#if !defined(CONFIG_CAN_MCP251X) && !defined(CONFIG_SPI_RC522)
	s3c_i2c6_set_platdata(NULL);
	i2c_register_board_info(6, i2c_devs6, ARRAY_SIZE(i2c_devs6));
	#endif
	
	//外设驱动设备信息注册，跟外设驱动相关的，外设驱动：mcp251x.c
	#ifdef CONFIG_S3C64XX_DEV_SPI
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

	spi_register_board_info(spi2_board_info, ARRAY_SIZE(spi2_board_info));
	#endif
	
	//平台设备注册
	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
}	

//Mcp2515有标准的驱动，可以从网上找到下载，linux的内核里边也有默认的驱动。
//所以只要配置好内核，添加设备端的配置信息就好了。以下是我的配置信息：
static struct spi_board_info spi2_board_info[] __initdata = {
#ifdef CONFIG_CAN_MCP251X
	{
		.modalias = "mcp2515",//与mcp251x.c中mcp251x_id_table相匹配
		.platform_data = &mcp251x_info,
		
#if defined(CONFIG_CPU_TYPE_SCP_ELITE) || defined(CONFIG_CPU_TYPE_POP_ELITE) || defined(CONFIG_CPU_TYPE_POP2G_ELITE) //add by dg 2015 08 10

         	.irq = IRQ_EINT(1),
#elif defined(CONFIG_CPU_TYPE_SCP_SUPPER) || defined(CONFIG_CPU_TYPE_POP_SUPPER) || defined(CONFIG_CPU_TYPE_POP2G_SUPPER)
             .irq = IRQ_EINT(0),
#endif
	
		.max_speed_hz = 10*1000*1000,
		.bus_num = 2,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.controller_data = &spi2_csi[0],
	}
#endif
};

static struct mcp251x_platform_data mcp251x_info = {
	.oscillator_frequency = 8000000,
	.board_specific_setup = mcp251x_ioSetup,
};
static struct s3c64xx_spi_csinfo spi2_csi[] = {
	[0] = {
		.line = EXYNOS4_GPC1(2),
		.set_level = gpio_set_value,
		.fb_delay = 0x2,
	},
};
	
static struct platform_device *smdk4x12_devices[] __initdata = {
	#ifdef CONFIG_S3C64XX_DEV_SPI
	#if 0	//remove by cym 20130529
		&exynos_device_spi0,
	#ifndef CONFIG_FB_S5P_LMS501KF03
		&exynos_device_spi1,
	#endif
	#endif
		&exynos_device_spi2,
	#endif
};

//平台相关
/* file:dev_spi.c */
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

//主机控制器相关
/*file:spi_s3c64xx.c */
static struct platform_driver s3c64xx_spi_driver = {
	.driver = {
		.name	= "s3c64xx-spi",//与exynos_device_spi2的name相匹配
		.owner = THIS_MODULE,
	},
	.remove = s3c64xx_spi_remove,
	.suspend = s3c64xx_spi_suspend,
	.resume = s3c64xx_spi_resume,
};
static int __init s3c64xx_spi_init(void)
{
	return platform_driver_probe(&s3c64xx_spi_driver, s3c64xx_spi_probe);
}

//外设驱动相关
/*file:mcp251x.c */
static const struct spi_device_id mcp251x_id_table[] = {
	{ "mcp2510",	CAN_MCP251X_MCP2510 },
	{ "mcp2515",	CAN_MCP251X_MCP2515 },
	{ },
};

MODULE_DEVICE_TABLE(spi, mcp251x_id_table);

static struct spi_driver mcp251x_can_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},

	.id_table = mcp251x_id_table,
	.probe = mcp251x_can_probe,
	.remove = __devexit_p(mcp251x_can_remove),
	.suspend = mcp251x_can_suspend,
	.resume = mcp251x_can_resume,
};













