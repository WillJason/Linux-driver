/*
��Ӣ���itop4412��CAN ������Ҫ SPI ����֧�֡�

CAN������һ���������Ϲ㷺���õ�����Э�飬�������Ϊ���������е�΢������ͨѶ��
LZ����֪ʶ���ޣ����ϳ�һ����ܵİɡ����£�CAN(Controller Area Network)���ߣ������������������ߣ�
��һ����Ч֧�ֲַ�ʽ���ƻ�ʵʱ���ƵĴ���ͨ�����硣����������ܡ��߿ɿ��ԡ������ص���ƺ����˵�
�۸���㷺Ӧ���ڹ�ҵ�ֳ����ơ�����¥�ҽ����е����ͨ�����Լ������������򣬲��ѱ�����Ϊ������
��ǰ;���ֳ�����֮һ��CAN���߹淶�Ѿ������ʱ�׼����֯�ƶ�Ϊ���ʱ�׼ISO11898��
���õ����ڶ�뵼���������̵�֧�֡�

��·��ص����ϲ�����������ˣ����Բ鿴���͵����ϡ�

Socket CAN����ƿ˷��˽�CAN�豸����ʵ��Ϊ�ַ��豸���������ľ����ԣ���Ϊ�ַ��豸����ֱ�Ӳ���������
Ӳ������֡���շ���֡���Ŷ��Լ�һЩ�߲㴫��Э��ֻ����Ӧ�ò�ʵ�֡����⣬�ַ��豸����ֻ�ܵ����̽���
���ʣ���֧�ֶ����ͬʱ������
SocketCAN����ƻ����µ�Э����PF_CAN.Э����PF_CANһ������Ӧ�ó����ṩSocket�ӿڣ���һ������������Linux
������ϵ�ṹ�е������֮�ϣ��Ӷ����Ը���Ч����Linux������ϵͳ�еĸ����ŶӲ��ԡ�CAN��������ע���һ��
�����豸�������Ŀ������յ���֡�Ϳ��Դ��͸�����㣬�������͵�CANЭ���岿�֣�֡���͵Ĺ��̵Ĵ��ݷ���
����෴����ͬʱ��Э����ģ���ṩ�����Э�鶯̬���غ�ж�ؽӿں��������õ�֧��ʹ�ø���CAN�����Э��
��ĿǰЭ������ֻ��������CANЭ�飺CAN_RAW��CAN_BCM��������Э���廹֧�ָ���CAN֡�Ķ��ģ�֧�ֶ������
ͬʱ����SOCKET CANͨ�ţ�ÿ�����̿���ͬʱʹ�ò�ͬЭ����и���֡���շ���

CAN��ϵͳ��
can��ϵͳʵ��Э����PF_CAN����Ҫ��������C�ļ���af_can.c��raw.c��bcm.c������af_can.c��������ϵͳ��
���Ĺ����ļ���raw.c��bcm.c�ֱ���raw��bcmЭ���ʵ���ļ���CAN��ϵͳ������ģ��Ĺ�ϵ��ͼ��

						BSD Socket Layer
								|
						CAN��ϵͳ
			CAN_RAW        CAN_BCM
								|
				  Network	Layer		
				  			|
				  	CAN�豸����
				  	
		af_can.c��Socket CAN�ĺ��Ĺ���ģ�顣can_creat()����CANͨ�������socket����Ӧ�ó������socket����
����socketʱ���ͻ�����ô˺�����can_proto_register(struct can_proto *)��can_proto_unregister(
struct can_proto *)�ɱ�������̬���غ�ж��CAN����Э�飬����Э���ڴ���struct can_proto��ʾ��
CAN_RAW��CAN_BCMЭ��ֱ������£�
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

can_rcv()��������������հ��Ĳ��������������Ӧ�Ĳ�������ͨ������struct packet_type��ָ����
 *
 * af_can module init/exit functions
 *

static struct packet_type can_packet __read_mostly = {
	.type = cpu_to_be16(ETH_P_CAN),
	.dev  = NULL,
	.func = can_rcv,
};
	can_rcv()�е���can_rcv_filter()���յ���CAN֡���й��˴���ֻ�����û�ͨ��can_rx_register()���ĵ�
CAN֡����can_rx_register()��Ӧ�ĺ���can_rx_unregister��������ȡ���û����ĵ�CAN֡��
	raw.c��bcm.c�ֱ���Э����������ʵ��raw.socket��bcm.socketͨ��������ļ�������CAN_RAW��CAN_BCMЭ��
����ʵ��֡ID�Ķ��ģ���������CAN_BCMЭ�黹�ܽ���֡���ݵĹ��ˡ�
	���У�raw_rcv()��bcm_send_to_user()�Ǵ������������CAN֡�Ĳ�����������can_rcv()���յ��û����ĵ�֡
ʱ���ͻ��������������֡�ŵ����ն����У�Ȼ��raw_recvmsg()��bcm_recvmsg()�ͻ�ͨ������
skb_recv_datagram()���ӽ��ն�����ȡ��֡���Ӷ���֡��һ�����͵��ϲ㣻
	raw_sendmsg()��bcm_sendmsg()ͨ������can_send()����֡�ķ��ͣ���can_send()�����dev_queue_xmit()��
��CAN�豸����һ��CAN֡��Ȼ��dev_queue_xmit()���ӵ���CAN�豸����hard_start_xmit()������ʵ��֡��
���͡�

Socket CAN�����������
	Linux����������Linux�ں���Ҫ��ɲ��֣��书����Ҫ�ǲ���Ӳ���豸��Ϊ�û������豸�Ĺ���ϸ�ڣ�����
�û��ṩ͸������Ӳ���豸�Ļ��ơ�Linux��������֧��3�����͵��豸���ַ��豸�����豸�������豸��
	������CAN������MCP2515ʵ��CANЭ����������������·�㣬���ĵ�Ӧ���н�����Ϊ�����豸����ͬʱ,
MCP2515ͨ��SPI�ӿ����ӵ�����������Ҳ����ΪSPI���豸����ˣ��������򲻽�Ҫ����CAN������������Ҫ��
�ܺõ���Linux CAN��ϵͳ��������ϵͳ��Linux SPI���Ľ��н�����
CAN�����豸���������Ķ������£�
	static const struct net_device_ops mcp251x_netdev_ops = {
	.ndo_open = mcp251x_open,
	.ndo_stop = mcp251x_stop,
	.ndo_start_xmit = mcp251x_hard_start_xmit,
};
�ṹ���в��ֳ�Ա�Ĺ����������£�
	mcp251x_open���������CAN���������и�λ��ʼ�������ҵ���netif_wake_queue()֪ͨ�ϲ������µĴ��䡣
	mcp251x_stop����ʹ������ֹͣ����������˯�ߡ�
	mcp251x_hard_start_xmit��������֡����д������������Ӧ�Ĵ����ͷ��ͻ������У������з��͡�
	
SPI���豸��Ϣ�������£�
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

	���⣬�����л��漰�����Ƚ���Ҫ������
	
*/
/*
	��������������������������˼��

12.4.1������������������������

Linux��SPI��I2C��USB����ϵͳ�����õ��͵İ�������������������������뷨��������ֻ���������
���ϵĴ��䲨�Σ��������ͨ����׼��API�����������ʵ��Ĳ��η�������

�����漰4�����ģ�飺

1�������˵����������ݾ����I2C��SPI��USB�ȿ�������Ӳ���ֲᣬ���������I2C��SPI��USB�ȿ�������
�������ߵĸ��ֲ��Ρ�

2�����������������Ŧ�������費ֱ�ӵ��������˵��������������Σ����ǵ�һ����׼��API���������
׼��API��������εĴ��������ӡ�ת�������˾�����������������������ðѹ��ڲ��ε�����Ҳ��
ĳ�����ݽṹ��׼����

3������˵��������������I2C��SPI��USB�����ϣ������豾������Ǵ�����������������������һ����
�͵��豸������ص�i2c_driver��spi_driver��usb_driver����xxx_driver��probe����������ȥע����
��������͡�����Щ����Ҫ��I2C��SPI��USB��ȥ��������ʱ�������á����������������Ŧ����ģ���
��׼API��

4���弶�߼����弶�߼�����������������������λ����ġ�����������ж��SPI�������Ͷ��SPI���裬
����˭����˭�����������ϵ���Ȳ��������˵����Σ�Ҳ��������˵����Σ������ڰ弶�߼������Ρ�
ͨ��������arch/arm/mach-xxx�������arch/arm/boot/dts���档

SPI��linux�з�Ϊ3��ֱ�Ϊ�������������������Ĳ�����������������

����������弶�����ܹ��ǣ�
			����������������spi_s3c64xx.c--------------����ļ���s3c6410��Ӳ��spi������������
     
     	���Ĳ����� ��spi.c---------------------------����ļ���ʵ����spi��ע���ע���ĺ����ȣ�����
     		�������������������������������������á�
     
      ��������  ��mcp251x.c----------------------�ں��е�һ��CAN�����������
      
 	spi_s3c24xx.c����û�������豸�ڵ㣬mcp251x.c���������Ϊ����user space��kernel space����spi��
�����裬����ͨ����������spi�豸�Ķ�д��

*/
////��Ҫ���lcd��ƽ̨�豸��Ϣ
/*file:mach-itop4412.c*/
static void __init smdk4x12_machine_init(void)
{
	//#ifndef CONFIG_CAN_MCP251X ���ʹ����CAN����ô���ǲ�����ʹ��ͬ�ܽŵ�i2c 6�ӿ�
	#if !defined(CONFIG_CAN_MCP251X) && !defined(CONFIG_SPI_RC522)
	s3c_i2c6_set_platdata(NULL);
	i2c_register_board_info(6, i2c_devs6, ARRAY_SIZE(i2c_devs6));
	#endif
	
	//���������豸��Ϣע�ᣬ������������صģ�����������mcp251x.c
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
	
	//ƽ̨�豸ע��
	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
}	

//Mcp2515�б�׼�����������Դ������ҵ����أ�linux���ں����Ҳ��Ĭ�ϵ�������
//����ֻҪ���ú��ںˣ�����豸�˵�������Ϣ�ͺ��ˡ��������ҵ�������Ϣ��
static struct spi_board_info spi2_board_info[] __initdata = {
#ifdef CONFIG_CAN_MCP251X
	{
		.modalias = "mcp2515",//��mcp251x.c��mcp251x_id_table��ƥ��
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

//ƽ̨���
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

//�������������
/*file:spi_s3c64xx.c */
static struct platform_driver s3c64xx_spi_driver = {
	.driver = {
		.name	= "s3c64xx-spi",//��exynos_device_spi2��name��ƥ��
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

//�����������
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

/*-----------------------------------------------------------------------------------
samsung spi-s3c64xx.c��������������probe��������
-----------------------------------------------------------------------------------*/
static int __init s3c64xx_spi_probe(struct platform_device *pdev)
{
	struct resource	*mem_res, *dmatx_res, *dmarx_res;
	struct s3c64xx_spi_driver_data *sdd;
	struct s3c64xx_spi_info *sci;
	struct spi_master *master;
	int ret;

	printk("%s(%d)\n", __FUNCTION__, __LINE__);

	if (pdev->id < 0) {
		dev_err(&pdev->dev,
				"Invalid platform device id-%d\n", pdev->id);
		return -ENODEV;
	}

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "platform_data missing!\n");
		return -ENODEV;
	}

	sci = pdev->dev.platform_data;//��ȡ�豸��Ϣ��������ɿ���û���豸����ص��жϣ����Ը������������豸����
	if (!sci->src_clk_name) {
		dev_err(&pdev->dev,
			"Board init must call s3c64xx_spi_set_info()\n");
		return -EINVAL;
	}

	/* Check for availability of necessary resource */

	dmatx_res = platform_get_resource(pdev, IORESOURCE_DMA, 0);//�õ�ƽ̨����DMA tx��Դ
	if (dmatx_res == NULL) {
		dev_err(&pdev->dev, "Unable to get SPI-Tx dma resource\n");
		return -ENXIO;
	}

	dmarx_res = platform_get_resource(pdev, IORESOURCE_DMA, 1);//�õ�ƽ̨����DMA rx��Դ
	if (dmarx_res == NULL) {
		dev_err(&pdev->dev, "Unable to get SPI-Rx dma resource\n");
		return -ENXIO;
	}

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);//��ȡIO�ڴ���Դ
	if (mem_res == NULL) {
		dev_err(&pdev->dev, "Unable to get SPI MEM resource\n");
		return -ENXIO;
	}

	//Ϊ�ṹ��spi_master�ͽṹ��s3c64xx_spi_driver_data����洢�ռ䣬����s3c64xx_spi_driver_data��Ϊspi_master��drvdata��
	master = spi_alloc_master(&pdev->dev,
				sizeof(struct s3c64xx_spi_driver_data));
	if (master == NULL) {
		dev_err(&pdev->dev, "Unable to allocate SPI Master\n");
		return -ENOMEM;
	}

	/** 
     * platform_set_drvdata �� platform_get_drvdata 
     * probe�����ж���ľֲ���������������������ط�ʹ������ô���أ� 
     * �����Ҫ���������������ں��ṩ����������� 
     * ʹ�ú���platform_set_drvdata()���Խ�master�����ƽ̨�����豸��˽�����ݡ� 
     * �Ժ���Ҫʹ����ʱֻ�����platform_get_drvdata()�Ϳ����ˡ� 
 */  
	platform_set_drvdata(pdev, master);

	//��master�л��s3c64xx_spi_driver_data������ʼ����س�Ա  
	sdd = spi_master_get_devdata(master);
	sdd->master = master;
	sdd->cntrlr_info = sci;
	sdd->pdev = pdev;
	sdd->sfr_start = mem_res->start;
	sdd->tx_dmach = dmatx_res->start;
	sdd->rx_dmach = dmarx_res->start;

	sdd->cur_bpw = 8;

	//master��س�Ա�ĳ�ʼ�� 
	master->bus_num = pdev->id;//���ߺ�
	master->setup = s3c64xx_spi_setup;
	master->transfer = s3c64xx_spi_transfer;
	master->num_chipselect = sci->num_cs;//�������ϵ��豸�� 
	master->dma_alignment = 8;
	/* the spi->mode bits understood by this driver: */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;

	if (request_mem_region(mem_res->start,
			resource_size(mem_res), pdev->name) == NULL) {
		dev_err(&pdev->dev, "Req mem region failed\n");
		ret = -ENXIO;
		goto err0;
	}

	sdd->regs = ioremap(mem_res->start, resource_size(mem_res));//����IO�ڴ� 
	if (sdd->regs == NULL) {
		dev_err(&pdev->dev, "Unable to remap IO\n");
		ret = -ENXIO;
		goto err1;
	}

	//SPI��IO�ܽ����ã�����Ӧ��IO�ܽ�����ΪSPI���� 
	if (sci->cfg_gpio == NULL || sci->cfg_gpio(pdev)) {
		dev_err(&pdev->dev, "Unable to config gpio\n");
		ret = -EBUSY;
		goto err2;
	}

	//ʹ��ʱ��  
	/* Setup clocks */
	sdd->clk = clk_get(&pdev->dev, "spi");
	if (IS_ERR(sdd->clk)) {
		dev_err(&pdev->dev, "Unable to acquire clock 'spi'\n");
		ret = PTR_ERR(sdd->clk);
		goto err3;
	}

	if (clk_enable(sdd->clk)) {
		dev_err(&pdev->dev, "Couldn't enable clock 'spi'\n");
		ret = -EBUSY;
		goto err4;
	}

	sdd->src_clk = clk_get(&pdev->dev, sci->src_clk_name);
	if (IS_ERR(sdd->src_clk)) {
		dev_err(&pdev->dev,
			"Unable to acquire clock '%s'\n", sci->src_clk_name);
		ret = PTR_ERR(sdd->src_clk);
		goto err5;
	}

	if (clk_enable(sdd->src_clk)) {
		dev_err(&pdev->dev, "Couldn't enable clock '%s'\n",
							sci->src_clk_name);
		ret = -EBUSY;
		goto err6;
	}

	sdd->workqueue = create_singlethread_workqueue(
						dev_name(master->dev.parent));
	if (sdd->workqueue == NULL) {
		dev_err(&pdev->dev, "Unable to create workqueue\n");
		ret = -ENOMEM;
		goto err7;
	}
	printk("%s(%d)\n", __FUNCTION__, __LINE__);
	
	//Ӳ����ʼ������ʼ�����üĴ�����������SPIMOSI��SPIMISO��SPICLK���ŵ����� 
	/* Setup Deufult Mode */
	s3c64xx_spi_hwinit(sdd, pdev->id);

	//�����������еȳ�ʼ�� 
	spin_lock_init(&sdd->lock);
	init_completion(&sdd->xfer_completion);
	INIT_WORK(&sdd->work, s3c64xx_spi_work);
	INIT_LIST_HEAD(&sdd->queue);

	//ע��spi_master��spi��ϵͳ��ȥ
	if (spi_register_master(master)) {
		dev_err(&pdev->dev, "cannot register SPI master\n");
		ret = -EBUSY;
		goto err8;
	}

	dev_dbg(&pdev->dev, "Samsung SoC SPI Driver loaded for Bus SPI-%d "
					"with %d Slaves attached\n",
					pdev->id, master->num_chipselect);
	dev_dbg(&pdev->dev, "\tIOmem=[0x%x-0x%x]\tDMA=[Rx-%d, Tx-%d]\n",
					mem_res->end, mem_res->start,
					sdd->rx_dmach, sdd->tx_dmach);
	printk("%s(%d)\n", __FUNCTION__, __LINE__);
	return 0;

err8:
	destroy_workqueue(sdd->workqueue);
err7:
	clk_disable(sdd->src_clk);
err6:
	clk_put(sdd->src_clk);
err5:
	clk_disable(sdd->clk);
err4:
	clk_put(sdd->clk);
err3:
err2:
	iounmap((void *) sdd->regs);
err1:
	release_mem_region(mem_res->start, resource_size(mem_res));
err0:
	platform_set_drvdata(pdev, NULL);
	spi_master_put(master);

	return ret;
}

/*-----------------------------------------------------------------------------------
mcp251x.c��������probe��������
-----------------------------------------------------------------------------------*/
//����������һ���豸��ʼ������
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

static int __init mcp251x_can_init(void)
{
	return spi_register_driver(&mcp251x_can_driver);
}
//����һ���ǳ���Ҫ�Ľṹ�壬��probe�����б�ע��
static const struct net_device_ops mcp251x_netdev_ops = {
	.ndo_open = mcp251x_open,
	.ndo_stop = mcp251x_stop,
	.ndo_start_xmit = mcp251x_hard_start_xmit,
};

static int __devinit mcp251x_can_probe(struct spi_device *spi)
{
	struct net_device *net;
	struct mcp251x_priv *priv;
	struct mcp251x_platform_data *pdata = spi->dev.platform_data;//��ȡSPI�豸��Ϣ
	int ret = -ENODEV;

	if (!pdata)
		/* Platform data is required for osc freq */
		goto error_out;

	//���������CAN�����豸�Ŀռ�
	/* Allocate can/net device */
	net = alloc_candev(sizeof(struct mcp251x_priv), TX_ECHO_SKB_MAX);
	if (!net) {
		ret = -ENOMEM;
		goto error_alloc;
	}

	net->netdev_ops = &mcp251x_netdev_ops;
	net->flags |= IFF_ECHO;

	//ͨ��ָ��ƫ�ƻ��˽�����ݵ��׵�ַ
	priv = netdev_priv(net);

	//����CAN��˽������
	/* add by cym 20131018 */
	priv->can.bittiming.bitrate = 250000;//������
	/* end add */
	
	priv->can.bittiming_const = &mcp251x_bittiming_const;
	priv->can.do_set_mode = mcp251x_do_set_mode;
	priv->can.clock.freq = pdata->oscillator_frequency / 2;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_3_SAMPLES |
		CAN_CTRLMODE_LOOPBACK | CAN_CTRLMODE_LISTENONLY;
	priv->model = spi_get_device_id(spi)->driver_data;//��ȡоƬ�ͺ�
	priv->net = net;//ָ�������豸
	dev_set_drvdata(&spi->dev, priv);

	priv->spi = spi;//ָ��SPI�豸
	mutex_init(&priv->mcp_lock);

	/* If requested, allocate DMA buffers */
	if (mcp251x_enable_dma) {
		spi->dev.coherent_dma_mask = ~0;

		/*
		 * Minimum coherent DMA allocation is PAGE_SIZE, so allocate
		 * that much and share it between Tx and Rx DMA buffers.
		 */
		priv->spi_tx_buf = dma_alloc_coherent(&spi->dev,
						      PAGE_SIZE,
						      &priv->spi_tx_dma,
						      GFP_DMA);

		if (priv->spi_tx_buf) {
			priv->spi_rx_buf = (u8 *)(priv->spi_tx_buf +
						  (PAGE_SIZE / 2));
			priv->spi_rx_dma = (dma_addr_t)(priv->spi_tx_dma +
							(PAGE_SIZE / 2));
		} else {
			/* Fall back to non-DMA */
			mcp251x_enable_dma = 0;
		}
	}

	/* Allocate non-DMA buffers */
	if (!mcp251x_enable_dma) {
		priv->spi_tx_buf = kmalloc(SPI_TRANSFER_BUF_LEN, GFP_KERNEL);
		if (!priv->spi_tx_buf) {
			ret = -ENOMEM;
			goto error_tx_buf;
		}
		priv->spi_rx_buf = kmalloc(SPI_TRANSFER_BUF_LEN, GFP_KERNEL);
		if (!priv->spi_rx_buf) {
			ret = -ENOMEM;
			goto error_rx_buf;
		}
	}

	if (pdata->power_enable)
		pdata->power_enable(1);

	/* Call out to platform specific setup */
	if (pdata->board_specific_setup)
		pdata->board_specific_setup(spi);

	SET_NETDEV_DEV(net, &spi->dev);

	/* Configure the SPI bus */
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi_setup(spi);

	/* Here is OK to not lock the MCP, no one knows about it yet */
	if (!mcp251x_hw_probe(spi)) {
		dev_info(&spi->dev, "Probe failed\n");
		goto error_probe;
	}
	mcp251x_hw_sleep(spi);

	if (pdata->transceiver_enable)
		pdata->transceiver_enable(0);

	//ע��net_device
	ret = register_candev(net);
	if (!ret) {
		dev_info(&spi->dev, "probed\n");
		return ret;
	}
error_probe:
	if (!mcp251x_enable_dma)
		kfree(priv->spi_rx_buf);
error_rx_buf:
	if (!mcp251x_enable_dma)
		kfree(priv->spi_tx_buf);
error_tx_buf:
	free_candev(net);
	if (mcp251x_enable_dma)
		dma_free_coherent(&spi->dev, PAGE_SIZE,
				  priv->spi_tx_buf, priv->spi_tx_dma);
error_alloc:
	if (pdata->power_enable)
		pdata->power_enable(0);
	dev_err(&spi->dev, "probe failed\n");
error_out:
	return ret;
}

//����������Socket CAN������������

/*���������û���ͨ��socket����CAN���ݵķ���ʱ����Ҫ�������²�����

    ��1�� ����һ���׽���socket������AF_CANЭ�飻

    ��2�����������׽��ַ���������sockfd���󶨵����صĵ�ַ��

    ��3��ͨ��sendtoϵͳ���ú������з��ͣ�
   int sendto(int sockfd, const void *msg, intlen,unsigned intflags, const struct sockaddr *to, int tolen);

         ��Ҫ����˵�����£�
         sockfd:ͨ��socket�������ɵ��׽�����������
         msg:��ָ��ָ����Ҫ�������ݵĻ�������
         len:�Ƿ������ݵĳ��ȣ�
         to:Ŀ��������IP��ַ���˿ں���Ϣ�� 
         
	 sendto��ϵͳ���ûᷢ��һ֡���ݱ���ָ���ĵ�ַ����CANЭ�����֮ǰ�Ѹõ�ַ�Ƶ��ں˿ռ�
�ͼ���û��ռ��������Ƿ�ɶ�����net/socket.cԴ�ļ��У�sendto������ϵͳ�������´��룺    
*/
SYSCALL_DEFINE6(sendto, int, fd, void __user *, buff, size_t, len,
		unsigned, flags, struct sockaddr __user *, addr,
		int, addr_len)
{
		...
	
	 /*���û��ռ�ĵ�ַ�ƶ����ں˿ռ���*/
		if (addr) {
		err = move_addr_to_kernel(addr, addr_len, (struct sockaddr *)&address);
		if (err < 0)
			goto out_put;
		msg.msg_name = (struct sockaddr *)&address;
		msg.msg_namelen = addr_len;
	}
	err = sock_sendmsg(sock, &msg, len);
	/*���ú���->__sock_sendmsg(&iocb, sock, msg, size);
								-> __sock_sendmsg_nosec(iocb, sock, msg, size) 
									 ��__sock_sendmsg_nosec()�����л᷵��һ��sendmsg����ָ�롣
										->sock->ops->sendmsg(iocb, sock, msg, size);
											��/net/can/raw.cԴ�ļ��У���raw_sendmsg������ַ����sendmsg����ָ�룬
											���ں���__sock_sendmsg_nosec()��return sock->ops->sendmsg(iocb,sock, msg, size)��
											���صĺ���ָ�뽫ָ��raw_sendmsg()������
											static const struct proto_ops raw_ops = {
												...
												.poll          = datagram_poll,
												.ioctl         = can_ioctl,	  //use can_ioctl() from af_can.c 
												...
												.sendmsg       = raw_sendmsg,//ָ���������
												.recvmsg       = raw_recvmsg,
												.mmap          = sock_no_mmap,
												.sendpage      = sock_no_sendpage,
											};
											
											raw_sendmsg(struct kiocb *iocb, struct socket *sock,struct msghdr *msg, size_t size)
													->can_send(skb, ro->loopback);
														 ��net/can/af_can.cԴ�ļ��У�can_send��������CANЭ�������ݴ��䣬
														 ������һ֡CAN���ģ���ѡ���ػػ���������skbָ��ָ���׽��ֻ�������
														 �����ݶε�CAN֡��loop�������ڱ���CAN�׽�����Ϊ�������ṩ�ػ���
														 ->dev_queue_xmit(skb);
														 		->dev_hard_start_xmit(skb, dev, txq);
														 			->rc = ops->ndo_start_xmit(nskb, dev);
														 				 ���¿�ʼ���е�CAN�ĵײ����������ˣ�����CAN�����Ǳ�����ں��У�
														 				 ������ϵͳ����ʱ��ע��CAN������ע��CAN���������л��ʼ��
														 				 mcp251x_netdev_ops�ṹ�����������������У�mcp251x_netdev_ops
														 				 �ṹ�����������3������ָ�룬����(*ndo_start_xmit)����
														 				 ָ��ָ��mcp251x_hard_start_xmit��������ڵ�ַ��
														 				 ->����queue_work(priv->wq, &priv->tx_work);��txָ��ĺ�������
														 				 		֮ǰmcp251x_open��ʵ��INIT_WORK(&priv->tx_work,
														 				 		mcp251x_tx_work_handler)��Ȼ����øú���׼����Ϣ���Ľ��д���

	���ϼ��Ǳ��˶�Socket CAN�������ݷ��͵���⡣
	*/
	
}
//����������Socket CAN������������

/*
	���������豸�����ݽ��մ����ϲ����ж�+NAPI���ƽ������ݵĽ��ա�������������û�в��������ķ��������Ǵ�
���ϵ����̱Ƚ����ơ� 
	���û�̬��ifconfig��������IP��ַ������������ʱ����ndo_open����������Ӳ���ĳ�ʼ������ndo_open�н���
�ġ�
*/
/*
mcp251x_open(struct net_device *net)
	->ret = request_threaded_irq(spi->irq, NULL, mcp251x_can_ist,
		  pdata->irq_flags ? pdata->irq_flags : IRQF_TRIGGER_FALLING,
		  DEVICE_NAME, priv);
		ע���жϴ�����mcp251x_can_ist���������жϲ���ʱ������øú�����
		mcp251x_can_ist(int irq, void *dev_id)
			->mcp251x_hw_rx(struct spi_device *spi, int buf_idx)
				->mcp251x_hw_rx_frame(struct spi_device *spi, u8 *buf,int buf_idx)
					->mcp251x_read_reg
						Ȼ���CANģ��Ľ��ռĴ����н�������
*/

//���������������ο���
//1��������Linux ����ѡ��
Networking support --->
	CAN bus subsystem support --->
		CAN Device Drivers --->
			Platform CAN drivers with Netlink support
			CAN bit-timing calculation
//2.�����ļ�mcp251x.c mcp251x.h can.h
//�ļ�mcp251x.c����Ŀ¼drivers/net/can/�£�
//�ļ�mcp251x.h����Ŀ¼include/linux/can/platform/�£�
//�ļ�can.h����Ŀ¼include/linux/can/��

//3.��������ļ�drivers/net/can/Kconfig
//���ļ������
config CAN_MCP251X
tristate "Microchip 251x series SPI CAN Controller"
depends on CAN && SPI
default N
---help---
  Say Y here if you want support for the Microchip 251x series of
  SPI based CAN controllers.

//4.��drivers/net/can/Makefile�ļ�����ӱ����ļ�
obj-$(CONFIG_CAN_MCP251X) += mcp251x.o


//Linux-c CANģ����Գ���

/*client*/
#include <string.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>

#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

int main()
{
	int s;
	unsigned long nbytes;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_frame frame;


	s = socket(PF_CAN,SOCK_RAW,CAN_RAW);

	strcpy((char *)(ifr.ifr_name),"can0");
	ioctl(s,SIOCGIFINDEX,&ifr);
	printf("can0 can_ifindex = %x\n",ifr.ifr_ifindex);


	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	bind(s,(struct sockaddr*)&addr,sizeof(addr));


	frame.can_id = 0x123;
	strcpy((char *)frame.data,"hello");
	frame.can_dlc = strlen(frame.data);

	printf("Send a CAN frame from interface %s\n",ifr.ifr_name);

	nbytes = sendto(s,&frame,sizeof(struct can_frame),0,(struct sockaddr*)&addr,sizeof(addr));
	
	return 0;
}


/*server*/
#include <string.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>

#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

int main()
{
	int s;
	unsigned long nbytes,len;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_frame frame;


	s = socket(PF_CAN,SOCK_RAW,CAN_RAW);

	strcpy(ifr.ifr_name,"can0");
	ioctl(s,SIOCGIFINDEX,&ifr);
	printf("can0 can_ifindex = %x\n",ifr.ifr_ifindex);

	//bind to all enabled can interface
	addr.can_family = AF_CAN;
	addr.can_ifindex =0;
	bind(s,(struct sockaddr*)&addr,sizeof(addr));

	nbytes = recvfrom(s,&frame,sizeof(struct can_frame),0,(struct sockaddr *)&addr,&len);
	
	/*get interface name of the received CAN frame*/
	ifr.ifr_ifindex = addr.can_ifindex;
	ioctl(s,SIOCGIFNAME,&ifr);
	printf("Received a CAN frame from interface %s\n",ifr.ifr_name);
	printf("frame message\n"
		"--can_id = %x\n"
		"--can_dlc = %x\n"
		"--data = %s\n",frame.can_id,frame.can_dlc,frame.data);

	return 0;
}





