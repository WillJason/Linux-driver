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













