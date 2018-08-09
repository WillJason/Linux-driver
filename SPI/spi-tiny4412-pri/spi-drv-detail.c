/*SPI��ͬ������ӿڣ���Ħ��������˾������ȫ˫��ͬ���������ߣ��ӿ���MISO��MOSI��SCK��SS������ɡ������ﲻ�������SPI�Ĺ���ԭ���ˣ�����ѧSPI������ͬѧ�Ѿ��ڵ�Ƭ����ʵ�ֹ���SPI��ͨѶ��

      ѧϰSPI�������ȱ���Ҫ����һ���ֲ�����˼�룬�ֲ���Ʋ�����SPI�����֣���linux�ں��ж��Ƿֲ��˼�룬ʹ��linux����ǿ�����Ӧ�ԡ��ֲ����˼����linux��Input��RTC��MTD��IIC��TTY��USB��SPI�Ⱥܶ��豸�����еõ����֡���ôSPI���������ļ������أ�

      SPI��linux�з�Ϊ3��ֱ�Ϊ�������������������Ĳ�����������������

       ��OK6410��     ����������������spi_s3c64xx.c--------------����ļ���s3c6410��Ӳ��spi������������

                     	���Ĳ����� ��spi.c---------------------------����ļ���ʵ����spi��ע���ע���ĺ����ȣ������������������������������������������á�

                      ��������  ��spidev.c----------------------�ں��е�һ��ͨ�õ�����������



ѧϰ�ײ������ⲻ�˴����Ľṹ�壬���������һЩ��Ҫ�Ľṹ�壺

Ϊ�˿����������е����ԣ����ȴ������������Ľṹ�忪ʼ��

��include\linux\spi����spi.h

*/

struct spi_master {
	struct device	dev;							//��Ƕ��׼dev�ṹ
	struct list_head list;	
	s16			bus_num;									//���߱��
	u16			num_chipselect;						//оƬ֧�ֵ�Ƭѡ����
	u16			dma_alignment;						//����dmaʱ�Ķ���Ҫ��

  /* spi_device.mode flags understood by this controller driver */
	u16			mode_bits;

	/* other constraints relevant to this driver */
	u16			flags;
	#define SPI_MASTER_HALF_DUPLEX	BIT(0)		/* can't do full duplex */
	#define SPI_MASTER_NO_RX	BIT(1)		/* can't do buffer read */
	#define SPI_MASTER_NO_TX	BIT(2)		/* can't do buffer write */

	/* lock and mutex for SPI bus locking */
	spinlock_t		bus_lock_spinlock;					//����������
	struct mutex		bus_lock_mutex;					//�����ź�������

	/* flag indicating that the SPI bus is locked for exclusive use */
	bool			bus_lock_flag;

	//�ı�spi_device�������磺����ģʽ���ֳ���ʱ��Ƶ��  
	int			(*setup)(struct spi_device *spi);		

	 /*�����Ϣ�����еķ����������������˯�ߣ����������ǰ��ŷ����Ĵ��Ͳ��ҵ���ע��Ļص�����complete()*/ 
	int			(*transfer)(struct spi_device *spi,
						struct spi_message *mesg);

	/* called on release() to free memory provided by spi_master */
	void			(*cleanup)(struct spi_device *spi);				//�ͷ��ڴ�
};
/*
��linux�У�ʹ��spi_master�ṹ��������һ��SPI�������������������ṹ����Ҫ������������������кţ�s3c6410��������spi��������������Ƭѡ������SPIģʽ��ʱ���Լ�һЩ���亯����ʵ�֡�
��������ṹ��ͨ��һ��3��������ע���ע����
*/
struct spi_master *spi_alloc_master(struct device *host, unsigned size);      //�Զ������ڴ�
int spi_register_master(struct spi_master *master);                           //ע��spi������
void spi_unregister_master(struct spi_master *master);                        //ע��spi������
/*���ϵĽṹ��ͺ���������spi_s3c64xx.c�еõ�ʵ�֣�����Ȥ�Ŀ��Կ�һ������Ĵ���ܲ�������Ҳ������⣬��Ϊ6410��spi����������DMA���䡣



SPI�ĺ��Ĳ㣬��Ϊ�����Ĺ��ܣ�������Ҫע����һ��spi_bus_type��spi���ߺ�spi_master��class�ࡣͨ��ע�������ǵ�ϵͳ��sys/bus�в�����һ��spi��Ŀ¼��sys/class�²���spi_masterĿ¼��



SPI���������㣬�ò��������ƽ��д�������������м�����Ҫ�Ľṹ��ֱ��ǣ�*/

struct spi_driver {
	const struct spi_device_id *id_table;
	int			(*probe)(struct spi_device *spi);
	int			(*remove)(struct spi_device *spi);
	void			(*shutdown)(struct spi_device *spi);
	int			(*suspend)(struct spi_device *spi, pm_message_t mesg);
	int			(*resume)(struct spi_device *spi);
	struct device_driver	driver;
};
/*spi_driver����ṹ���platform_driver�ṹ��ǳ������ơ�
�������д���ʹ�õĽṹ�壺*/

struct spi_transfer {

	const void	*tx_buf;			//Ҫд���豸������(������dma_safe)������ΪNULL
	void		*rx_buf;				//Ҫ��ȡ�����ݻ���(������dma_safe)������ΪNULL  
	unsigned	len;					 //tx��rx�Ĵ�С(�ֽ���),���ﲻ��ָ���ĺͣ����Ǹ��Եĳ��ȣ�����������ȵ�  

	dma_addr_t	tx_dma;			//���spi_message.is_dma_mapped���棬�����tx��dma��ַ 
	dma_addr_t	rx_dma;			//���spi_message.is_dma_mapped���棬�����rx��dma��ַ  

	unsigned	cs_change:1;		//Ӱ��˴δ���֮���Ƭѡ��ָʾ����tranfer����֮���Ƿ�Ҫ����Ƭѡ������setup�ı����ã������־���Խ���ϵͳ����u8      
	u8		bits_per_word;		 //ÿ���ֳ��ı������������0��ʹ��Ĭ��ֵ  
	u16		delay_usecs;		//�˴δ��������Ƭѡ�ı�֮�����ʱ��֮��ͻ�������һ��������߽���������Ϣ  
	u32		speed_hz;			//ͨ��ʱ�ӡ������0��ʹ��Ĭ��ֵ  

	struct list_head transfer_list;	        //�������ӵ�˫������ڵ�
};
struct spi_message {
	struct list_head	transfers;			//�˴���Ϣ�Ĵ�����У�һ����Ϣ���԰�����������

	struct spi_device	*spi;				//�����Ŀ���豸  

	unsigned		is_dma_mapped:1;		//���Ϊ�棬�˴ε����ṩdma��cpu�����ַ  

	/* completion is reported through a callback */
	void			(*complete)(void *context);		//�첽������ɺ�Ļص�����  
	void			*context;						//�ص������Ĳ���  
	unsigned		actual_length;					//�˴δ����ʵ�ʳ���  
	int			status;							//ִ�еĽ�����ɹ�����0��������һ�����Ĵ�����  

	struct list_head	queue;
	void			*state;
};
/*ͨ�������������*/
static inline void spi_message_add_tail(struct spi_transfer *t, struct spi_message *m)
//��ʼ��spi_message,������spi_transfer��ӵ�spi_message�У������������ַ�ʽ�ֱ���ͬ��������첽���䡣
extern int spi_sync(struct spi_device *spi, struct spi_message *message);    -------------ͬ������
extern int spi_async(struct spi_device *spi, struct spi_message *message);   -------------�첽����

//�����������ͨ�õ�SPI���亯����

static inline ssize_t spidev_sync_write(struct spidev_data *spidev, size_t len)
{
	struct spi_transfer	t = {
			.tx_buf		= spidev->buffer,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(spidev, &m);
}

static inline ssize_t spidev_sync_read(struct spidev_data *spidev, size_t len)
{
	struct spi_transfer	t = {
			.rx_buf		= spidev->buffer,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(spidev, &m);
}
//===============================================================================================
//��Ҫ���spi��ƽ̨�豸��Ϣ
static void __init smdk4x12_machine_init(void)
{
	//...
	//��ʼ�����ƽ̨��Ϣ

	#ifdef CONFIG_S3C64XX_DEV_SPI0
	s3c64xx_spi0_set_platdata(NULL, 0, 1);
#endif

	/*
		1���ȴ���һ��spi_board_info�ṹ����spi�豸��Ϣ������spi_register_board_info������ṹ��ӵ�board_list�С�
		2��Ȼ�����spi_register_masterע��SPI��������������ʱ�����scan_boardinfoɨ��board_list������ spi_board_info����spi_new_device����spi_device�ṹ����spi_add_device����豸��
		3������spi_register_driverע��spi_driver��ͨ����deviceƥ�������豸��
	*/
	#ifdef CONFIG_S3C64XX_DEV_SPI0
	spi_register_board_info(spi0_board_info, ARRAY_SIZE(spi0_board_info));
#endif

	//ע��spi��ƽ̨�豸��Ϣ

	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
	//...
}
static struct spi_board_info __initdata spi0_board_info[] = {
	{
		.modalias	= "ads7846",
		.max_speed_hz	= 3250000,
		.chip_select	= 0,
		.bus_num	= 0,
		.platform_data	= &ads7846_data,
		.mode		= SPI_MODE_0,
	},
};

void __init s3c64xx_spi0_set_platdata(int (*cfg_gpio)(void), int src_clk_nr,
						int num_cs)
{
	struct s3c64xx_spi_info pd;

	/* Reject invalid configuration */
	if (!num_cs || src_clk_nr < 0) {
		pr_err("%s: Invalid SPI configuration\n", __func__);
		return;
	}

	pd.num_cs = num_cs;
	pd.src_clk_nr = src_clk_nr;
	pd.cfg_gpio = (cfg_gpio) ? cfg_gpio : s3c64xx_spi0_cfg_gpio;

	s3c_set_platdata(&pd, sizeof(pd), &s3c64xx_device_spi0);
}

struct platform_device s3c64xx_device_spi0 = {
	.name		= "exynos4210-spi",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(s3c64xx_spi0_resource),
	.resource	= s3c64xx_spi0_resource,
	.dev = {
		.dma_mask		= &samsung_device_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
};

static struct resource s3c64xx_spi0_resource[] = {
	[0] = DEFINE_RES_MEM(S3C_PA_SPI0, SZ_256),
	[1] = DEFINE_RES_DMA(DMACH_SPI0_TX),
	[2] = DEFINE_RES_DMA(DMACH_SPI0_RX),
	[3] = DEFINE_RES_IRQ(IRQ_SPI0),
};

/*-----------------------------------------------------------------------------------
samsung spi-s3c64xx.c����
-----------------------------------------------------------------------------------*/
static int __init s3c64xx_spi_probe(struct platform_device *pdev)
{
	struct resource	*mem_res;
	struct s3c64xx_spi_driver_data *sdd;
	struct s3c64xx_spi_info *sci = pdev->dev.platform_data;
	struct spi_master *master;
	int ret, irq;
	char clk_name[16];

	if (!sci && pdev->dev.of_node) {//�����豸���ڵ㣬��δʹ���豸���򷵻�dev->platform_data
		sci = s3c64xx_spi_parse_dt(&pdev->dev);
		if (IS_ERR(sci))
			return PTR_ERR(sci);
	}

	if (!sci) {
		dev_err(&pdev->dev, "platform_data missing!\n");
		return -ENODEV;
	}

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);//��ȡIO�ڴ���Դ
	if (mem_res == NULL) {
		dev_err(&pdev->dev, "Unable to get SPI MEM resource\n");
		return -ENXIO;
	}

	irq = platform_get_irq(pdev, 0);//��ȡ�жϺ�
	if (irq < 0) {
		dev_warn(&pdev->dev, "Failed to get IRQ: %d\n", irq);
		return irq;
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
	sdd->port_conf = s3c64xx_spi_get_port_config(pdev);
	sdd->master = master;
	sdd->cntrlr_info = sci;
	sdd->pdev = pdev;
	sdd->sfr_start = mem_res->start;
	if (pdev->dev.of_node) {
		//��������device_node������/i2c@13880000�ڵ㣩��Ӧ��alias_prop��id�������/i2c@13880000�ڵ�Ϊ��������õ���id����2��
		ret = of_alias_get_id(pdev->dev.of_node, "spi");
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to get alias id, "
						"errno %d\n", ret);
			goto err0;
		}
		sdd->port_id = ret;
	} else {
		sdd->port_id = pdev->id;
	}

	sdd->cur_bpw = 8;

	ret = s3c64xx_spi_get_dmares(sdd, true);
	if (ret)
		goto err0;

	ret = s3c64xx_spi_get_dmares(sdd, false);
	if (ret)
		goto err0;

	//master��س�Ա�ĳ�ʼ�� 
	master->dev.of_node = pdev->dev.of_node;
	master->bus_num = sdd->port_id;//���ߺ�
	master->setup = s3c64xx_spi_setup;
	master->cleanup = s3c64xx_spi_cleanup;
	master->prepare_transfer_hardware = s3c64xx_spi_prepare_transfer;
	master->transfer_one_message = s3c64xx_spi_transfer_one_message;
	master->unprepare_transfer_hardware = s3c64xx_spi_unprepare_transfer;
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
	if (!sci->cfg_gpio && pdev->dev.of_node) {
		if (s3c64xx_spi_parse_dt_gpio(sdd))
			return -EBUSY;
	} else if (sci->cfg_gpio == NULL || sci->cfg_gpio()) {
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

	sprintf(clk_name, "spi_busclk%d", sci->src_clk_nr);
	sdd->src_clk = clk_get(&pdev->dev, clk_name);
	if (IS_ERR(sdd->src_clk)) {
		dev_err(&pdev->dev,
			"Unable to acquire clock '%s'\n", clk_name);
		ret = PTR_ERR(sdd->src_clk);
		goto err5;
	}

	if (clk_enable(sdd->src_clk)) {
		dev_err(&pdev->dev, "Couldn't enable clock '%s'\n", clk_name);
		ret = -EBUSY;
		goto err6;
	}

	//Ӳ����ʼ������ʼ�����üĴ�����������SPIMOSI��SPIMISO��SPICLK���ŵ����� 
	/* Setup Deufult Mode */
	s3c64xx_spi_hwinit(sdd, sdd->port_id);

	//�����������еȳ�ʼ��  
	spin_lock_init(&sdd->lock);
	init_completion(&sdd->xfer_completion);
	INIT_LIST_HEAD(&sdd->queue);

	ret = request_irq(irq, s3c64xx_spi_irq, 0, "spi-s3c64xx", sdd);//ע���жϺ��жϴ�����
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to request IRQ %d: %d\n",
			irq, ret);
		goto err7;
	}

	writel(S3C64XX_SPI_INT_RX_OVERRUN_EN | S3C64XX_SPI_INT_RX_UNDERRUN_EN |
	       S3C64XX_SPI_INT_TX_OVERRUN_EN | S3C64XX_SPI_INT_TX_UNDERRUN_EN,
	       sdd->regs + S3C64XX_SPI_INT_EN);

	//ע��spi_master��spi��ϵͳ��ȥ
	if (spi_register_master(master)) {
		dev_err(&pdev->dev, "cannot register SPI master\n");
		ret = -EBUSY;
		goto err8;
	}

	dev_dbg(&pdev->dev, "Samsung SoC SPI Driver loaded for Bus SPI-%d "
					"with %d Slaves attached\n",
					sdd->port_id, master->num_chipselect);
	dev_dbg(&pdev->dev, "\tIOmem=[0x%x-0x%x]\tDMA=[Rx-%d, Tx-%d]\n",
					mem_res->end, mem_res->start,
					sdd->rx_dma.dmach, sdd->tx_dma.dmach);

	pm_runtime_enable(&pdev->dev);

	return 0;

err8:
	free_irq(irq, sdd);
err7:
	clk_disable(sdd->src_clk);
err6:
	clk_put(sdd->src_clk);
err5:
	clk_disable(sdd->clk);
err4:
	clk_put(sdd->clk);
err3:
	if (!sdd->cntrlr_info->cfg_gpio && pdev->dev.of_node)
		s3c64xx_spi_dt_gpio_free(sdd);
err2:
	iounmap((void *) sdd->regs);
err1:
	release_mem_region(mem_res->start, resource_size(mem_res));
err0:
	platform_set_drvdata(pdev, NULL);
	spi_master_put(master);

	return ret;
}



struct spi_master *spi_alloc_master(struct device *dev, unsigned size)
{
	struct spi_master	*master;

	if (!dev)
		return NULL;

	master = kzalloc(size + sizeof *master, GFP_KERNEL);//������ڴ��С��*master��size���������������ڴ�
	if (!master)
		return NULL;

	device_initialize(&master->dev);//�豸ģ���еĳ�ʼ�豸����
	master->bus_num = -1;
	master->num_chipselect = 1;
	master->dev.class = &spi_master_class;
	master->dev.parent = get_device(dev);
	spi_master_set_devdata(master, &master[1]);/*&master[1]����master֮�����һ�����ڴ����ʼ��ַ��
     �ص�s3c64xx_spi_probe����������ȡ���ղ�����ĵڶ������ڴ����ʼ��ַ��*/

	return master;
}

//spi_master_register�����������ں�ע��һ��spi_master��
int spi_register_master(struct spi_master *master)  
{  
    ......
  
    /* even if it's just one always-selected device, there must 
     * be at least one chipselect 
     */  
    if (master->num_chipselect == 0) //һ��SPI������������һ��Ƭѡ�����Ƭѡ��Ϊ0�����  
        return -EINVAL;  
  
    /* convention:  dynamically assigned bus IDs count down from the max */  
    if (master->bus_num < 0) { //������ߺ�С��0��̬����һ�����ߺ�  
        /* FIXME switch to an IDR based scheme, something like 
         * I2C now uses, so we can't run out of "dynamic" IDs 
         */  
        master->bus_num = atomic_dec_return(&dyn_bus_id);  
        dynamic = 1;  
    }  
  
    ......
    /* register the device, then userspace will see it. 
     * registration fails if the bus ID is in use. 
     */  
    dev_set_name(&master->dev, "spi%u", master->bus_num); //��master���뵽�豸ģ����  
    status = device_add(&master->dev);  
    if (status < 0)  
        goto done;  
    dev_dbg(dev, "registered master %s%s\n", dev_name(&master->dev),  
            dynamic ? " (dynamic)" : "");  
  
    mutex_lock(&board_lock);  
    list_add_tail(&master->list, &spi_master_list);  
    list_for_each_entry(bi, &board_list, list) //����board_list�������  
        spi_match_master_to_boardinfo(master, &bi->board_info);  
    mutex_unlock(&board_lock);  
  
    status = 0;  
  
    /* Register devices from the device tree */  
    of_register_spi_devices(master);  
done:  
    return status;  
}  
//spi_match_master_to_boardinfo(kernel3.0.15/drivers/spi/spi.c)
static void spi_match_master_to_boardinfo(struct spi_master *master,  
                struct spi_board_info *bi)  
{  
    struct spi_device *dev;  
  
    if (master->bus_num != bi->bus_num) //ÿ�ҵ�һ����Ա�ͽ��������ߺ���master�����ߺŽ��бȽϣ������������spi_new_device��������һ��spi�豸  
        return;  
  
    dev = spi_new_device(master, bi);  
    if (!dev)  
        dev_err(master->dev.parent, "can't create new device for %s\n",  
            bi->modalias);  
}  
//spi_new_device(kernel3.0.15/drivers/spi/spi.c)
struct spi_device *spi_new_device(struct spi_master *master,  
                  struct spi_board_info *chip)  
{  
    struct spi_device   *proxy;  
    int         status;  
  
    /* NOTE:  caller did any chip->bus_num checks necessary. 
     * 
     * Also, unless we change the return value convention to use 
     * error-or-pointer (not NULL-or-pointer), troubleshootability 
     * suggests syslogged diagnostics are best here (ugh). 
     */  
  
    proxy = spi_alloc_device(master);  
    if (!proxy)  
        return NULL;  
  
    WARN_ON(strlen(chip->modalias) >= sizeof(proxy->modalias));  
  
    proxy->chip_select = chip->chip_select;  
    proxy->max_speed_hz = chip->max_speed_hz;  
    proxy->mode = chip->mode;  
    proxy->irq = chip->irq;  
    strlcpy(proxy->modalias, chip->modalias, sizeof(proxy->modalias)); //�˴��ȽϹؼ����豸���ֿ���  
    proxy->dev.platform_data = (void *) chip->platform_data;  
    proxy->controller_data = chip->controller_data;  
    proxy->controller_state = NULL;  
  
    status = spi_add_device(proxy);  
    if (status < 0) {  
        spi_dev_put(proxy);  
        return NULL;  
    }  
  
    return proxy;  
}  












