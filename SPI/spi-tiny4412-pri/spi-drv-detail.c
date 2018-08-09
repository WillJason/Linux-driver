/*SPI是同步外设接口，由摩托罗拉公司开发的全双工同步串行总线，接口有MISO、MOSI、SCK和SS四线组成。在这里不具体介绍SPI的工作原理了，相信学SPI驱动的同学已经在单片机上实现过了SPI的通讯。

      学习SPI驱动首先必须要建立一个分层的设计思想，分层设计不光在SPI中体现，在linux内核中都是分层的思想，使得linux具有强大的适应性。分层设计思想在linux的Input、RTC、MTD、IIC、TTY、USB、SPI等很多设备驱动中得到体现。那么SPI驱动分了哪几个层呢？

      SPI在linux中分为3层分别为主机控制器驱动、核心层驱动和外设驱动：

       在OK6410中     主机控制器驱动：spi_s3c64xx.c--------------这个文件将s3c6410的硬件spi控制器驱动。

                     	核心层驱动 ：spi.c---------------------------这个文件是实现了spi的注册和注销的函数等，连接了主机控制器和外设驱动，起到了桥梁的作用。

                      外设驱动  ：spidev.c----------------------内核中的一个通用的外设驱动。



学习底层驱动免不了大量的结构体，下面就来看一些重要的结构体：

为了看起来可以有调理性，首先从主机控制器的结构体开始：

在include\linux\spi下有spi.h

*/

struct spi_master {
	struct device	dev;							//内嵌标准dev结构
	struct list_head list;	
	s16			bus_num;									//总线编号
	u16			num_chipselect;						//芯片支持的片选数量
	u16			dma_alignment;						//采用dma时的对齐要求

  /* spi_device.mode flags understood by this controller driver */
	u16			mode_bits;

	/* other constraints relevant to this driver */
	u16			flags;
	#define SPI_MASTER_HALF_DUPLEX	BIT(0)		/* can't do full duplex */
	#define SPI_MASTER_NO_RX	BIT(1)		/* can't do buffer read */
	#define SPI_MASTER_NO_TX	BIT(2)		/* can't do buffer write */

	/* lock and mutex for SPI bus locking */
	spinlock_t		bus_lock_spinlock;					//自旋锁定义
	struct mutex		bus_lock_mutex;					//互斥信号量定义

	/* flag indicating that the SPI bus is locked for exclusive use */
	bool			bus_lock_flag;

	//改变spi_device的特性如：传输模式，字长，时钟频率  
	int			(*setup)(struct spi_device *spi);		

	 /*添加消息到队列的方法，这个函数不可睡眠，他的任务是安排发生的传送并且调用注册的回调函数complete()*/ 
	int			(*transfer)(struct spi_device *spi,
						struct spi_message *mesg);

	/* called on release() to free memory provided by spi_master */
	void			(*cleanup)(struct spi_device *spi);				//释放内存
};
/*
在linux中，使用spi_master结构体来描述一个SPI的主机控制器驱动，结构体主要完成主机控制器的序列号（s3c6410中有两个spi主机控制器）、片选数量、SPI模式和时钟以及一些传输函数的实现。
上面这个结构体通过一下3个函数来注册和注销：
*/
struct spi_master *spi_alloc_master(struct device *host, unsigned size);      //自动分配内存
int spi_register_master(struct spi_master *master);                           //注册spi控制器
void spi_unregister_master(struct spi_master *master);                        //注销spi控制器
/*以上的结构体和函数最终在spi_s3c64xx.c中得到实现，有兴趣的可以看一下里面的代码很不错，但是也不好理解，因为6410的spi驱动采用了DMA传输。



SPI的核心层，作为桥梁的功能，里面主要注册了一个spi_bus_type的spi总线和spi_master的class类。通过注册在我们的系统中sys/bus中产生了一个spi的目录和sys/class下产生spi_master目录。



SPI外设驱动层，该层就像我们平常写的驱动，里面有几个重要的结构体分别是：*/

struct spi_driver {
	const struct spi_device_id *id_table;
	int			(*probe)(struct spi_device *spi);
	int			(*remove)(struct spi_device *spi);
	void			(*shutdown)(struct spi_device *spi);
	int			(*suspend)(struct spi_device *spi, pm_message_t mesg);
	int			(*resume)(struct spi_device *spi);
	struct device_driver	driver;
};
/*spi_driver这个结构体和platform_driver结构体非常的相似。
在驱动中传输使用的结构体：*/

struct spi_transfer {

	const void	*tx_buf;			//要写入设备的数据(必须是dma_safe)，或者为NULL
	void		*rx_buf;				//要读取的数据缓冲(必须是dma_safe)，或者为NULL  
	unsigned	len;					 //tx和rx的大小(字节数),这里不是指它的和，而是各自的长度，他们总是相等的  

	dma_addr_t	tx_dma;			//如果spi_message.is_dma_mapped是真，这个是tx的dma地址 
	dma_addr_t	rx_dma;			//如果spi_message.is_dma_mapped是真，这个是rx的dma地址  

	unsigned	cs_change:1;		//影响此次传输之后的片选，指示本次tranfer结束之后是否要重新片选并调用setup改变设置，这个标志可以较少系统开销u8      
	u8		bits_per_word;		 //每个字长的比特数，如果是0，使用默认值  
	u16		delay_usecs;		//此次传输结束和片选改变之间的延时，之后就会启动另一个传输或者结束整个消息  
	u32		speed_hz;			//通信时钟。如果是0，使用默认值  

	struct list_head transfer_list;	        //用来连接的双向链表节点
};
struct spi_message {
	struct list_head	transfers;			//此次消息的传输队列，一个消息可以包含多个传输段

	struct spi_device	*spi;				//传输的目的设备  

	unsigned		is_dma_mapped:1;		//如果为真，此次调用提供dma和cpu虚拟地址  

	/* completion is reported through a callback */
	void			(*complete)(void *context);		//异步调用完成后的回调函数  
	void			*context;						//回调函数的参数  
	unsigned		actual_length;					//此次传输的实际长度  
	int			status;							//执行的结果，成功被置0，否则是一个负的错误码  

	struct list_head	queue;
	void			*state;
};
/*通过下面这个函数*/
static inline void spi_message_add_tail(struct spi_transfer *t, struct spi_message *m)
//初始化spi_message,进而将spi_transfer添加到spi_message中，发起传输有两种方式分别是同步传输和异步传输。
extern int spi_sync(struct spi_device *spi, struct spi_message *message);    -------------同步传输
extern int spi_async(struct spi_device *spi, struct spi_message *message);   -------------异步传输

//下面介绍两个通用的SPI传输函数：

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
//需要添加spi的平台设备信息
static void __init smdk4x12_machine_init(void)
{
	//...
	//初始化相关平台信息

	#ifdef CONFIG_S3C64XX_DEV_SPI0
	s3c64xx_spi0_set_platdata(NULL, 0, 1);
#endif

	/*
		1、先创建一个spi_board_info结构描述spi设备信息，调用spi_register_board_info将这个结构添加到board_list中。
		2、然后调用spi_register_master注册SPI控制器驱动，此时会调用scan_boardinfo扫描board_list，根据 spi_board_info调用spi_new_device生成spi_device结构，用spi_add_device添加设备。
		3、调用spi_register_driver注册spi_driver，通过与device匹配驱动设备。
	*/
	#ifdef CONFIG_S3C64XX_DEV_SPI0
	spi_register_board_info(spi0_board_info, ARRAY_SIZE(spi0_board_info));
#endif

	//注册spi的平台设备信息

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
samsung spi-s3c64xx.c分析
-----------------------------------------------------------------------------------*/
static int __init s3c64xx_spi_probe(struct platform_device *pdev)
{
	struct resource	*mem_res;
	struct s3c64xx_spi_driver_data *sdd;
	struct s3c64xx_spi_info *sci = pdev->dev.platform_data;
	struct spi_master *master;
	int ret, irq;
	char clk_name[16];

	if (!sci && pdev->dev.of_node) {//查找设备树节点，若未使用设备树则返回dev->platform_data
		sci = s3c64xx_spi_parse_dt(&pdev->dev);
		if (IS_ERR(sci))
			return PTR_ERR(sci);
	}

	if (!sci) {
		dev_err(&pdev->dev, "platform_data missing!\n");
		return -ENODEV;
	}

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);//获取IO内存资源
	if (mem_res == NULL) {
		dev_err(&pdev->dev, "Unable to get SPI MEM resource\n");
		return -ENXIO;
	}

	irq = platform_get_irq(pdev, 0);//获取中断号
	if (irq < 0) {
		dev_warn(&pdev->dev, "Failed to get IRQ: %d\n", irq);
		return irq;
	}

	//为结构体spi_master和结构体s3c64xx_spi_driver_data分配存储空间，并将s3c64xx_spi_driver_data设为spi_master的drvdata。
	master = spi_alloc_master(&pdev->dev,
				sizeof(struct s3c64xx_spi_driver_data));
	if (master == NULL) {
		dev_err(&pdev->dev, "Unable to allocate SPI Master\n");
		return -ENOMEM;
	}

	/** 
     * platform_set_drvdata 和 platform_get_drvdata 
     * probe函数中定义的局部变量，如果我想在其他地方使用它怎么办呢？ 
     * 这就需要把它保存起来。内核提供了这个方法， 
     * 使用函数platform_set_drvdata()可以将master保存成平台总线设备的私有数据。 
     * 以后再要使用它时只需调用platform_get_drvdata()就可以了。 
     */  
	platform_set_drvdata(pdev, master);

	//从master中获得s3c64xx_spi_driver_data，并初始化相关成员  
	sdd = spi_master_get_devdata(master);
	sdd->port_conf = s3c64xx_spi_get_port_config(pdev);
	sdd->master = master;
	sdd->cntrlr_info = sci;
	sdd->pdev = pdev;
	sdd->sfr_start = mem_res->start;
	if (pdev->dev.of_node) {
		//获得与这个device_node（类似/i2c@13880000节点）对应的alias_prop的id，如果以/i2c@13880000节点为例，这里得到的id就是2。
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

	//master相关成员的初始化 
	master->dev.of_node = pdev->dev.of_node;
	master->bus_num = sdd->port_id;//总线号
	master->setup = s3c64xx_spi_setup;
	master->cleanup = s3c64xx_spi_cleanup;
	master->prepare_transfer_hardware = s3c64xx_spi_prepare_transfer;
	master->transfer_one_message = s3c64xx_spi_transfer_one_message;
	master->unprepare_transfer_hardware = s3c64xx_spi_unprepare_transfer;
	master->num_chipselect = sci->num_cs;//该总线上的设备数  
	master->dma_alignment = 8;
	/* the spi->mode bits understood by this driver: */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;

	if (request_mem_region(mem_res->start,
			resource_size(mem_res), pdev->name) == NULL) {
		dev_err(&pdev->dev, "Req mem region failed\n");
		ret = -ENXIO;
		goto err0;
	}

	sdd->regs = ioremap(mem_res->start, resource_size(mem_res));//申请IO内存 
	if (sdd->regs == NULL) {
		dev_err(&pdev->dev, "Unable to remap IO\n");
		ret = -ENXIO;
		goto err1;
	}

	//SPI的IO管脚配置，将相应的IO管脚设置为SPI功能 
	if (!sci->cfg_gpio && pdev->dev.of_node) {
		if (s3c64xx_spi_parse_dt_gpio(sdd))
			return -EBUSY;
	} else if (sci->cfg_gpio == NULL || sci->cfg_gpio()) {
		dev_err(&pdev->dev, "Unable to config gpio\n");
		ret = -EBUSY;
		goto err2;
	}

	//使能时钟  
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

	//硬件初始化，初始化设置寄存器，包括对SPIMOSI、SPIMISO、SPICLK引脚的设置 
	/* Setup Deufult Mode */
	s3c64xx_spi_hwinit(sdd, sdd->port_id);

	//锁、工作队列等初始化  
	spin_lock_init(&sdd->lock);
	init_completion(&sdd->xfer_completion);
	INIT_LIST_HEAD(&sdd->queue);

	ret = request_irq(irq, s3c64xx_spi_irq, 0, "spi-s3c64xx", sdd);//注册中断和中断处理函数
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to request IRQ %d: %d\n",
			irq, ret);
		goto err7;
	}

	writel(S3C64XX_SPI_INT_RX_OVERRUN_EN | S3C64XX_SPI_INT_RX_UNDERRUN_EN |
	       S3C64XX_SPI_INT_TX_OVERRUN_EN | S3C64XX_SPI_INT_TX_UNDERRUN_EN,
	       sdd->regs + S3C64XX_SPI_INT_EN);

	//注册spi_master到spi子系统中去
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

	master = kzalloc(size + sizeof *master, GFP_KERNEL);//分配的内存大小是*master加size，包含了两部分内存
	if (!master)
		return NULL;

	device_initialize(&master->dev);//设备模型中的初始设备函数
	master->bus_num = -1;
	master->num_chipselect = 1;
	master->dev.class = &spi_master_class;
	master->dev.parent = get_device(dev);
	spi_master_set_devdata(master, &master[1]);/*&master[1]就是master之后的另一部分内存的起始地址。
     回到s3c64xx_spi_probe函数，就是取出刚才申请的第二部分内存的起始地址。*/

	return master;
}

//spi_master_register函数用于向内核注册一个spi_master。
int spi_register_master(struct spi_master *master)  
{  
    ......
  
    /* even if it's just one always-selected device, there must 
     * be at least one chipselect 
     */  
    if (master->num_chipselect == 0) //一个SPI控制器至少有一个片选，因此片选数为0则出错  
        return -EINVAL;  
  
    /* convention:  dynamically assigned bus IDs count down from the max */  
    if (master->bus_num < 0) { //如果总线号小于0则动态分配一个总线号  
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
    dev_set_name(&master->dev, "spi%u", master->bus_num); //把master加入到设备模型中  
    status = device_add(&master->dev);  
    if (status < 0)  
        goto done;  
    dev_dbg(dev, "registered master %s%s\n", dev_name(&master->dev),  
            dynamic ? " (dynamic)" : "");  
  
    mutex_lock(&board_lock);  
    list_add_tail(&master->list, &spi_master_list);  
    list_for_each_entry(bi, &board_list, list) //遍历board_list这个链表  
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
  
    if (master->bus_num != bi->bus_num) //每找到一个成员就将它的总线号与master的总线号进行比较，如果相等则调用spi_new_device函数创建一个spi设备  
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
    strlcpy(proxy->modalias, chip->modalias, sizeof(proxy->modalias)); //此处比较关键，设备名字拷贝  
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












