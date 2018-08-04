/*
I2C设备驱动通常只是需要挂载在I2C总线（即依附于I2C子系统），I2C子系统对于设备驱动来说
只是一个载体、基石。许多设备的主要核心是建立在其他子系统上，如重力传感器、三轴传感器
触摸屏等通常主要工作集中在INPUT子系统中，而相机模块、FM模块、GPS模块大多主要依附于V4L2子
系统。这也能通过I2C设计理念证明，I2C的产生正是为了节省外围电路复杂度，让CPU使用有限的
IO口挂载更多的外部模块。假设CPU的扩展IO口足够多，我想I2C也没什么必要存在了，毕竟直接操
作IO口驱动设备比I2C来的更简单。

I2C adapter
是CPU集成或外接的I2C适配器，用来控制各种I2C从设备，其驱动需要完成对适配器的完整描述，最主
要的工作是需要完成i2c_algorithm结构体。

I2C driver
具体的I2C设备驱动

I2C client
即I2C设备。I2C设备的注册一般在板级代码中


*/
//需要添加i2c的平台设备信息
static void __init smdk4x12_machine_init(void)
{
	//...
	//初始化相关平台信息

	s3c_i2c0_set_platdata(&tiny4412_i2c0_data);
	/*
		i2c_register_board_info()函数用于往__i2c_board_list这条链表添加一条i2c设备
		信息，在i2c adapter注册的时候，会扫描__i2c_board_list链表，然后调用i2c_new_device()
		函数来注册i2c设备,真实的I2C设备是在适配器成功注册后才被生成的.注意，要先于i2c adapter
		注册之前就添加好i2c设备信息，否则会出现调用了i2c_register_board_info()函数，而设备不能
		注册的情况。
	*/
	i2c_register_board_info(0, smdk4x12_i2c_devs0,
			ARRAY_SIZE(smdk4x12_i2c_devs0));

	//注册i2c的平台设备信息

	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
	//...
}


static struct s3c2410_platform_i2c tiny4412_i2c0_data __initdata = {
	.flags			= 0,
	.bus_num		= 0,
	.slave_addr		= 0x10,
	.frequency		= 200*1000,
	.sda_delay		= 100,
};

static struct i2c_board_info smdk4x12_i2c_devs0[] __initdata = {
#ifdef CONFIG_SND_SOC_WM8960_TINY4412
	{
		I2C_BOARD_INFO("wm8960", 0x1a),
		.platform_data = &wm8960_pdata,
	},
#endif

void __init s3c_i2c0_set_platdata(struct s3c2410_platform_i2c *pd)
{
	struct s3c2410_platform_i2c *npd;

	if (!pd) {
		pd = &default_i2c_data;
		pd->bus_num = 0;
	}

	npd = s3c_set_platdata(pd, sizeof(struct s3c2410_platform_i2c),
			       &s3c_device_i2c0);

	if (!npd->cfg_gpio)
		npd->cfg_gpio = s3c_i2c0_cfg_gpio;
}

/* I2C */

static struct resource s3c_i2c0_resource[] = {
	[0] = DEFINE_RES_MEM(S3C_PA_IIC, SZ_4K),
	[1] = DEFINE_RES_IRQ(IRQ_IIC),
};

struct platform_device s3c_device_i2c0 = {
	.name		= "s3c2410-i2c",
#ifdef CONFIG_S3C_DEV_I2C1
	.id		= 0,
#else
	.id		= -1,
#endif
	.num_resources	= ARRAY_SIZE(s3c_i2c0_resource),
	.resource	= s3c_i2c0_resource,
};

/*-----------------------------------------------------------------------------------
samsung i2c-s3c2410.c分析
-----------------------------------------------------------------------------------*/
static int s3c24xx_i2c_probe(struct platform_device *pdev)
{
	struct s3c24xx_i2c *i2c;
	struct s3c2410_platform_i2c *pdata = NULL;
	struct resource *res;
	int ret;

	if (!pdev->dev.of_node) {
		pdata = pdev->dev.platform_data;//获取platform_data 找一个和平台相关的驱动程序
		if (!pdata) {
			dev_err(&pdev->dev, "no platform data\n");
			return -EINVAL;
		}
	}

	//为结构体i2c分配内存空间并清零。
	i2c = devm_kzalloc(&pdev->dev, sizeof(struct s3c24xx_i2c), GFP_KERNEL);
	if (!i2c) {
		dev_err(&pdev->dev, "no memory for state\n");
		return -ENOMEM;
	}

	i2c->pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!i2c->pdata) {
		dev_err(&pdev->dev, "no memory for platform data\n");
		return -ENOMEM;
	}

	i2c->quirks = s3c24xx_get_device_quirks(pdev);
	if (pdata)
		memcpy(i2c->pdata, pdata, sizeof(*pdata));
	else
		s3c24xx_i2c_parse_dt(pdev->dev.of_node, i2c);

	//这整个驱动程序就是在向内核加一个适配器
	strlcpy(i2c->adap.name, "s3c2410-i2c", sizeof(i2c->adap.name));
	i2c->adap.owner   = THIS_MODULE;
	i2c->adap.algo    = &s3c24xx_i2c_algorithm;//适配器操作方法
	i2c->adap.retries = 2;//数据传输异常打断后，重复尝试的次数
	i2c->adap.class   = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	i2c->tx_setup     = 50;//数据写入IICDS后延时时间

	init_waitqueue_head(&i2c->wait);

	/* find the clock and enable it */

	i2c->dev = &pdev->dev;
	i2c->clk = clk_get(&pdev->dev, "i2c");//获取平台时钟
	if (IS_ERR(i2c->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		ret = -ENOENT;
		goto err_noclk;
	}

	dev_dbg(&pdev->dev, "clock source %p\n", i2c->clk);

	clk_enable(i2c->clk);//使能时钟

	/* map the registers */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);//获取I/O资源
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot find IO resource\n");
		ret = -ENOENT;
		goto err_clk;
	}

	i2c->ioarea = request_mem_region(res->start, resource_size(res),
					 pdev->name);//申请IO内存

	if (i2c->ioarea == NULL) {
		dev_err(&pdev->dev, "cannot request IO\n");
		ret = -ENXIO;
		goto err_clk;
	}

	i2c->regs = ioremap(res->start, resource_size(res));//IO映射

	if (i2c->regs == NULL) {
		dev_err(&pdev->dev, "cannot map IO\n");
		ret = -ENXIO;
		goto err_ioarea;
	}

	dev_dbg(&pdev->dev, "registers %p (%p, %p)\n",
		i2c->regs, i2c->ioarea, res);

	/* setup info block for the i2c core */
//将i2c结构体设为adap的私有数据成员
	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &pdev->dev;

	/* inititalise the i2c gpio lines */

	if (i2c->pdata->cfg_gpio) {
		i2c->pdata->cfg_gpio(to_platform_device(i2c->dev));
	} else if (s3c24xx_i2c_parse_dt_gpio(i2c)) {
		ret = -EINVAL;
		goto err_iomap;
	}

	/* initialise the i2c controller */

	ret = s3c24xx_i2c_init(i2c);
	if (ret != 0) {
		dev_err(&pdev->dev, "I2C controller init failed\n");
		goto err_iomap;
	}

	/* find the IRQ for this unit (note, this relies on the init call to
	 * ensure no current IRQs pending
	 */

	i2c->irq = ret = platform_get_irq(pdev, 0);
	if (ret <= 0) {
		dev_err(&pdev->dev, "cannot find IRQ\n");
		goto err_iomap;
	}

	//申请一个中断并关联它的处理函数，IRQF_DISABLED该中断不被共享（我认为）
	ret = request_irq(i2c->irq, s3c24xx_i2c_irq, 0,
			  dev_name(&pdev->dev), i2c);

	if (ret != 0) {
		dev_err(&pdev->dev, "cannot claim IRQ %d\n", i2c->irq);
		goto err_iomap;
	}
	
	//将通知节点注册到通知链。
	ret = s3c24xx_i2c_register_cpufreq(i2c);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register cpufreq notifier\n");
		goto err_irq;
	}

	/* Note, previous versions of the driver used i2c_add_adapter()
	 * to add the bus at any number. We now pass the bus number via
	 * the platform data, so if unset it will now default to always
	 * being bus 0.
	 */

	i2c->adap.nr = i2c->pdata->bus_num;//设置总线号
	i2c->adap.dev.of_node = pdev->dev.of_node;

	//特别重要的一个函数，添加一个适配器设备
	/*
	i2c_add_numbered_adapter()->i2c_register_adapter()
	在i2c_register_adapter()函数中调用了i2c_scan_static_board_info()函数
	i2c_scan_static_board_info()函数先扫描__i2c_board_list，然后调用i2c_new_device()
	函数来注册i2c设备。这就是静态注册i2c设备的方法，一般在板级的初始化函数中调用
	i2c_register_board_info()函数来完成i2c设备信息的添加。
	i2c_new_device()将会详细介绍
	*/
	
	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add bus to i2c core\n");
		goto err_cpufreq;
	}

	of_i2c_register_devices(&i2c->adap);
	platform_set_drvdata(pdev, i2c);//将i2c设为pdev的drvdata。

	pm_runtime_enable(&pdev->dev);
	pm_runtime_enable(&i2c->adap.dev);

	dev_info(&pdev->dev, "%s: S3C I2C adapter\n", dev_name(&i2c->adap.dev));
	clk_disable(i2c->clk);
	return 0;

 err_cpufreq:
	s3c24xx_i2c_deregister_cpufreq(i2c);

 err_irq:
	free_irq(i2c->irq, i2c);

 err_iomap:
	if (i2c->gpios[0])
		s3c24xx_i2c_dt_gpio_free(i2c);

	iounmap(i2c->regs);

 err_ioarea:
	release_resource(i2c->ioarea);
	kfree(i2c->ioarea);

 err_clk:
	clk_disable(i2c->clk);
	clk_put(i2c->clk);

 err_noclk:
	return ret;
}

/*
i2d_new_device没什么好多说的，由于有i2c_register_board_info的铺垫，相信也很好理解了。
而i2c_new_device不但印证了i2c_client与i2c_board_info的对应关系，还顺便体现了10bit地址
设备与7bit地址设备的略微不同。通过两者的对比，可以再总结出几点区别，从而更好的理解I2C
设备的注册方法：

i2c_register_board_info的形参需要的是总线号
i2c_new_device的形参需要的直接是适配器的指针
我想这也正好能完美的说明两者的根本区别，对于板级设备更在乎适配器的总线号，因为这是固定的，
没有异议的。而对于可插拔设备，因为其适配器可能非板级集成的，所以不能在乎其总线号，反而只要
寻求其适配器指针进行绑定即可。后边adapter注册的分析能更好的证明这一点。

i2c_register_board_info可以同时注册多个I2C设备
i2c_new_device只能一次注册一个I2C设备
这也是其根本区别决定的，板级代码中常包含有许多I2C设备，所以i2c_register_board_info需要有同时
注册多个I2C设备的能力也可以说是刚需。而i2c_new_device既然是用来给可插拔设备用的，想必设备数量
并不多，而常可能只是一个两个而已，所以一次一个就够了，需求量并不大。
*/
struct i2c_client *i2c_new_device(struct i2c_adapter *adap, struct i2c_board_info const *info)
{
    struct i2c_client    *client;
    int            status;

    client = kzalloc(sizeof *client, GFP_KERNEL);  //为即将注册的client申请内存
    if (!client)
        return NULL;

    client->adapter = adap;  //绑定指定的adapter适配器

    client->dev.platform_data = info->platform_data;  //保存设备数据

    if (info->archdata)  //代码上看是DMA相关操作数据
        client->dev.archdata = *info->archdata;

    client->flags = info->flags;  //类型，（一）中说过，或是10位地址，或是使用SMBus检错
    client->addr = info->addr;  //设备从地址
    client->irq = info->irq;  //设备终端

    strlcpy(client->name, info->type, sizeof(client->name));  //从设备名
            //瞧！（一）中说过i2c_board_info中的信息是与i2c_client有对应关系的，灵验了吧！

    /* Check for address validity */
    status = i2c_check_client_addr_validity(client);  //检测地址是否有效，10位地址是否大于0x3ff，7位地址是否大于0x7f或为0
    if (status) {  //非零（实际上为-22，无效参数Invalid argument
        dev_err(&adap->dev, "Invalid %d-bit I2C address 0x%02hx\n",
            client->flags & I2C_CLIENT_TEN ? 10 : 7, client->addr);
        goto out_err_silent;
    }

    /* Check for address business */
    status = i2c_check_addr_busy(adap, client->addr);  //检测指定适配器上该地址状态
    if (status)
        goto out_err;

    client->dev.parent = &client->adapter->dev;  //建立从设备与适配器的父子关系
    client->dev.bus = &i2c_bus_type;
    client->dev.type = &i2c_client_type;
    client->dev.of_node = info->of_node;
    ACPI_HANDLE_SET(&client->dev, info->acpi_node.handle);

    /* For 10-bit clients, add an arbitrary offset to avoid collisions */
    dev_set_name(&client->dev, "%d-%04x", i2c_adapter_id(adap),
             client->addr | ((client->flags & I2C_CLIENT_TEN)
                     ? 0xa000 : 0));  //如果是10位地址设备，那名字格式与7bit的会有不同
    status = device_register(&client->dev);  //注册了！注册了！！！
    if (status)
        goto out_err;

    dev_dbg(&adap->dev, "client [%s] registered with bus id %s\n",
        client->name, dev_name(&client->dev));

    return client;

out_err:
    dev_err(&adap->dev, "Failed to register i2c client %s at 0x%02x "
        "(%d)\n", client->name, client->addr, status);
out_err_silent:
    kfree(client);
    return NULL;
}





















