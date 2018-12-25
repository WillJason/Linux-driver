/*
	TSC2007 是美国 TI 公司推出的新一代 4 线制 触摸屏 控制器，它在与触摸屏连接后，一旦有笔或手指触
摸在屏上时，便可以迅速得到该点的位置信号，从而达到在触摸屏表面上寻址的目的。
　　TSC2007 是典型的逐步逼近寄存器型 A ／ D 变换器，其结构以电容再分布为基础，同时内部包含有取
样／保持功能。 TSC2007 具有片内温度测量、触摸压力测量和预处理三项功能。带有 I 2C 接口，能以标准模式、高速模式和超高速模式进行数据传输与通讯；具有可编程 8 位或 12 位分辨率；
德州仪器(TI)公司的ADS7843/45/46、TSC2046及TSC2003/4/5/6/7型触摸屏控制器(TSC)具有触摸屏输入引脚和一个或多个非触摸屏或辅助模拟输入引脚，诸如TSC2046的电池电压监测(VBAT)或TSC2007的AUX等辅助输入引脚。在触摸屏输入工作的同时，或在未触碰触摸屏期间，可借助于这些辅助输入引脚监测系统的电池电压或其他电压信号。
由于在触摸屏被点击之后，一般都需要确定所点击点的 X 、 Y 坐标参数，以备系统处理并发送相应的消息。为此，设计时就需要对 TSC2007 进行读写操作。
*/
/*
  I2C的驱动需要根据具体的ARM芯片，一般来说，IC原厂，一般会将在linux的bsp中都会有I2C的驱动，这
个部分不需要我们去写的，我们只需要将FT5X06和BSP包中的I2C驱动匹配起来就好了。
*/
//内核配置
Device Drivers --->
	Input device support--->
		Touchscreens--->
			TSC2007 based touchscreens
//需要添加i2c的平台设备信息			
static void __init smdk4x12_machine_init(void)
{
	s3c_i2c7_set_platdata(NULL);
	i2c_register_board_info(7, i2c_devs7, ARRAY_SIZE(i2c_devs7));
	
	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
}

#ifdef CONFIG_TOUCHSCREEN_TSC2007
#define GPIO_TSC_PORT EXYNOS4_GPX0(0)
static int ts_get_pendown_state(void)
{
	int val;

	val = gpio_get_value(GPIO_TSC_PORT);

	return !val;
}

static int ts_init(void)
{
	int err;

	err = gpio_request_one(EXYNOS4_GPX0(0), GPIOF_IN, "TSC2007_IRQ");
	if (err) {
		printk(KERN_ERR "failed to request TSC2007_IRQ pin\n");
		return -1;
	}

	s3c_gpio_cfgpin(EXYNOS4_GPX0(0), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS4_GPX0(0), S3C_GPIO_PULL_NONE);
	gpio_free(EXYNOS4_GPX0(0));

	return 0;
}

static struct tsc2007_platform_data tsc2007_info = {
	.model			= 2007,
	.x_plate_ohms		= 180,
	.get_pendown_state	= ts_get_pendown_state,
	.init_platform_hw	= ts_init,
};
#endif

/* I2C7 */
static struct i2c_board_info i2c_devs7[] __initdata = {

#if defined(CONFIG_CPU_TYPE_SCP_ELITE) || defined(CONFIG_CPU_TYPE_POP_ELITE) || defined(CONFIG_CPU_TYPE_POP2G_ELITE)
	/* add by cym 20130417 for TSC2007 TouchScreen */
#ifdef CONFIG_TOUCHSCREEN_TSC2007
	{
		I2C_BOARD_INFO("tsc2007", 0x48),
		.type		= "tsc2007",
		.platform_data	= &tsc2007_info,
		.irq = IRQ_EINT(0),
         }
#endif
	/* end add */
#endif

};
static struct platform_device *smdk4x12_devices[] __initdata = {
&s3c_device_i2c7,
};
//主机控制器相关
/*file:i2c-s3c2410.c */
static struct platform_driver s3c24xx_i2c_driver = {
	.probe		= s3c24xx_i2c_probe,
	.remove		= s3c24xx_i2c_remove,
	.id_table	= s3c24xx_driver_ids,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c-i2c",
		.pm	= S3C24XX_DEV_PM_OPS,
	},
};

static int __init i2c_adap_s3c_init(void)
{
	return platform_driver_register(&s3c24xx_i2c_driver);
}
//外设驱动相关
/*file:tsc2007.c */
static struct i2c_driver tsc2007_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tsc2007"
	},
	.id_table	= tsc2007_idtable,
	.probe		= tsc2007_probe,
	.remove		= __devexit_p(tsc2007_remove),
};

static int __init tsc2007_init(void)
{
	return i2c_add_driver(&tsc2007_driver);
}
/*-----------------------------------------------------------------------------------
Linux-driver/i2c/i2c-tiny4412-pri中已经对i2c-s3c2410.c主机驱动probe进行了分析，本文不再重复
tsc2007.c外设驱动probe函数分析
-----------------------------------------------------------------------------------*/
static int __devinit tsc2007_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct tsc2007 *ts;
	struct tsc2007_platform_data *pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int err;

	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}
	//检测适配器是否支持处理SMBus read_word_data这个命令，通过一个bitmap，告诉调用者该I2C adapter支
	//持的功能，
	//SMBUS相关的功能
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	//用kzalloc申请内存的时候， 效果等同于先是用 kmalloc() 申请空间 , 然后用 memset() 来初始化 ,
	//所有申请的元素都被初始化为 0.
	ts = kzalloc(sizeof(struct tsc2007), GFP_KERNEL);
	//（向input核心层申请）分配一个input_dev结构体。
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->client = client;
	ts->irq = client->irq;
	ts->input = input_dev;
	/*
	初始化工作队列，用于源源不断的从触控ic中获取数据。tsc2007_work等待中断产生并读取，
	从ic中获取准备上报到系统端的数据。并使能中断，这样触屏在检测到报点的时候，也就是中断发生的
	时候能够上报数据。
	*/
	INIT_DELAYED_WORK(&ts->work, tsc2007_work);

	ts->model             = pdata->model;
	ts->x_plate_ohms      = pdata->x_plate_ohms;
	ts->max_rt            = pdata->max_rt ? : MAX_12BIT;
	ts->poll_delay        = pdata->poll_delay ? : 1;
	ts->poll_period       = pdata->poll_period ? : 1;
	ts->get_pendown_state = pdata->get_pendown_state;
	ts->clear_penirq      = pdata->clear_penirq;

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/ts2007", dev_name(&client->dev));

	input_dev->name = "TSC2007 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;

	/*
	有些Android 4.0系统上层配置必须采用SLOT方式报点，此时若驱动依旧采取系统传统的报点方式，Android上层可能会将上报的坐标标识成相对坐标，如果出现这样的现象，请将GTP_ICS_SLOT_REPORT宏打开，将报点方式切换到SLOT方式即可。详细内容参考linux输入子系统和Android上层inputreader.cpp中关于上报事件的相关资料。
	*/
#if GTP_ICS_SLOT_REPORT
	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);

	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	input_mt_init_slots(input_dev, 255);

	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
    	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, CFG_MAX_TOUCH_POINTS, 0, 0);
#else
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	//input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	//input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, MAX_12BIT, pdata->fuzzx, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_12BIT, pdata->fuzzy, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_12BIT,
			pdata->fuzzz, 0);
#endif

	if (pdata->init_platform_hw)
		pdata->init_platform_hw();

	err = request_irq(ts->irq, tsc2007_irq, IRQ_TYPE_EDGE_FALLING,
			client->dev.driver->name, ts);
	if (err < 0) {
		dev_err(&client->dev, "irq %d busy?\n", ts->irq);
		goto err_free_mem;
	}

	/* Prepare for touch readings - power down ADC and enable PENIRQ */
	err = tsc2007_xfer(ts, PWRDOWN);
	if (err < 0)
		goto err_free_irq;

	////像input核心层注册input_dev结构体
	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	i2c_set_clientdata(client, ts);

	/* add by cym 20130417 */
	//创建proc文件，现在已经被proc_create()替代
	ts_proc_entry = create_proc_entry("driver/micc_ts", 0, NULL);   
	if (ts_proc_entry) {   
		ts_proc_entry->write_proc = ts_proc_write;   
	}
	/* end add */

	/* add by cym 20141202 */
	//Android驱动开发里earlysuspend睡眠模式
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;//EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	ts->early_suspend.suspend = tsc2007_ts_suspend;
	ts->early_suspend.resume = tsc2007_ts_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	/* end add */

	return 0;

 err_free_irq:
	tsc2007_free_irq(ts);
	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

/*
再来回顾一下整个的流程：
首先将 TSC2007 设成 PowerDown 模式---->屏幕被触摸--->tsc2007产生中断--->进入中断入口函数tsc2007_irq
---->关闭中断，延时再提交工作---->未松手则继续提交工作，否则使能中断
*/
