/*
	FT5x06系列ICs是单芯片电容式触摸屏控制器IC，带有一个内置的8位微控制器单元（MCU）。
采用互电容的方法，在配合的相互的电容式触摸面板，它支持真正的多点触摸功能。FT5x06具有用户友好
的输入的功能，这可以应用在许多便携式设备，例如蜂窝式电话，移动互联网设备，上网本和笔记本个人
电脑。FT5x06系列IC包括FT5206/FT5306/FT5406。
	从FT5X06的datasheet中，我们可以看到，FT5X06既可以工作的SPI的接口方式，也可以工作在I2C的接口
方式，不管工作在SPI，还是工作在I2C，从硬件的接口设计上来说，这下面的几个控制口，都是需要要接的。
	1）：INT引脚，这个脚是一个中端信号，它用来通知HOST端FT5X06已经准备好，可以进行读操作了。
	2）：WAKE引脚：这个功能主要的作用是将FT5X06从睡眠状态转换到工作状态。
	3）：/RST引脚：FT5X06的芯片复位信号。
	根据FT5406数据手册上的指令，我们先了解下驱动如何实现电容屏的多点触摸，其实很简单，主要需要触
摸屏IC FT5406 能够捕获多点数据，这点电容屏基本多能支持到捕获2点以上，而FT5406 可以捕获5个触摸点,
编写驱动时，只要去获取这几个点的数据，然后上报就可以了。
	
*/
/*
  I2C的驱动需要根据具体的ARM芯片，一般来说，IC原厂，一般会将在linux的bsp中都会有I2C的驱动，这
个部分不需要我们去写的，我们只需要将FT5X06和BSP包中的I2C驱动匹配起来就好了。
*/
//需要添加i2c的平台设备信息
#ifdef CONFIG_TOUCHSCREEN_FT5X0X
#include <plat/ft5x0x_touch.h>
static void __init smdk4x12_machine_init(void)
{
	s3c_i2c3_set_platdata(NULL);
	i2c_register_board_info(3, i2c_devs3, ARRAY_SIZE(i2c_devs3));
	
	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
}

static struct ft5x0x_i2c_platform_data ft5x0x_pdata = {
        .gpio_irq               = EXYNOS4_GPX0(4),
        .irq_cfg                = S3C_GPIO_SFN(0xf),
        .screen_max_x   = 768,
        .screen_max_y   = 1024,
        .pressure_max   = 255,
};

static struct i2c_board_info i2c_devs3[] __initdata = {
	/* support for FT5X0X TouchScreen */
#if defined(CONFIG_TOUCHSCREEN_FT5X0X)
	{
		I2C_BOARD_INFO("ft5x0x_ts", 0x70>>1),
		.irq = IRQ_EINT(4),
		.platform_data = &ft5x0x_pdata,
	},
#endif
	/* end add */
};

struct platform_device s3c_device_i2c3 = {
	.name		= "s3c2440-i2c",
	.id		= 3,
	.num_resources	= ARRAY_SIZE(s3c_i2c_resource),
	.resource	= s3c_i2c_resource,
};

void __init s3c_i2c3_set_platdata(struct s3c2410_platform_i2c *pd)
{
	struct s3c2410_platform_i2c *npd;

	if (!pd) {
		pd = &default_i2c_data;
		pd->bus_num = 3;
	}

	npd = s3c_set_platdata(pd, sizeof(struct s3c2410_platform_i2c),
			       &s3c_device_i2c3);

	if (!npd->cfg_gpio)
		npd->cfg_gpio = s3c_i2c3_cfg_gpio;
}
static struct platform_device *smdk4x12_devices[] __initdata = {
&s3c_device_i2c3,
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
/*file:ft5x06_ts.c */
static struct i2c_driver ft5x0x_ts_driver = {
	.probe		= ft5x0x_ts_probe,
	.remove		= __devexit_p(ft5x0x_ts_remove),
	.id_table	= ft5x0x_ts_id,
	.driver	= {
		.name	= FT5X0X_NAME,
		.owner	= THIS_MODULE,
	},
};
static int __init ft5x0x_ts_init(void)
{
	int ret;
	int type;
	type = get_lcd_type();

#if 1
        //TP1_EN
	//printk("==%s: TP1_EN==\n", __FUNCTION__);
        ret = gpio_request(EXYNOS4_GPL0(2), "TP1_EN");
        if (ret) {
                printk(KERN_ERR "failed to request TP1_EN for "
                        "I2C control\n");
                //return err;
        }

        gpio_direction_output(EXYNOS4_GPL0(2), 1);

        s3c_gpio_cfgpin(EXYNOS4_GPL0(2), S3C_GPIO_OUTPUT);
        gpio_free(EXYNOS4_GPL0(2));

        mdelay(5);
#endif

#if 1
        printk("==%s: reset==\n", __FUNCTION__);
        ret = gpio_request(EXYNOS4_GPX0(3), "GPX0_3");
        if (ret) {
                gpio_free(EXYNOS4_GPX0(3));

                ret = gpio_request(EXYNOS4_GPX0(3), "GPX0_3");
                if(ret)
                {
                        printk("ft5xox: Failed to request GPX0_3 \n");
                }
        }
        gpio_direction_output(EXYNOS4_GPX0(3), 0);
        mdelay(200);
	
        gpio_direction_output(EXYNOS4_GPX0(3), 1);
	

        s3c_gpio_cfgpin(EXYNOS4_GPX0(3), S3C_GPIO_OUTPUT);
        gpio_free(EXYNOS4_GPX0(3));
        msleep(300);
#endif
	//type = get_lcd_type();

	if(0x00 == type)  //9.7
	{
		TOUCH_MAX_X = 1024;
                TOUCH_MAX_Y = 768;

#ifdef CONFIG_VT        //for Ubuntu
                touch_size = 1;
                scal_xy = 1;
#else
                touch_size = 0;
#endif
	}
	else if(0x01 == type) //7.0
	{
#ifdef CONFIG_VT        //for Ubuntu
                TOUCH_MAX_X = 800;//1280;
                TOUCH_MAX_Y = 1280;//800;

                scal_xy = 1;
                touch_size = 0;
#else
                touch_size = 1;
#endif
	}
	else if(0x02 == type)  //4.3
	{
		;
	}

	if(1 == touch_size)
	{
		swap_xy = 1;
	}
	else
	{
		swap_xy = 0;
	}

	return i2c_add_driver(&ft5x0x_ts_driver);
}
/*-----------------------------------------------------------------------------------
Linux-driver/i2c/i2c-tiny4412-pri中已经对i2c-s3c2410.c主机驱动probe进行了分析，本文不再重复
ft5x06_ts.c外设驱动probe函数分析
-----------------------------------------------------------------------------------*/
static int ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_i2c_platform_data *pdata;
	struct ft5x0x_ts_data *ts;
	struct input_dev *input_dev;
	unsigned char val;
//	unsigned int ctp_id;
	int err = -EINVAL;

	//cym ctp_id = get_ctp();
	//if (ctp_id != CTP_FT5X06 && ctp_id != CTP_AUTO) {
	//	return -ENODEV;
	//}

	//检测适配器是否支持I2C_FUNC_I2C通讯方式，通过一个bitmap，告诉调用者该I2C adapter支持的功能，
	//包括I2C_FUNC_I2C，支持传统的I2C功能；还包括其他
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	//用kzalloc申请内存的时候， 效果等同于先是用 kmalloc() 申请空间 , 然后用 memset() 来初始化 ,
	//所有申请的元素都被初始化为 0.
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	pdata = client->dev.platform_data;//获取平台数据
	if (!pdata) {
		dev_err(&client->dev, "failed to get platform data\n");
		goto exit_no_pdata;
	}
	/*
	分贝通过i2c将触控IC中的屏幕的分辨率以及压力值，还有中断号读取出
	来存在一个结构体中。
	*/
	ts->screen_max_x = pdata->screen_max_x;
	ts->screen_max_y = pdata->screen_max_y;
	ts->pressure_max = pdata->pressure_max;

	ts->gpio_irq = pdata->gpio_irq;
	if (ts->gpio_irq != -EINVAL) {
		client->irq = gpio_to_irq(ts->gpio_irq);//申请中断号
	} else {
		goto exit_no_pdata;
	}
	if (pdata->irq_cfg) {
		s3c_gpio_cfgpin(ts->gpio_irq, pdata->irq_cfg);
		s3c_gpio_setpull(ts->gpio_irq, S3C_GPIO_PULL_NONE);
	}

	ts->gpio_wakeup = pdata->gpio_wakeup;
	ts->gpio_reset = pdata->gpio_reset;

	/*
	初始化工作队列，用于源源不断的从触控ic中获取数据。ft5x0x_ts_pen_irq_work等待中断产生并读取，
	从ic中获取准备上报到系统端的数据。并使能中断，这样触屏在检测到报点的时候，也就是中断发生的
	时候能够上报数据。
	*/
	INIT_WORK(&ts->work, ft5x0x_ts_pen_irq_work);
	this_client = client;
	i2c_set_clientdata(client, ts);

	//保存在这个变量中，后面执行工作队列的时候就可以获取该变量的值，找到对应work要执行的function
	ts->queue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ts->queue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	//输入子系统
	ts->input_dev = input_dev;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

#ifdef CONFIG_FT5X0X_MULTITOUCH
	set_bit(ABS_MT_TRACKING_ID, input_dev->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ts->screen_max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ts->screen_max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, ts->pressure_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, FT5X0X_PT_MAX, 0, 0);
#else
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, ts->screen_max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, ts->screen_max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, ts->pressure_max, 0 , 0);
#endif

	/*
	填写驱动必要的信息，如驱动名称，描述信息，使用的总线协议，厂商ID号，产品ID号
	版本号。
	*/
	input_dev->name = FT5X0X_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x12FA;
	input_dev->id.product = 0x2143;
	input_dev->id.version = 0x0100;

	//注册输入设备
	err = input_register_device(input_dev);
	if (err) {
		input_free_device(input_dev);
		dev_err(&client->dev, "failed to register input device %s, %d\n",
				dev_name(&client->dev), err);
		goto exit_input_dev_alloc_failed;
	}

	msleep(3);
	err = ft5x0x_read_fw_ver(&val);//读取芯片ID
	if (err < 0) {
		dev_err(&client->dev, "chip not found\n");
		goto exit_irq_request_failed;
	}

	//注册中断服务函数，ft5x0x_ts_interrupt查找工作项当前是否挂起，调用queue_work(ts->queue, &ts->work);
	//用来获取ic发生中断的数据，并上报到input子系统
	err = request_irq(client->irq, ft5x0x_ts_interrupt,
			IRQ_TYPE_EDGE_FALLING /*IRQF_TRIGGER_FALLING*/, "ft5x0x_ts", ts);
	if (err < 0) {
		dev_err(&client->dev, "Request IRQ %d failed, %d\n", client->irq, err);
		goto exit_irq_request_failed;
	}

	disable_irq(client->irq);

	dev_info(&client->dev, "Firmware version 0x%02x\n", val);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;//EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ts->early_suspend.resume = ft5x0x_ts_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	enable_irq(client->irq);

	//cym 4412_set_ctp(CTP_FT5X06);
	dev_info(&client->dev, "FocalTech ft5x0x TouchScreen initialized\n");
	return 0;

exit_irq_request_failed:
	input_unregister_device(input_dev);

exit_input_dev_alloc_failed:
	cancel_work_sync(&ts->work);
	destroy_workqueue(ts->queue);

exit_create_singlethread:
	i2c_set_clientdata(client, NULL);

exit_no_pdata:
	kfree(ts);

exit_alloc_data_failed:
exit_check_functionality_failed:
	dev_err(&client->dev, "probe ft5x0x TouchScreen failed, %d\n", err);

	return err;
}
//这个驱动到这里大概的分析基本完成了，我们的i2c使用的是静态注册
/*
屏幕被触摸--->ft5x06产生中断--->进入中断入口函数ft5x0x_ts_interrupt--->关闭中断--->等待调度--->
得到CPU资源进入ft5x0x_ts_pen_irq_work执行工作--->开启中断--当次处理结束。
*/



