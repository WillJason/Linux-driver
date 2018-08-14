/*
Linux常见的驱动在driver目录下都有一个文件夹，当然我们的RTC也不例外，
我们进入kernel主目录下的drivers/rtc，发现下面包含了许多开发平台的RTC驱动，
我们这里是以S3C24xx为主，所以我们要寻找的是“rtc-s3c.c”，她是我们要分析的核心；

首先，当然是我们主角中的战斗机rtc-s3c.c，她是最顶层的直接和硬件打交道的驱动文件，
每个平台，高通，marvell，三星都有自己的这部分，一般被命名为类似的”rtc-msm.c,rtc-pxa.c，rtc-s3c.c”;
rtc-s3c.c上面的是interface.c，顾名思义就知道是接口文件，它主要是对rtc-s3c.c进行封装，
给上层提供统一的接口，屏蔽底层差异化的东东。
Interface.c再往上就到了rtc-dev.c.，rtc-dev.c最终生成了/dev/rtc，上层的应用程序
就是通过操作此文件来进行RTC的相关的设置系统时间，设置闹钟，等等。

上层应用层序
        |
        |
        V
rtc-dev.c
        |
        |
        V
interface.c
       |
       |
       V
rtc-s3c.c

class.c：提供了RTC子系统一些的公共函数，让各个RTC驱动注册集成到我们的linux内核中

hctosys.c：系统起来之后，会调用到这个文件中的rtc_hctosys()函数，
主要功能是系统起来的时候，去读RTC硬件中的时间，然后更新我们的系统时间。

开始分析真正的主角rtc-s3c.c.看一个驱动，一般都是从驱动文件的最底层看起
*/
//先来看一下平台信息
static void __init smdk4x12_machine_init(void)
{
	//...
	smdk4x12_rtc_wake_init();
	
	//注册平台设备信息
	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
	//...
}
static void smdk4x12_rtc_wake_init(void)
{
#ifdef CONFIG_PM
	gic_arch_extn.irq_set_wake = s3c_irq_wake;
#endif
}
/*s3c_irq_wake()函数提供至关重要的两点信息：
1、作为唤醒源的中断，必须被允许具有唤醒功能，即配置s3c_irqwake_eintallow变量；
2、允许之后，开启该中断的唤醒功能，即取消中断的屏蔽，具体就是s3c_irqwake_intmask变量设置，else分支中。
*/
static struct platform_device *smdk4x12_devices[] __initdata = {
	&s3c_device_rtc,
}
#ifdef CONFIG_S3C_DEV_RTC
static struct resource s3c_rtc_resource[] = {
	[0] = DEFINE_RES_MEM(S3C_PA_RTC, SZ_256),
	[1] = DEFINE_RES_IRQ(IRQ_RTC_ALARM),
	[2] = DEFINE_RES_IRQ(IRQ_RTC_TIC),
};

struct platform_device s3c_device_rtc = {
	.name		= "s3c64xx-rtc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(s3c_rtc_resource),
	.resource	= s3c_rtc_resource,
};
#endif /* CONFIG_S3C_DEV_RTC */
/*-----------------------------------------------------------------------------------
samsung rtc-s3c.c分析
-----------------------------------------------------------------------------------*/
static struct platform_driver s3c_rtc_driver = {
	.probe		= s3c_rtc_probe,
	.remove		= __devexit_p(s3c_rtc_remove),
	.suspend	= s3c_rtc_suspend,
	.resume		= s3c_rtc_resume,
	.id_table	= s3c_rtc_driver_ids,
	.driver		= {
		.name	= "s3c-rtc",
		.owner	= THIS_MODULE,
		.of_match_table	= s3c_rtc_dt_match,
	},
};

module_platform_driver(s3c_rtc_driver);

static struct platform_device_id s3c_rtc_driver_ids[] = {
	{
		.name		= "s3c64xx-rtc",
		.driver_data	= TYPE_S3C64XX,
	},
	{ }
};
/*我们这里采用的是s3c_rtc_driver_ids匹配，该驱动程序也支持设备树匹配加载*/
static const struct of_device_id s3c_rtc_dt_match[] = {
	{
		.compatible = "samsung,s3c2410-rtc",
		.data = &s3c_rtc_drv_data_array[TYPE_S3C2410],
	},
	//........
};
static int __devinit s3c_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;
	struct rtc_time rtc_tm;
	struct resource *res;
	int ret;
	int tmp;

	pr_debug("%s: probe=%p\n", __func__, pdev);

	/* find the IRQs */

	/*获得IRQ资源中的第二个，即TICK节拍时间中断号*/  
	s3c_rtc_tickno = platform_get_irq(pdev, 1);
	if (s3c_rtc_tickno < 0) {
		dev_err(&pdev->dev, "no irq for rtc tick\n");
		return -ENOENT;
	}

	/*获取IRQ资源中的第一个，即RTC报警中断*/ 
	s3c_rtc_alarmno = platform_get_irq(pdev, 0);
	if (s3c_rtc_alarmno < 0) {
		dev_err(&pdev->dev, "no irq for alarm\n");
		return -ENOENT;
	}

	pr_debug("s3c2410_rtc: tick irq %d, alarm irq %d\n",
		 s3c_rtc_tickno, s3c_rtc_alarmno);

	/* get the memory region */
	/*获取RTC平台设备所使用的IO端口资源*/ 
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get memory region resource\n");
		return -ENOENT;
	}
	/*申请IO端口资源所占用的IO空间*/
	s3c_rtc_mem = request_mem_region(res->start, resource_size(res),
					 pdev->name);

	if (s3c_rtc_mem == NULL) {
		dev_err(&pdev->dev, "failed to reserve memory region\n");
		ret = -ENOENT;
		goto err_nores;
	}
	/*将IO端口占用的IO空间映射到虚拟地址，s3c_rtc_base是这段虚拟地址的起始地址*/
	s3c_rtc_base = ioremap(res->start, resource_size(res));
	if (s3c_rtc_base == NULL) {
		dev_err(&pdev->dev, "failed ioremap()\n");
		ret = -EINVAL;
		goto err_nomap;
	}

	rtc_clk = clk_get(&pdev->dev, "rtc");//获取平台时钟
	if (IS_ERR(rtc_clk)) {
		dev_err(&pdev->dev, "failed to find rtc clock source\n");
		ret = PTR_ERR(rtc_clk);
		rtc_clk = NULL;
		goto err_clk;
	}

	clk_enable(rtc_clk);

	/* check to see if everything is setup correctly */
	/*对RTCCON第0位进行操作，使能RTC*/ 
	s3c_rtc_enable(pdev, 1);

	pr_debug("s3c2410_rtc: RTCCON=%02x\n",
		 readw(s3c_rtc_base + S3C2410_RTCCON));
	/*让电源管理支持唤醒功能*/
	device_init_wakeup(&pdev->dev, 1);

	/* register RTC and exit */
	/*注册rtc设备，名为"s3c",与s3c_rtcops这个rtc_class_ops进行关联*/ 
	rtc = rtc_device_register("s3c", &pdev->dev, &s3c_rtcops,
				  THIS_MODULE);

	if (IS_ERR(rtc)) {
		dev_err(&pdev->dev, "cannot attach rtc\n");
		ret = PTR_ERR(rtc);
		goto err_nortc;
	}

	//得到当前CPU的类型，因为该驱动是通用的，也适用于其他s3c类型的处理器
	s3c_rtc_cpu_type = s3c_rtc_get_driver_data(pdev);

	/* Check RTC Time */
	//得到RTC的当前时间
	s3c_rtc_gettime(NULL, &rtc_tm);
	//如果RTC的当前时间无效，则重新设置
	if (rtc_valid_tm(&rtc_tm)) {
		/* Set the default time to 2013-1-1 12:00 */
		rtc_tm.tm_year	= 116;
		rtc_tm.tm_mon	= 0;
		rtc_tm.tm_mday	= 1;
		rtc_tm.tm_hour	= 12;
		rtc_tm.tm_min	= 0;
		rtc_tm.tm_sec	= 0;

		s3c_rtc_settime(NULL, &rtc_tm);

		dev_warn(&pdev->dev, "warning: invalid RTC value so initializing it\n");
	}
	//依据CPU的类型，设置RTC节拍
	if (s3c_rtc_cpu_type != TYPE_S3C2410)
		rtc->max_user_freq = 32768;
	else
		rtc->max_user_freq = 128;

	if (s3c_rtc_cpu_type == TYPE_S3C2416 || s3c_rtc_cpu_type == TYPE_S3C2443) {
		tmp = readw(s3c_rtc_base + S3C2410_RTCCON);
		tmp |= S3C2443_RTCCON_TICSEL;
		writew(tmp, s3c_rtc_base + S3C2410_RTCCON);
	}

	 /*将rtc这个rtc_device存放在&pdev->dev->driver_data*/  
	platform_set_drvdata(pdev, rtc);

	/*对TICNT第7位进行操作，使能节拍时间计数寄存器*/
	s3c_rtc_setfreq(&pdev->dev, 1);

	//申请RTC报警中断
	ret = request_irq(s3c_rtc_alarmno, s3c_rtc_alarmirq,
			  0,  "s3c2410-rtc alarm", rtc);
	if (ret) {
		dev_err(&pdev->dev, "IRQ%d error %d\n", s3c_rtc_alarmno, ret);
		goto err_alarm_irq;
	}

	//申请RTC时间节拍中断
	ret = request_irq(s3c_rtc_tickno, s3c_rtc_tickirq,
			  0,  "s3c2410-rtc tick", rtc);
	if (ret) {
		dev_err(&pdev->dev, "IRQ%d error %d\n", s3c_rtc_tickno, ret);
		free_irq(s3c_rtc_alarmno, rtc);
		goto err_tick_irq;
	}

	clk_disable(rtc_clk);

	return 0;

 err_tick_irq:
	free_irq(s3c_rtc_alarmno, rtc);

 err_alarm_irq:
	platform_set_drvdata(pdev, NULL);
	rtc_device_unregister(rtc);

 err_nortc:
	s3c_rtc_enable(pdev, 0);
	clk_disable(rtc_clk);
	clk_put(rtc_clk);

 err_clk:
	iounmap(s3c_rtc_base);

 err_nomap:
	release_resource(s3c_rtc_mem);

 err_nores:
	return ret;
}
//下面介绍一下2440的RTC操作集——s3c_rtcops：
static const struct rtc_class_ops s3c_rtcops = {
	.read_time	= s3c_rtc_gettime, //读取当前时间
	.set_time	= s3c_rtc_settime, //设置当前时间
	.read_alarm	= s3c_rtc_getalarm,//读取报警时间
	.set_alarm	= s3c_rtc_setalarm,//设置报警时间
	.proc		= s3c_rtc_proc,
	.alarm_irq_enable = s3c_rtc_setaie, //用于设置RTCALM寄存器
};

struct rtc_device *rtc_device_register(const char *name, struct device *dev,  
                    const struct rtc_class_ops *ops,  
                    struct module *owner)  
{  
    struct rtc_device *rtc;  
    int id, err;  
    /*为idr(rtc_idr)分配内存*/  
    if (idr_pre_get(&rtc_idr, GFP_KERNEL) == 0) {  
        err = -ENOMEM;  
        goto exit;  
    }  
    mutex_lock(&idr_lock);  
    /*分配ID号存于id中，该ID号最终将作为该RTC设备的次设备号*/  
    err = idr_get_new(&rtc_idr, NULL, &id);  
    mutex_unlock(&idr_lock);  
    if (err < 0)  
        goto exit;  
    id = id & MAX_ID_MASK;  
    /*为RTC结构分配内存*/  
    rtc = kzalloc(sizeof(struct rtc_device), GFP_KERNEL);  
    if (rtc == NULL) {  
        err = -ENOMEM;  
        goto exit_idr;  
    }  
    rtc->id = id;  
    /*指向原始操作函数集*/  
    rtc->ops = ops;    
    rtc->owner = owner;  
    rtc->max_user_freq = 64;  
    rtc->dev.parent = dev;  
    rtc->dev.class = rtc_class;  
    rtc->dev.release = rtc_device_release;  
    mutex_init(&rtc->ops_lock);  
    spin_lock_init(&rtc->irq_lock);  
    spin_lock_init(&rtc->irq_task_lock);  
    init_waitqueue_head(&rtc->irq_queue);  
    strlcpy(rtc->name, name, RTC_DEVICE_NAME_SIZE);  
    dev_set_name(&rtc->dev, "rtc%d", id);  
    /*rtc->dev.devt = MKDEV(MAJOR(rtc_devt),rtc->id); cdev_init(&rtc->char_dev,&rtc_dev_fops);其中rtc_devt是从调用alloc_chrdev_region时获得的*/  
    rtc_dev_prepare(rtc);  
    /*注册该RTC设备rtc->dev*/  
    err = device_register(&rtc->dev);  
    if (err)  
        goto exit_kfree;  
    /*cdev_add(&rtc->chr_dev,rtc->dev.devt,1);将rtc->chrdev注册到系统中*/  
    rtc_dev_add_device(rtc);  
    /*在/sys下添加属性文件*/  
    rtc_sysfs_add_device(rtc);  
    /*在/proc中创建入口项"driver/rtc"*/  
    rtc_proc_add_device(rtc);  
    dev_info(dev, "rtc core: registered %s as %s\n",  
            rtc->name, dev_name(&rtc->dev));  
    return rtc;  
exit_kfree:  
    kfree(rtc);  
exit_idr:  
    mutex_lock(&idr_lock);  
    idr_remove(&rtc_idr, id);  
    mutex_unlock(&idr_lock);  
exit:  
    dev_err(dev, "rtc core: unable to register %s, err = %d\n",  
            name, err);  
    return ERR_PTR(err);  
}  























