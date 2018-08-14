/*
该驱动的RTC使用I2C接口操作pcf8563，这里针对pcf8563对Linux的RTC框架进行分析

i2c的平台设备相关的初始化和注册的过程就略过了，直接看调用的probe函数
*/
static int pcf8563_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct pcf8563 *pcf8563;

	int err = 0;

	dev_dbg(&client->dev, "%s\n", __func__);

	//判定适配器能力如
	/*
	static const struct i2c_algorithm smbus_algorithm = {
                .smbus_xfer= i801_access,
                .functionality= i801_func,
        };
	*/
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	pcf8563 = kzalloc(sizeof(struct pcf8563), GFP_KERNEL);
	if (!pcf8563)
		return -ENOMEM;

	dev_info(&client->dev, "chip found, driver version " DRV_VERSION "\n");

	/*
	将自定义的设备结构dev赋给设备驱动client的私有指针
	*/
	i2c_set_clientdata(client, pcf8563);

	pcf8563->rtc = rtc_device_register(pcf8563_driver.driver.name,
				&client->dev, &pcf8563_rtc_ops, THIS_MODULE);

	if (IS_ERR(pcf8563->rtc)) {
		err = PTR_ERR(pcf8563->rtc);
		goto exit_kfree;
	}

	return 0;

exit_kfree:
	kfree(pcf8563);

	return err;
}
/*探测函数比较简单，比较重要的语句为红色标注部分，这里主要涉及到两个部分。
        1、rtc设备注册函数rtc_device_register，此函数完成rtc设备的注册，在后面会重点讲述。
        2、pcf8563_rtc_ops，此结构体定义了操作pcf8563的函数，包括读时间和设置时间等，上层调用的对时间操作就是调用此处的函数，具体如下：*/
static const struct rtc_class_ops pcf8563_rtc_ops = {
	.ioctl		= pcf8563_rtc_ioctl,
	.read_time	= pcf8563_rtc_read_time,
	.set_time	= pcf8563_rtc_set_time,
};

/*
读时间函数pcf8563_rtc_read_time，就是通过I2C接口读取pcf8563时间寄存器里的值
设置时间函数pcf8563_rtc_set_time，就是通过I2C接口写pcf8563时间寄存器里的值
最重要的rtc注册函数rtc_device_register，在class.c中
*/
struct rtc_device *rtc_device_register(const char *name, struct device *dev,
					const struct rtc_class_ops *ops,
					struct module *owner)
{
	struct rtc_device *rtc;
	struct rtc_wkalrm alrm;
	int id, err;

	id = ida_simple_get(&rtc_ida, 0, 0, GFP_KERNEL);
	if (id < 0) {
		err = id;
		goto exit;
	}

	rtc = kzalloc(sizeof(struct rtc_device), GFP_KERNEL);
	if (rtc == NULL) {
		err = -ENOMEM;
		goto exit_ida;
	}

	rtc->id = id;
	rtc->ops = ops;
	rtc->owner = owner;
	rtc->irq_freq = 1;
	rtc->max_user_freq = 64;
	rtc->dev.parent = dev;
	rtc->dev.class = rtc_class;
	rtc->dev.release = rtc_device_release;

	mutex_init(&rtc->ops_lock);
	spin_lock_init(&rtc->irq_lock);
	spin_lock_init(&rtc->irq_task_lock);
	init_waitqueue_head(&rtc->irq_queue);

	/* Init timerqueue */
	timerqueue_init_head(&rtc->timerqueue);
	INIT_WORK(&rtc->irqwork, rtc_timer_do_work);
	/* Init aie timer */
	rtc_timer_init(&rtc->aie_timer, rtc_aie_update_irq, (void *)rtc);
	/* Init uie timer */
	rtc_timer_init(&rtc->uie_rtctimer, rtc_uie_update_irq, (void *)rtc);
	/* Init pie timer */
	hrtimer_init(&rtc->pie_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	rtc->pie_timer.function = rtc_pie_update_irq;
	rtc->pie_enabled = 0;

	/* Check to see if there is an ALARM already set in hw */
	err = __rtc_read_alarm(rtc, &alrm);

	if (!err && !rtc_valid_tm(&alrm.time))
		rtc_initialize_alarm(rtc, &alrm);

	strlcpy(rtc->name, name, RTC_DEVICE_NAME_SIZE);
	dev_set_name(&rtc->dev, "rtc%d", id);

	rtc_dev_prepare(rtc);

	err = device_register(&rtc->dev);
	if (err) {
		put_device(&rtc->dev);
		goto exit_kfree;
	}

	rtc_dev_add_device(rtc);
	rtc_sysfs_add_device(rtc);
	rtc_proc_add_device(rtc);

	dev_info(dev, "rtc core: registered %s as %s\n",
			rtc->name, dev_name(&rtc->dev));

	return rtc;

exit_kfree:
	kfree(rtc);

exit_ida:
	ida_simple_remove(&rtc_ida, id);

exit:
	dev_err(dev, "rtc core: unable to register %s, err = %d\n",
			name, err);
	return ERR_PTR(err);
}
/*
（1）处理一个idr结构，idr在linux内核中指的就是整数ID管理机制，从本质上来说，idr是一种将整数ID号和特定指针关联在一起的机制。
			这个机制最早是在2003年2月加入内核的，当时是作为POSIX定时器的一个补丁。现在在内核的很多地方都可以找到idr的身影。这里从内核中获取一个idr结构，并与id相关联。
（2）分配了一个rtc_device的结构--rtc，并且初始化了相关的成员：id, rtc_class_ops等等。
（3）首先调用rtc_dev_prepare（在rtc-dev.c中定义）。因为RTC设备本质来讲还是字符设备，所以这里初始化了字符设备相关的结构：设备号以及文件操作。
     然后调用device_register将设备注册到linux设备模型核心。这样在模块加载的时候，udev daemon就会自动为我们创建设备文件rtc(n)。
（4）先后调用rtc_dev_add_device，rtc_sysfs_add_device，rtc_proc_add_device三个函数。 rtc_dev_add_device注册字符设备，
     rtc_sysfs_add_device只是为设备添加了一个闹钟属性，rtc_proc_add_device 创建proc文件系统接口。
*/
//class.c中初始化函数rtc_init()为：
static int __init rtc_init(void)
{
    rtc_class = class_create(THIS_MODULE, "rtc");    //创建rtc设备类
    if (IS_ERR(rtc_class)) {
        printk(KERN_ERR "%s: couldn't create class\n", __FILE__);
        return PTR_ERR(rtc_class);
    }
    rtc_class->suspend = rtc_suspend;    //挂起函数，后面讲述
    rtc_class->resume = rtc_resume;    //恢复函数，后面讲述
    rtc_dev_init();    //分配设备号，rtc-dev.c，后面讲述
    rtc_sysfs_init(rtc_class);    //创建sys,rtc-sysfs.c后面讲述
    return 0;
}
/*rtc_init 首先调用class_create创建了一个类--rtc_class。我们知道类是一个设备的高层视图，他抽象出了底层的实现细节。
类的作用就是向用户空间提供设备 的信息，驱动程序不需要直接处理类。然后初始化类结构的相应成员，rtc_suspend，
rtc_resume这两个函数也是在class.c中实现 的。接下来调用rtc_dev_init()，这个函数为RTC设备动态分配设备号，保存在rtc_devt中。最后调用 rtc_sysfs_init，初始化rtc_class的属性。
subsys_initcall(rtc_init);
    由subsys_initcall(rtc_init);知道，此函数在系统开始运行的时候即被执行。*/














