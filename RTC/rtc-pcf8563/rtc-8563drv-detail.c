/*
��������RTCʹ��I2C�ӿڲ���pcf8563���������pcf8563��Linux��RTC��ܽ��з���

i2c��ƽ̨�豸��صĳ�ʼ����ע��Ĺ��̾��Թ��ˣ�ֱ�ӿ����õ�probe����
*/
static int pcf8563_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct pcf8563 *pcf8563;

	int err = 0;

	dev_dbg(&client->dev, "%s\n", __func__);

	//�ж�������������
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
	���Զ�����豸�ṹdev�����豸����client��˽��ָ��
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
/*̽�⺯���Ƚϼ򵥣��Ƚ���Ҫ�����Ϊ��ɫ��ע���֣�������Ҫ�漰���������֡�
        1��rtc�豸ע�ắ��rtc_device_register���˺������rtc�豸��ע�ᣬ�ں�����ص㽲����
        2��pcf8563_rtc_ops���˽ṹ�嶨���˲���pcf8563�ĺ�����������ʱ�������ʱ��ȣ��ϲ���õĶ�ʱ��������ǵ��ô˴��ĺ������������£�*/
static const struct rtc_class_ops pcf8563_rtc_ops = {
	.ioctl		= pcf8563_rtc_ioctl,
	.read_time	= pcf8563_rtc_read_time,
	.set_time	= pcf8563_rtc_set_time,
};

/*
��ʱ�亯��pcf8563_rtc_read_time������ͨ��I2C�ӿڶ�ȡpcf8563ʱ��Ĵ������ֵ
����ʱ�亯��pcf8563_rtc_set_time������ͨ��I2C�ӿ�дpcf8563ʱ��Ĵ������ֵ
����Ҫ��rtcע�ắ��rtc_device_register����class.c��
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
��1������һ��idr�ṹ��idr��linux�ں���ָ�ľ�������ID������ƣ��ӱ�������˵��idr��һ�ֽ�����ID�ź��ض�ָ�������һ��Ļ��ơ�
			���������������2003��2�¼����ں˵ģ���ʱ����ΪPOSIX��ʱ����һ���������������ں˵ĺܶ�ط��������ҵ�idr����Ӱ��������ں��л�ȡһ��idr�ṹ������id�������
��2��������һ��rtc_device�Ľṹ--rtc�����ҳ�ʼ������صĳ�Ա��id, rtc_class_ops�ȵȡ�
��3�����ȵ���rtc_dev_prepare����rtc-dev.c�ж��壩����ΪRTC�豸�������������ַ��豸�����������ʼ�����ַ��豸��صĽṹ���豸���Լ��ļ�������
     Ȼ�����device_register���豸ע�ᵽlinux�豸ģ�ͺ��ġ�������ģ����ص�ʱ��udev daemon�ͻ��Զ�Ϊ���Ǵ����豸�ļ�rtc(n)��
��4���Ⱥ����rtc_dev_add_device��rtc_sysfs_add_device��rtc_proc_add_device���������� rtc_dev_add_deviceע���ַ��豸��
     rtc_sysfs_add_deviceֻ��Ϊ�豸�����һ���������ԣ�rtc_proc_add_device ����proc�ļ�ϵͳ�ӿڡ�
*/
//class.c�г�ʼ������rtc_init()Ϊ��
static int __init rtc_init(void)
{
    rtc_class = class_create(THIS_MODULE, "rtc");    //����rtc�豸��
    if (IS_ERR(rtc_class)) {
        printk(KERN_ERR "%s: couldn't create class\n", __FILE__);
        return PTR_ERR(rtc_class);
    }
    rtc_class->suspend = rtc_suspend;    //�����������潲��
    rtc_class->resume = rtc_resume;    //�ָ����������潲��
    rtc_dev_init();    //�����豸�ţ�rtc-dev.c�����潲��
    rtc_sysfs_init(rtc_class);    //����sys,rtc-sysfs.c���潲��
    return 0;
}
/*rtc_init ���ȵ���class_create������һ����--rtc_class������֪������һ���豸�ĸ߲���ͼ����������˵ײ��ʵ��ϸ�ڡ�
������þ������û��ռ��ṩ�豸 ����Ϣ������������Ҫֱ�Ӵ����ࡣȻ���ʼ����ṹ����Ӧ��Ա��rtc_suspend��
rtc_resume����������Ҳ����class.c��ʵ�� �ġ�����������rtc_dev_init()���������ΪRTC�豸��̬�����豸�ţ�������rtc_devt�С������� rtc_sysfs_init����ʼ��rtc_class�����ԡ�
subsys_initcall(rtc_init);
    ��subsys_initcall(rtc_init);֪�����˺�����ϵͳ��ʼ���е�ʱ�򼴱�ִ�С�*/














