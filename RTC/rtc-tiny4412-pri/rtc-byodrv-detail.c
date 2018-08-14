/*
Linux������������driverĿ¼�¶���һ���ļ��У���Ȼ���ǵ�RTCҲ�����⣬
���ǽ���kernel��Ŀ¼�µ�drivers/rtc�����������������࿪��ƽ̨��RTC������
������������S3C24xxΪ������������ҪѰ�ҵ��ǡ�rtc-s3c.c������������Ҫ�����ĺ��ģ�

���ȣ���Ȼ�����������е�ս����rtc-s3c.c����������ֱ�Ӻ�Ӳ���򽻵��������ļ���
ÿ��ƽ̨����ͨ��marvell�����Ƕ����Լ����ⲿ�֣�һ�㱻����Ϊ���Ƶġ�rtc-msm.c,rtc-pxa.c��rtc-s3c.c��;
rtc-s3c.c�������interface.c������˼���֪���ǽӿ��ļ�������Ҫ�Ƕ�rtc-s3c.c���з�װ��
���ϲ��ṩͳһ�Ľӿڣ����εײ���컯�Ķ�����
Interface.c�����Ͼ͵���rtc-dev.c.��rtc-dev.c����������/dev/rtc���ϲ��Ӧ�ó���
����ͨ���������ļ�������RTC����ص�����ϵͳʱ�䣬�������ӣ��ȵȡ�

�ϲ�Ӧ�ò���
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

class.c���ṩ��RTC��ϵͳһЩ�Ĺ����������ø���RTC����ע�Ἧ�ɵ����ǵ�linux�ں���

hctosys.c��ϵͳ����֮�󣬻���õ�����ļ��е�rtc_hctosys()������
��Ҫ������ϵͳ������ʱ��ȥ��RTCӲ���е�ʱ�䣬Ȼ��������ǵ�ϵͳʱ�䡣

��ʼ��������������rtc-s3c.c.��һ��������һ�㶼�Ǵ������ļ�����ײ㿴��
*/
//������һ��ƽ̨��Ϣ
static void __init smdk4x12_machine_init(void)
{
	//...
	smdk4x12_rtc_wake_init();
	
	//ע��ƽ̨�豸��Ϣ
	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
	//...
}
static void smdk4x12_rtc_wake_init(void)
{
#ifdef CONFIG_PM
	gic_arch_extn.irq_set_wake = s3c_irq_wake;
#endif
}
/*s3c_irq_wake()�����ṩ������Ҫ��������Ϣ��
1����Ϊ����Դ���жϣ����뱻������л��ѹ��ܣ�������s3c_irqwake_eintallow������
2������֮�󣬿������жϵĻ��ѹ��ܣ���ȡ���жϵ����Σ��������s3c_irqwake_intmask�������ã�else��֧�С�
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
samsung rtc-s3c.c����
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
/*����������õ���s3c_rtc_driver_idsƥ�䣬����������Ҳ֧���豸��ƥ�����*/
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

	/*���IRQ��Դ�еĵڶ�������TICK����ʱ���жϺ�*/  
	s3c_rtc_tickno = platform_get_irq(pdev, 1);
	if (s3c_rtc_tickno < 0) {
		dev_err(&pdev->dev, "no irq for rtc tick\n");
		return -ENOENT;
	}

	/*��ȡIRQ��Դ�еĵ�һ������RTC�����ж�*/ 
	s3c_rtc_alarmno = platform_get_irq(pdev, 0);
	if (s3c_rtc_alarmno < 0) {
		dev_err(&pdev->dev, "no irq for alarm\n");
		return -ENOENT;
	}

	pr_debug("s3c2410_rtc: tick irq %d, alarm irq %d\n",
		 s3c_rtc_tickno, s3c_rtc_alarmno);

	/* get the memory region */
	/*��ȡRTCƽ̨�豸��ʹ�õ�IO�˿���Դ*/ 
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get memory region resource\n");
		return -ENOENT;
	}
	/*����IO�˿���Դ��ռ�õ�IO�ռ�*/
	s3c_rtc_mem = request_mem_region(res->start, resource_size(res),
					 pdev->name);

	if (s3c_rtc_mem == NULL) {
		dev_err(&pdev->dev, "failed to reserve memory region\n");
		ret = -ENOENT;
		goto err_nores;
	}
	/*��IO�˿�ռ�õ�IO�ռ�ӳ�䵽�����ַ��s3c_rtc_base����������ַ����ʼ��ַ*/
	s3c_rtc_base = ioremap(res->start, resource_size(res));
	if (s3c_rtc_base == NULL) {
		dev_err(&pdev->dev, "failed ioremap()\n");
		ret = -EINVAL;
		goto err_nomap;
	}

	rtc_clk = clk_get(&pdev->dev, "rtc");//��ȡƽ̨ʱ��
	if (IS_ERR(rtc_clk)) {
		dev_err(&pdev->dev, "failed to find rtc clock source\n");
		ret = PTR_ERR(rtc_clk);
		rtc_clk = NULL;
		goto err_clk;
	}

	clk_enable(rtc_clk);

	/* check to see if everything is setup correctly */
	/*��RTCCON��0λ���в�����ʹ��RTC*/ 
	s3c_rtc_enable(pdev, 1);

	pr_debug("s3c2410_rtc: RTCCON=%02x\n",
		 readw(s3c_rtc_base + S3C2410_RTCCON));
	/*�õ�Դ����֧�ֻ��ѹ���*/
	device_init_wakeup(&pdev->dev, 1);

	/* register RTC and exit */
	/*ע��rtc�豸����Ϊ"s3c",��s3c_rtcops���rtc_class_ops���й���*/ 
	rtc = rtc_device_register("s3c", &pdev->dev, &s3c_rtcops,
				  THIS_MODULE);

	if (IS_ERR(rtc)) {
		dev_err(&pdev->dev, "cannot attach rtc\n");
		ret = PTR_ERR(rtc);
		goto err_nortc;
	}

	//�õ���ǰCPU�����ͣ���Ϊ��������ͨ�õģ�Ҳ����������s3c���͵Ĵ�����
	s3c_rtc_cpu_type = s3c_rtc_get_driver_data(pdev);

	/* Check RTC Time */
	//�õ�RTC�ĵ�ǰʱ��
	s3c_rtc_gettime(NULL, &rtc_tm);
	//���RTC�ĵ�ǰʱ����Ч������������
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
	//����CPU�����ͣ�����RTC����
	if (s3c_rtc_cpu_type != TYPE_S3C2410)
		rtc->max_user_freq = 32768;
	else
		rtc->max_user_freq = 128;

	if (s3c_rtc_cpu_type == TYPE_S3C2416 || s3c_rtc_cpu_type == TYPE_S3C2443) {
		tmp = readw(s3c_rtc_base + S3C2410_RTCCON);
		tmp |= S3C2443_RTCCON_TICSEL;
		writew(tmp, s3c_rtc_base + S3C2410_RTCCON);
	}

	 /*��rtc���rtc_device�����&pdev->dev->driver_data*/  
	platform_set_drvdata(pdev, rtc);

	/*��TICNT��7λ���в�����ʹ�ܽ���ʱ������Ĵ���*/
	s3c_rtc_setfreq(&pdev->dev, 1);

	//����RTC�����ж�
	ret = request_irq(s3c_rtc_alarmno, s3c_rtc_alarmirq,
			  0,  "s3c2410-rtc alarm", rtc);
	if (ret) {
		dev_err(&pdev->dev, "IRQ%d error %d\n", s3c_rtc_alarmno, ret);
		goto err_alarm_irq;
	}

	//����RTCʱ������ж�
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
//�������һ��2440��RTC����������s3c_rtcops��
static const struct rtc_class_ops s3c_rtcops = {
	.read_time	= s3c_rtc_gettime, //��ȡ��ǰʱ��
	.set_time	= s3c_rtc_settime, //���õ�ǰʱ��
	.read_alarm	= s3c_rtc_getalarm,//��ȡ����ʱ��
	.set_alarm	= s3c_rtc_setalarm,//���ñ���ʱ��
	.proc		= s3c_rtc_proc,
	.alarm_irq_enable = s3c_rtc_setaie, //��������RTCALM�Ĵ���
};

struct rtc_device *rtc_device_register(const char *name, struct device *dev,  
                    const struct rtc_class_ops *ops,  
                    struct module *owner)  
{  
    struct rtc_device *rtc;  
    int id, err;  
    /*Ϊidr(rtc_idr)�����ڴ�*/  
    if (idr_pre_get(&rtc_idr, GFP_KERNEL) == 0) {  
        err = -ENOMEM;  
        goto exit;  
    }  
    mutex_lock(&idr_lock);  
    /*����ID�Ŵ���id�У���ID�����ս���Ϊ��RTC�豸�Ĵ��豸��*/  
    err = idr_get_new(&rtc_idr, NULL, &id);  
    mutex_unlock(&idr_lock);  
    if (err < 0)  
        goto exit;  
    id = id & MAX_ID_MASK;  
    /*ΪRTC�ṹ�����ڴ�*/  
    rtc = kzalloc(sizeof(struct rtc_device), GFP_KERNEL);  
    if (rtc == NULL) {  
        err = -ENOMEM;  
        goto exit_idr;  
    }  
    rtc->id = id;  
    /*ָ��ԭʼ����������*/  
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
    /*rtc->dev.devt = MKDEV(MAJOR(rtc_devt),rtc->id); cdev_init(&rtc->char_dev,&rtc_dev_fops);����rtc_devt�Ǵӵ���alloc_chrdev_regionʱ��õ�*/  
    rtc_dev_prepare(rtc);  
    /*ע���RTC�豸rtc->dev*/  
    err = device_register(&rtc->dev);  
    if (err)  
        goto exit_kfree;  
    /*cdev_add(&rtc->chr_dev,rtc->dev.devt,1);��rtc->chrdevע�ᵽϵͳ��*/  
    rtc_dev_add_device(rtc);  
    /*��/sys����������ļ�*/  
    rtc_sysfs_add_device(rtc);  
    /*��/proc�д��������"driver/rtc"*/  
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























