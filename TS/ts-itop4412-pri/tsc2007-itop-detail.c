/*
	TSC2007 ������ TI ��˾�Ƴ�����һ�� 4 ���� ������ �������������봥�������Ӻ�һ���бʻ���ָ��
��������ʱ�������Ѹ�ٵõ��õ��λ���źţ��Ӷ��ﵽ�ڴ�����������Ѱַ��Ŀ�ġ�
����TSC2007 �ǵ��͵��𲽱ƽ��Ĵ����� A �� D �任������ṹ�Ե����ٷֲ�Ϊ������ͬʱ�ڲ�������ȡ
�������ֹ��ܡ� TSC2007 ����Ƭ���¶Ȳ���������ѹ��������Ԥ��������ܡ����� I 2C �ӿڣ����Ա�׼ģʽ������ģʽ�ͳ�����ģʽ�������ݴ�����ͨѶ�����пɱ�� 8 λ�� 12 λ�ֱ��ʣ�
��������(TI)��˾��ADS7843/45/46��TSC2046��TSC2003/4/5/6/7�ʹ�����������(TSC)���д������������ź�һ�������Ǵ���������ģ���������ţ�����TSC2046�ĵ�ص�ѹ���(VBAT)��TSC2007��AUX�ȸ����������š��ڴ��������빤����ͬʱ������δ�����������ڼ䣬�ɽ�������Щ�����������ż��ϵͳ�ĵ�ص�ѹ��������ѹ�źš�
�����ڴ����������֮��һ�㶼��Ҫȷ���������� X �� Y ����������Ա�ϵͳ����������Ӧ����Ϣ��Ϊ�ˣ����ʱ����Ҫ�� TSC2007 ���ж�д������
*/
/*
  I2C��������Ҫ���ݾ����ARMоƬ��һ����˵��ICԭ����һ��Ὣ��linux��bsp�ж�����I2C����������
�����ֲ���Ҫ����ȥд�ģ�����ֻ��Ҫ��FT5X06��BSP���е�I2C����ƥ�������ͺ��ˡ�
*/
//�ں�����
Device Drivers --->
	Input device support--->
		Touchscreens--->
			TSC2007 based touchscreens
//��Ҫ���i2c��ƽ̨�豸��Ϣ			
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
//�������������
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
//�����������
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
Linux-driver/i2c/i2c-tiny4412-pri���Ѿ���i2c-s3c2410.c��������probe�����˷��������Ĳ����ظ�
tsc2007.c��������probe��������
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
	//����������Ƿ�֧�ִ���SMBus read_word_data������ͨ��һ��bitmap�����ߵ����߸�I2C adapter֧
	//�ֵĹ��ܣ�
	//SMBUS��صĹ���
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	//��kzalloc�����ڴ��ʱ�� Ч����ͬ�������� kmalloc() ����ռ� , Ȼ���� memset() ����ʼ�� ,
	//���������Ԫ�ض�����ʼ��Ϊ 0.
	ts = kzalloc(sizeof(struct tsc2007), GFP_KERNEL);
	//����input���Ĳ����룩����һ��input_dev�ṹ�塣
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->client = client;
	ts->irq = client->irq;
	ts->input = input_dev;
	/*
	��ʼ���������У�����ԴԴ���ϵĴӴ���ic�л�ȡ���ݡ�tsc2007_work�ȴ��жϲ�������ȡ��
	��ic�л�ȡ׼���ϱ���ϵͳ�˵����ݡ���ʹ���жϣ����������ڼ�⵽�����ʱ��Ҳ�����жϷ�����
	ʱ���ܹ��ϱ����ݡ�
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
	��ЩAndroid 4.0ϵͳ�ϲ����ñ������SLOT��ʽ���㣬��ʱ���������ɲ�ȡϵͳ��ͳ�ı��㷽ʽ��Android�ϲ���ܻὫ�ϱ��������ʶ��������꣬������������������뽫GTP_ICS_SLOT_REPORT��򿪣������㷽ʽ�л���SLOT��ʽ���ɡ���ϸ���ݲο�linux������ϵͳ��Android�ϲ�inputreader.cpp�й����ϱ��¼���������ϡ�
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

	////��input���Ĳ�ע��input_dev�ṹ��
	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	i2c_set_clientdata(client, ts);

	/* add by cym 20130417 */
	//����proc�ļ��������Ѿ���proc_create()���
	ts_proc_entry = create_proc_entry("driver/micc_ts", 0, NULL);   
	if (ts_proc_entry) {   
		ts_proc_entry->write_proc = ts_proc_write;   
	}
	/* end add */

	/* add by cym 20141202 */
	//Android����������earlysuspend˯��ģʽ
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
�����ع�һ�����������̣�
���Ƚ� TSC2007 ��� PowerDown ģʽ---->��Ļ������--->tsc2007�����ж�--->�����ж���ں���tsc2007_irq
---->�ر��жϣ���ʱ���ύ����---->δ����������ύ����������ʹ���ж�
*/
