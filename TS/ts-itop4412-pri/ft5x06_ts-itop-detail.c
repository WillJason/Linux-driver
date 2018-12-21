/*
	FT5x06ϵ��ICs�ǵ�оƬ����ʽ������������IC������һ�����õ�8λ΢��������Ԫ��MCU����
���û����ݵķ���������ϵ��໥�ĵ���ʽ������壬��֧�������Ķ�㴥�����ܡ�FT5x06�����û��Ѻ�
������Ĺ��ܣ������Ӧ��������Яʽ�豸���������ʽ�绰���ƶ��������豸���������ͱʼǱ�����
���ԡ�FT5x06ϵ��IC����FT5206/FT5306/FT5406��
	��FT5X06��datasheet�У����ǿ��Կ�����FT5X06�ȿ��Թ�����SPI�Ľӿڷ�ʽ��Ҳ���Թ�����I2C�Ľӿ�
��ʽ�����ܹ�����SPI�����ǹ�����I2C����Ӳ���Ľӿ��������˵��������ļ������ƿڣ�������ҪҪ�ӵġ�
	1����INT���ţ��������һ���ж��źţ�������֪ͨHOST��FT5X06�Ѿ�׼���ã����Խ��ж������ˡ�
	2����WAKE���ţ����������Ҫ�������ǽ�FT5X06��˯��״̬ת��������״̬��
	3����/RST���ţ�FT5X06��оƬ��λ�źš�
	����FT5406�����ֲ��ϵ�ָ��������˽����������ʵ�ֵ������Ķ�㴥������ʵ�ܼ򵥣���Ҫ��Ҫ��
����IC FT5406 �ܹ����������ݣ�����������������֧�ֵ�����2�����ϣ���FT5406 ���Բ���5��������,
��д����ʱ��ֻҪȥ��ȡ�⼸��������ݣ�Ȼ���ϱ��Ϳ����ˡ�
	
*/
/*
  I2C��������Ҫ���ݾ����ARMоƬ��һ����˵��ICԭ����һ��Ὣ��linux��bsp�ж�����I2C����������
�����ֲ���Ҫ����ȥд�ģ�����ֻ��Ҫ��FT5X06��BSP���е�I2C����ƥ�������ͺ��ˡ�
*/
//��Ҫ���i2c��ƽ̨�豸��Ϣ
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
Linux-driver/i2c/i2c-tiny4412-pri���Ѿ���i2c-s3c2410.c��������probe�����˷��������Ĳ����ظ�
ft5x06_ts.c��������probe��������
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

	//����������Ƿ�֧��I2C_FUNC_I2CͨѶ��ʽ��ͨ��һ��bitmap�����ߵ����߸�I2C adapter֧�ֵĹ��ܣ�
	//����I2C_FUNC_I2C��֧�ִ�ͳ��I2C���ܣ�����������
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	//��kzalloc�����ڴ��ʱ�� Ч����ͬ�������� kmalloc() ����ռ� , Ȼ���� memset() ����ʼ�� ,
	//���������Ԫ�ض�����ʼ��Ϊ 0.
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	pdata = client->dev.platform_data;//��ȡƽ̨����
	if (!pdata) {
		dev_err(&client->dev, "failed to get platform data\n");
		goto exit_no_pdata;
	}
	/*
	�ֱ�ͨ��i2c������IC�е���Ļ�ķֱ����Լ�ѹ��ֵ�������жϺŶ�ȡ��
	������һ���ṹ���С�
	*/
	ts->screen_max_x = pdata->screen_max_x;
	ts->screen_max_y = pdata->screen_max_y;
	ts->pressure_max = pdata->pressure_max;

	ts->gpio_irq = pdata->gpio_irq;
	if (ts->gpio_irq != -EINVAL) {
		client->irq = gpio_to_irq(ts->gpio_irq);//�����жϺ�
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
	��ʼ���������У�����ԴԴ���ϵĴӴ���ic�л�ȡ���ݡ�ft5x0x_ts_pen_irq_work�ȴ��жϲ�������ȡ��
	��ic�л�ȡ׼���ϱ���ϵͳ�˵����ݡ���ʹ���жϣ����������ڼ�⵽�����ʱ��Ҳ�����жϷ�����
	ʱ���ܹ��ϱ����ݡ�
	*/
	INIT_WORK(&ts->work, ft5x0x_ts_pen_irq_work);
	this_client = client;
	i2c_set_clientdata(client, ts);

	//��������������У�����ִ�й������е�ʱ��Ϳ��Ի�ȡ�ñ�����ֵ���ҵ���ӦworkҪִ�е�function
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

	//������ϵͳ
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
	��д������Ҫ����Ϣ�����������ƣ�������Ϣ��ʹ�õ�����Э�飬����ID�ţ���ƷID��
	�汾�š�
	*/
	input_dev->name = FT5X0X_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x12FA;
	input_dev->id.product = 0x2143;
	input_dev->id.version = 0x0100;

	//ע�������豸
	err = input_register_device(input_dev);
	if (err) {
		input_free_device(input_dev);
		dev_err(&client->dev, "failed to register input device %s, %d\n",
				dev_name(&client->dev), err);
		goto exit_input_dev_alloc_failed;
	}

	msleep(3);
	err = ft5x0x_read_fw_ver(&val);//��ȡоƬID
	if (err < 0) {
		dev_err(&client->dev, "chip not found\n");
		goto exit_irq_request_failed;
	}

	//ע���жϷ�������ft5x0x_ts_interrupt���ҹ����ǰ�Ƿ���𣬵���queue_work(ts->queue, &ts->work);
	//������ȡic�����жϵ����ݣ����ϱ���input��ϵͳ
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
//��������������ŵķ�����������ˣ����ǵ�i2cʹ�õ��Ǿ�̬ע��
/*
��Ļ������--->ft5x06�����ж�--->�����ж���ں���ft5x0x_ts_interrupt--->�ر��ж�--->�ȴ�����--->
�õ�CPU��Դ����ft5x0x_ts_pen_irq_workִ�й���--->�����ж�--���δ��������
*/



