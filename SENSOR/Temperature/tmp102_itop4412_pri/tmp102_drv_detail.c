/*
Ӳ��ƽ̨��itop4412
ϵͳ��linux-3.0.5
	TMP102 ������һ�������¶ȴ��������ǳ���������Ϊ��Ҫ�߾��ȵ� NTC/PTC ������������Ʒ��
������δ��У׼�����ⲿ����źŵ��ڵ�����¿��ṩ�ľ���Ϊ	 ��0.5��C�������¶ȴ�����Ϊ
�߶����Ի���Ʒ�����踴�Ӽ�������ɵ�֪�¶ȡ�Ƭ�� 12 λ ADC �߱���� 0.0625��C �ķֱ��ʡ�
���� I2C/SMBus �ӿ��ҹ�����ѹΪ 1.4V �� ��1��C �¶ȴ�������֧�ֱ������ܡ�

tmp102����λ��drivers/hwmonĿ¼�������ļ�Ϊdrivers/hwmon/tmp102.c��
*/
//������һ��I2C���ߵ�������������ص�ƽ̨�豸ע�����Ϣ�����Ǽ���ʹ�õ�I2C-4��
struct s3c2410_platform_i2c default_i2c_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 100*1000,
	.sda_delay	= 100,
};//
//arch/arm/platform-samsung/Dev-i2c4.c
static struct resource s3c_i2c_resource[] = {
	[0] = {
		.start	= S3C_PA_IIC4,
		.end	= S3C_PA_IIC4 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_IIC4,
		.end	= IRQ_IIC4,
		.flags	= IORESOURCE_IRQ,
	},
};
struct platform_device s3c_device_i2c4 = {
	.name		= "s3c2440-i2c",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(s3c_i2c_resource),
	.resource	= s3c_i2c_resource,
};
void __init s3c_i2c4_set_platdata(struct s3c2410_platform_i2c *pd)
{
	struct s3c2410_platform_i2c *npd;

	if (!pd) {
		pd = &default_i2c_data;
		pd->bus_num = 4;
	}

	npd = s3c_set_platdata(pd, sizeof(struct s3c2410_platform_i2c),
			       &s3c_device_i2c4);

	if (!npd->cfg_gpio)
		npd->cfg_gpio = s3c_i2c4_cfg_gpio;
}
//��Ϣ��ӵ���ں���
static struct platform_device *smdk4x12_devices[] __initdata = {
{
		&s3c_device_i2c4,
}
static void __init smdk4x12_machine_init(void)
{
		s3c_i2c4_set_platdata(NULL);
		platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
}

//������һ��tmp102����ע�����Ϣ
//�����ں����tmp102������������ѡ��:
Symbol: SENSORS_TMP102 [=y]
Type  : tristate
Prompt: Texas Instruments TMP102
  Location:
    -> Device Drivers
      -> Hardware Monitoring support (HWMON [=y])
  Defined at drivers/hwmon/Kconfig:1486
  Depends on: HWMON [=y] && I2C [=y] && (THERMAL [=y] || !THERMAL_OF [=y])
//ע����Ϣ���� 
#ifdef CONFIG_SENSOR_TMP102
static struct tmp102_data tmp102_pdata = {
	.xxx		= 0,
	.xxx		= 0,
};
#endif
/* I2C4 */
#define TMP102_DRIVER_NAME "tmp102"
static struct i2c_board_info i2c_devs4[] __initdata = {
#ifdef CONFIG_SENSOR_TMP102
	{
		I2C_BOARD_INFO(TMP102_DRIVER_NAME, 0x49),
		.platform_data	= &tmp102_pdata,
	},
#endif
};
//��Ϣͬ����ӵ���ں���
static void __init smdk4x12_machine_init(void)
{
	i2c_register_board_info(4, i2c_devs4, ARRAY_SIZE(i2c_devs4));
}

//�豸��Ϣע�������ϣ���������������
//��������������
/*file:i2c-s3c2410.c */
//�ڱ��Ŀ¼���Ѿ��з��������ﲻ���ظ�

//�����������
/*file:tmp102.c */
static const struct i2c_device_id tmp102_id[] = {
	{ "tmp102", 0 },
	{ }
};
static struct i2c_driver tmp102_driver = {
	.driver.name	= DRIVER_NAME,
	.driver.pm	= TMP102_DEV_PM_OPS,
	.probe		= tmp102_probe,
	.remove		= __devexit_p(tmp102_remove),
	.id_table	= tmp102_id,
};
static int __init tmp102_init(void)
{
	return i2c_add_driver(&tmp102_driver);
}
static void __exit tmp102_exit(void)
{
	i2c_del_driver(&tmp102_driver);
}
/*-----------------------------------------------------------------------------------
samsung tmp102.c��������probe��������
-----------------------------------------------------------------------------------*/
static int __devinit tmp102_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct tmp102 *tmp102;
	int status;
	//����������Ƿ�֧�ִ���SMBus read_word_data������ͨ��һ��bitmap�����ߵ����߸�I2C adapter֧
	//�ֵĹ��ܣ�
	//SMBUS��صĹ���
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "adapter doesn't support SMBus word "
			"transactions\n");
		return -ENODEV;
	}
	//����ռ䣬kzallocʵ����kmalloc�Լ�memset����һ��������������������
	tmp102 = kzalloc(sizeof(*tmp102), GFP_KERNEL);
	if (!tmp102) {
		dev_dbg(&client->dev, "kzalloc failed\n");
		return -ENOMEM;
	}
	//�����ݿռ��ָ�븳ֵ��client�ṹ���У������Ժ�i2c_get_clientdata��ȡ������Ϣ
	i2c_set_clientdata(client, tmp102);
	//оƬ��ز�������ȡ��������Ϣ
	status = tmp102_read_reg(client, TMP102_CONF_REG);
	if (status < 0) {
		dev_err(&client->dev, "error reading config register\n");
		goto fail_free;
	}
	tmp102->config_orig = status;//��������ã��Ա����ָ�
	status = tmp102_write_reg(client, TMP102_CONF_REG, TMP102_CONFIG);//����
	if (status < 0) {
		dev_err(&client->dev, "error writing config register\n");
		goto fail_restore_config;
	}
	status = tmp102_read_reg(client, TMP102_CONF_REG);//��������ϢȻ��У��
	if (status < 0) {
		dev_err(&client->dev, "error reading config register\n");
		goto fail_restore_config;
	}
	status &= ~TMP102_CONFIG_RD_ONLY;
	if (status != TMP102_CONFIG) {
		dev_err(&client->dev, "config settings did not stick\n");
		status = -ENODEV;
		goto fail_restore_config;
	}
	tmp102->last_update = jiffies - HZ;
	mutex_init(&tmp102->lock);

	status = sysfs_create_group(&client->dev.kobj, &tmp102_attr_group);
	if (status) {
		dev_dbg(&client->dev, "could not create sysfs files\n");
		goto fail_restore_config;
	}
	//ע��hwmon��Ӳ�����������豸
	tmp102->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(tmp102->hwmon_dev)) {
		dev_dbg(&client->dev, "unable to register hwmon device\n");
		status = PTR_ERR(tmp102->hwmon_dev);
		goto fail_remove_sysfs;
	}

	dev_info(&client->dev, "initialized\n");

	return 0;
}
//�����õ���һ��ע�ắ��sysfs_create_group������sysfs�ӿ��õ��ģ�������������
/*
	�ڵ���������������Ҫ���������ĳЩ�������ж�д���������á���ͨ��sysfs�ӿ�
����������Ӧ�����ԣ�ʹ�ÿ������û��ռ�ͨ��sysfs�ӿڵ�show������store������Ӳ��������
*/
//sysfs�ӿ���linux�б���Ϊsys/Ŀ¼�µ�һ���ļ�������temp1_input�����ǿ���������Щ
//�ӿڵ����ԡ�Ȩ�޺Ͷ�Ӧ�Ĳ�������
//�������Ǿ͵���SENSOR_DEVICE_ATTR�����ýӿڣ�����һ���ļ����ƽӿڸ��ʺϣ�������
//������������������ֱ������ơ�Ȩ��λ����������д����������ֵ����������ӳ��
//tmp102_reg��Ӧ�ļĴ�����
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, tmp102_show_temp, NULL , 0);//��ȡ
static SENSOR_DEVICE_ATTR(temp1_max_hyst, S_IWUSR | S_IRUGO, tmp102_show_temp,
			  tmp102_set_temp, 1);//��ȡ�����ñ�����Сֵ
static SENSOR_DEVICE_ATTR(temp1_max, S_IWUSR | S_IRUGO, tmp102_show_temp,
			  tmp102_set_temp, 2);//��ȡ�����ñ������ֵ
//�������߶�ӦBUS_ATTR���豸������ӦDRIVER_ATTR����(class)��ӦCLASS_ATTR������
//kernel/include/linux/device.h�¶���
//���Լ򵥵Ŀ�һ��SENSOR_DEVICE_ATTR��ʵ�֣�˵���˾���һ���Ķ����Խ��а�װ
#define SENSOR_DEVICE_ATTR(_name, _mode, _show, _store, _index)	\
struct sensor_device_attribute sensor_dev_attr_##_name		\
	= SENSOR_ATTR(_name, _mode, _show, _store, _index)

#define SENSOR_ATTR(_name, _mode, _show, _store, _index)	\
	{ .dev_attr = __ATTR(_name, _mode, _show, _store),	\
	  .index = _index }
	  
#define __ATTR(_name,_mode,_show,_store) { \
	.attr = {.name = __stringify(_name), .mode = _mode },	\
	.show	= _show,					\
	.store	= _store,					\
}
//.attr������һ��������������ԣ����ֺ�Ȩ�ޡ�
//Ȼ�����ǾͿ��Զ���һ�����Խṹ�����飬��������Ҫ�Ĵ����Ľӿڣ��ļ��������Զ�
//��ӽ�����������һ�����߶���ӿڣ��ļ����������������һ��ͳһ������
static struct attribute *tmp102_attributes[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_max_hyst.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	NULL
};
static const struct attribute_group tmp102_attr_group = {
	.attrs = tmp102_attributes,
};
//Ȼ��Ϳ��Ե��ô����ĺ���sysfs_create_group
//sysfs_create_group()��kobjĿ¼�´���һ�����Լ��ϣ�����ʾ�����е������ļ���
//����ļ��Ѵ��ڣ��ᱨ��
status = sysfs_create_group(&client->dev.kobj, &tmp102_attr_group);
//�����Ҫɾ���ӿڣ��͵���
sysfs_remove_group(&pdev->dev.kobj,&gpio_keys_attr_group);
//sysfs_remove_group()��kobjĿ¼��ɾ��һ�����Լ��ϣ���ɾ�������е������ļ�
//sysfs�ӿھͽ������ˡ�

//Ȼ��һ��temp1_input��temp1_max_hyst��temp1_max�����ӿڣ��ļ����Ķ�������д����
static ssize_t tmp102_show_temp(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	//��ȡ�����װ�����Խṹ��ָ��
	struct sensor_device_attribute *sda = to_sensor_dev_attr(attr);
	//����update������ȡ��ǰ�¶ȡ����±��������±���ȫ����Ϣ��Ȼ��ŵ�
	//���������������ṹ��
	struct tmp102 *tmp102 = tmp102_update_device(to_i2c_client(dev));
	//���ݽӿڵ�����ֵѡ������������ڶ�Ӧ��ֵ
	return sprintf(buf, "%d\n", tmp102->temp[sda->index]);
}

static ssize_t tmp102_set_temp(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct sensor_device_attribute *sda = to_sensor_dev_attr(attr);
	struct i2c_client *client = to_i2c_client(dev);
	struct tmp102 *tmp102 = i2c_get_clientdata(client);
	long val;
	int status;

	if (strict_strtol(buf, 10, &val) < 0)
		return -EINVAL;
	val = SENSORS_LIMIT(val, -256000, 255000);

	mutex_lock(&tmp102->lock);
	tmp102->temp[sda->index] = val;
	status = tmp102_write_reg(client, tmp102_reg[sda->index],
				  tmp102_mC_to_reg(val));
	mutex_unlock(&tmp102->lock);
	return status ? : count;
}

/*��3.0�ں��У�hwmon�ǳ��򵥣�����ֻ��һ��������
struct device *hwmon_device_register(struct device *dev)��
��������ӿ�ʵ�ֵ�hwmon�豸����/sys/class/hwmon/�»������ļ��У�
���ǰ���hwmon 0/1/2/3 ��ţ�����Щע��ʧ��ʱ����žͻᷢ���仯��
�ýű�ȥ�Զ��ɼ���Щ����ʱ��ÿ�ζ��и���ʵ������޸Ľű����ǳ����㣬
��������ӿ��ǵ�����hwmon_device_register_with_groups(struct device
*dev, const char *name, void *drvdata, const struct attribute_group **groups)��
���Խ��˽ӿڵ��������ڶ������������豸������/sys/class/hwmon/�µ�
�ļ��о���������������ˡ�

��3.13�ں��У�hwmon�ı�ܴ��������˿��Ը���ѡ��Ľӿ��ˣ�ʹ��Ҳ�����㣬ֻ�����һ��������
hwmon_dev = devm_hwmon_device_register_with_groups(dev, client->name,
  data, lm73_groups);
if (IS_ERR(hwmon_dev))
return PTR_ERR(hwmon_dev);
����������õĽڵ㶼������ǳ�ʼ���ã�ʡȥ���Լ���lm73_groups�����Խڵ�ע�ᡣ
*/

//����sysfs�ӿں󣬾Ϳ�����adb shell �ն˲鿴���Ͳ����ӿ��ˡ�
//�����ǽ�����echo ���ӿ���ʱ�����û��ռ������һ�� write ������
//��Ӧ�� kernel�������������еġ�store����������catһ���ӿ�ʱ������"show".
//�����ͽ�����android�㵽 kernel��������������ϸ����"show"��"store"����ɵġ�

//�����¶ȣ�
cat /sys/class/hwmon/hwmon0/device/temp1_input

























