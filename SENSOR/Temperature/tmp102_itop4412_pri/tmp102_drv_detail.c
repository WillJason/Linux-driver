/*
硬件平台：itop4412
系统：linux-3.0.5
	TMP102 器件是一款数字温度传感器，非常适用于作为需要高精度的 NTC/PTC 热敏电阻的替代品。
器件在未经校准或无外部组件信号调节的情况下可提供的精度为	 ±0.5°C。器件温度传感器为
高度线性化产品，无需复杂计算或查表即可得知温度。片上 12 位 ADC 具备最低 0.0625°C 的分辨率。
具有 I2C/SMBus 接口且工作电压为 1.4V 的 ±1°C 温度传感器，支持报警功能。

tmp102驱动位于drivers/hwmon目录。驱动文件为drivers/hwmon/tmp102.c。
*/
//先来看一下I2C总线的主机控制器相关的平台设备注册的信息（我们假设使用的I2C-4）
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
//信息添加到入口函数
static struct platform_device *smdk4x12_devices[] __initdata = {
{
		&s3c_device_i2c4,
}
static void __init smdk4x12_machine_init(void)
{
		s3c_i2c4_set_platdata(NULL);
		platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
}

//再来看一下tmp102外设注册的信息
//首先内核里的tmp102外设驱动配置选项:
Symbol: SENSORS_TMP102 [=y]
Type  : tristate
Prompt: Texas Instruments TMP102
  Location:
    -> Device Drivers
      -> Hardware Monitoring support (HWMON [=y])
  Defined at drivers/hwmon/Kconfig:1486
  Depends on: HWMON [=y] && I2C [=y] && (THERMAL [=y] || !THERMAL_OF [=y])
//注册信息设置 
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
//信息同样添加到入口函数
static void __init smdk4x12_machine_init(void)
{
	i2c_register_board_info(4, i2c_devs4, ARRAY_SIZE(i2c_devs4));
}

//设备信息注册分析完毕，接下来分析驱动
//主机控制器驱动
/*file:i2c-s3c2410.c */
//在别的目录下已经有分析，这里不再重复

//外设驱动相关
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
samsung tmp102.c外设驱动probe函数分析
-----------------------------------------------------------------------------------*/
static int __devinit tmp102_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct tmp102 *tmp102;
	int status;
	//检测适配器是否支持处理SMBus read_word_data这个命令，通过一个bitmap，告诉调用者该I2C adapter支
	//持的功能，
	//SMBUS相关的功能
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "adapter doesn't support SMBus word "
			"transactions\n");
		return -ENODEV;
	}
	//分配空间，kzalloc实现了kmalloc以及memset功能一个函数起到两个函数作用
	tmp102 = kzalloc(sizeof(*tmp102), GFP_KERNEL);
	if (!tmp102) {
		dev_dbg(&client->dev, "kzalloc failed\n");
		return -ENOMEM;
	}
	//将数据空间的指针赋值给client结构体中，方便以后i2c_get_clientdata获取数据信息
	i2c_set_clientdata(client, tmp102);
	//芯片相关操作：读取旧配置信息
	status = tmp102_read_reg(client, TMP102_CONF_REG);
	if (status < 0) {
		dev_err(&client->dev, "error reading config register\n");
		goto fail_free;
	}
	tmp102->config_orig = status;//保存旧配置，以便出错恢复
	status = tmp102_write_reg(client, TMP102_CONF_REG, TMP102_CONFIG);//配置
	if (status < 0) {
		dev_err(&client->dev, "error writing config register\n");
		goto fail_restore_config;
	}
	status = tmp102_read_reg(client, TMP102_CONF_REG);//读配置信息然后校对
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
	//注册hwmon（硬件监视器）设备
	tmp102->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(tmp102->hwmon_dev)) {
		dev_dbg(&client->dev, "unable to register hwmon device\n");
		status = PTR_ERR(tmp102->hwmon_dev);
		goto fail_remove_sysfs;
	}

	dev_info(&client->dev, "initialized\n");

	return 0;
}
//这里用到了一个注册函数sysfs_create_group，这是sysfs接口用到的，接下来介绍它
/*
	在调试驱动，可能需要对驱动里的某些变量进行读写，或函数调用。可通过sysfs接口
创建驱动对应的属性，使得可以在用户空间通过sysfs接口的show函数和store函数与硬件交互；
*/
//sysfs接口在linux中表现为sys/目录下的一个文件，比如temp1_input，我们可以设置这些
//接口的属性、权限和对应的操作函数
//这里我们就调用SENSOR_DEVICE_ATTR来设置接口（就是一个文件，称接口更适合）的属性
//宏声明有五个参数，分别是名称、权限位、读函数、写函数、索引值（这里用来映射
//tmp102_reg对应的寄存器）
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, tmp102_show_temp, NULL , 0);//读取
static SENSOR_DEVICE_ATTR(temp1_max_hyst, S_IWUSR | S_IRUGO, tmp102_show_temp,
			  tmp102_set_temp, 1);//读取、设置报警最小值
static SENSOR_DEVICE_ATTR(temp1_max, S_IWUSR | S_IRUGO, tmp102_show_temp,
			  tmp102_set_temp, 2);//读取、设置报警最大值
//另外总线对应BUS_ATTR、设备驱动对应DRIVER_ATTR、类(class)对应CLASS_ATTR，均在
//kernel/include/linux/device.h下定义
//可以简单的看一下SENSOR_DEVICE_ATTR的实现，说白了就是一层层的对属性进行包装
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
//.attr包含了一个的最基本的属性：名字和权限。
//然后我们就可以定义一个属性结构体数组，将我们想要的创建的接口（文件）的属性都
//添加进来，可以是一个或者多个接口（文件），方便后续我们一起统一创建：
static struct attribute *tmp102_attributes[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_max_hyst.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	NULL
};
static const struct attribute_group tmp102_attr_group = {
	.attrs = tmp102_attributes,
};
//然后就可以调用创建的函数sysfs_create_group
//sysfs_create_group()在kobj目录下创建一个属性集合，并显示集合中的属性文件。
//如果文件已存在，会报错。
status = sysfs_create_group(&client->dev.kobj, &tmp102_attr_group);
//如果想要删除接口，就调用
sysfs_remove_group(&pdev->dev.kobj,&gpio_keys_attr_group);
//sysfs_remove_group()在kobj目录下删除一个属性集合，并删除集合中的属性文件
//sysfs接口就介绍完了。

//然后看一下temp1_input、temp1_max_hyst、temp1_max三个接口（文件）的读函数和写函数
static ssize_t tmp102_show_temp(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	//获取顶层包装的属性结构体指针
	struct sensor_device_attribute *sda = to_sensor_dev_attr(attr);
	//调用update函数获取当前温度、高温报警、低温报警全部信息，然后放到
	//缓冲区返回整个结构体
	struct tmp102 *tmp102 = tmp102_update_device(to_i2c_client(dev));
	//根据接口的索引值选择输出缓冲区内对应的值
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

/*在3.0内核中，hwmon非常简单，对外只有一个函数：
struct device *hwmon_device_register(struct device *dev)，
调用这个接口实现的hwmon设备，在/sys/class/hwmon/下会多出个文件夹，
都是按照hwmon 0/1/2/3 编号，当有些注册失败时，编号就会发生变化，
用脚本去自动采集这些数据时，每次都有根据实际情况修改脚本，非常不便，
不过这个接口是调用了hwmon_device_register_with_groups(struct device
*dev, const char *name, void *drvdata, const struct attribute_group **groups)，
可以将此接口导出，将第二个参数赋成设备名，在/sys/class/hwmon/下的
文件夹就是所定义的名字了。

在3.13内核中，hwmon改变很大，终于有了可以更多选择的接口了，使用也更方便，只需调用一个函数：
hwmon_dev = devm_hwmon_device_register_with_groups(dev, client->name,
  data, lm73_groups);
if (IS_ERR(hwmon_dev))
return PTR_ERR(hwmon_dev);
甚至连定义好的节点都会帮我们初始化好，省去了自己将lm73_groups的属性节点注册。
*/

//创建sysfs接口后，就可以在adb shell 终端查看到和操作接口了。
//当我们将数据echo 到接口中时，在用户空间完成了一次 write 操作，
//对应到 kernel，调用了驱动中的”store”。当我们cat一个接口时则会调用"show".
//这样就建立了android层到 kernel的桥梁，操作的细节在"show"和"store"中完成的。

//测试温度：
cat /sys/class/hwmon/hwmon0/device/temp1_input

























