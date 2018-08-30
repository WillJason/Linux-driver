/*
硬件平台：itop4412
系统：linux-4.14.2
在设备树中直接使用中断资源，实现ADC采集底板上滑动变阻器的电压。

在Linux kernel 3.2已经引入了IIO子文件系统，但是驱动不多，毕竟刚开始的一个新的子文件系统 。
在kernel 3.x版本中目录在drivers/staging/iio目录下，到了kernel 4.x版本下被移到了drivers/iio，
显然大家认可了这个子文件系统的框架。

IIO子系统全称是 Industrial I/O subsystem（工业 I/O 子系统），根据内核说明文档的描述：
工业I/O子系统旨在提供那些在某种意义上作为模数转换器（ADC）的设备支持。此子系统的目的在于填补那些分类时处
在hwmon（硬件监视器）和输入子系统之间的设备类型。在某些情况下，IIO和hwmon、Input之间的相当大的重叠。

属于IIO的设备如下（参考ADI的WiKi）：
ADCs(模数转换器)
加速度传感器
陀螺仪
IMUs(惯性测量单位)
电容-数字转换器(CDCs)
压力、温度和光线传感器
从源码的目录中可以看出，还有：磁力计传感器、电能功率计、旋变数字转换器
可以看出这些外设的确都是工业领域使用的，当然其中的部分在其他领域也用到了，比如
加速度传感器、陀螺仪、温度和光线传感器在移动设备（手机、平板）中也有广泛应用。
 
iio子系统采用环形缓冲区，环形缓冲区本质是一个数据结构（单一，固定大小，可调并首尾相连），
这种结构非常适合缓冲数据流。这些缓冲区通常用来解决生产者消费者问题，在一些应用中，
它被设计成生产者会(例如一个ADC)覆盖消费者(例如一个用户空间应用程序)无法暂时处理的过期数据。
但是通常这种缓冲会被设置为适当的大小，以使这种情况不会发生。


*/
//对应的设备驱动源文件: drivers/iio/adc/exynos_adc.c
//exynos_adc.c源文件里设备树相关的主要内容:
static struct platform_driver exynos_adc_driver = {
	.probe		= exynos_adc_probe,
	.remove		= exynos_adc_remove,
	.driver		= {
		.name	= "exynos-adc",
		.of_match_table = exynos_adc_match,
		.pm	= &exynos_adc_pm_ops,
	},
};
static const struct of_device_id exynos_adc_match[] = {
 {
		.compatible = "samsung,exynos-adc-v1",
		.data = &exynos_adc_v1_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_adc_match);
//匹配上后， 设备驱动里exynos_adc_probe函数就会被触发调用. 在probe函数里就会取出设备树里提供的硬件资源.
static int exynos_adc_probe(struct platform_device *pdev)
{
	struct exynos_adc *info = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct s3c2410_ts_mach_info *pdata = dev_get_platdata(&pdev->dev);//使用设备树的方式, pdata应为NULL;
	struct iio_dev *indio_dev = NULL;
	struct resource	*mem;
	bool has_ts = false;
	int ret = -ENODEV;
	int irq;

	 /* 动态申请iio设备 */
	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(struct exynos_adc));
	if (!indio_dev) {
		dev_err(&pdev->dev, "failed allocating iio device\n");
		return -ENOMEM;
	}

	/*
	return (char *)indio_dev + ALIGN(sizeof(struct iio_dev), IIO_ALIGN);
	在linux内核中,经产会看到对齐ALIGN的调用,常见的如内存管理中page对齐，net_device中私有数据的获取等
	先说ALIGN的用法，ALIGN(x,a) 是为了使x以a为边界对齐，实现原理是给x加上一个最小的数，
	使x以a为边界对齐。举个例子，a = 8, x=0, ALIGN(x,a) 运算结果为0； a = 8, x = 3， 运算结果为8；
	a = 8, x = 11, 运算结果为16。 
	*/
	info = iio_priv(indio_dev);

	info->data = exynos_adc_get_data(pdev);//获取设备ID表结构以及平台设备资源
	if (!info->data) {
		dev_err(&pdev->dev, "failed getting exynos_adc_data\n");
		return -EINVAL;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	info->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(info->regs))
		return PTR_ERR(info->regs);


	if (info->data->needs_adc_phy) {
		info->pmu_map = syscon_regmap_lookup_by_phandle(
					pdev->dev.of_node,
					"samsung,syscon-phandle");
		if (IS_ERR(info->pmu_map)) {
			dev_err(&pdev->dev, "syscon regmap lookup failed.\n");
			return PTR_ERR(info->pmu_map);
		}
	}

	irq = platform_get_irq(pdev, 0);//获取第一号中断
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return irq;
	}
	info->irq = irq;

	irq = platform_get_irq(pdev, 1);//获取第二号中断
	if (irq == -EPROBE_DEFER)
		return irq;

	info->tsirq = irq;

	info->dev = &pdev->dev;

	/*
	completion是一种轻量级的机制,它允许一个线程告诉另一个线程工作已经完成。
	可以利用下面的宏静态创建completion:
    DECLARE_COMPLETION(my_completion); 
       
  如果运行时创建completion,则必须采用以下方法动态创建和初始化:
     struct compltion my_completion;
     init_completion(&;my_completion);
  completion的相关定义包含在kernel/include/linux/completion.h中:
  struct completion {
     unsigned int done;
     wait_queue_head_t wait;
  };
	*/
	init_completion(&info->completion);

	info->clk = devm_clk_get(&pdev->dev, "adc");
	if (IS_ERR(info->clk)) {
		dev_err(&pdev->dev, "failed getting clock, err = %ld\n",
							PTR_ERR(info->clk));
		return PTR_ERR(info->clk);
	}

	if (info->data->needs_sclk) {
		info->sclk = devm_clk_get(&pdev->dev, "sclk");
		if (IS_ERR(info->sclk)) {
			dev_err(&pdev->dev,
				"failed getting sclk clock, err = %ld\n",
				PTR_ERR(info->sclk));
			return PTR_ERR(info->sclk);
		}
	}

	info->vdd = devm_regulator_get(&pdev->dev, "vdd");
	if (IS_ERR(info->vdd)) {
		dev_err(&pdev->dev, "failed getting regulator, err = %ld\n",
							PTR_ERR(info->vdd));
		return PTR_ERR(info->vdd);
	}

	ret = regulator_enable(info->vdd);
	if (ret)
		return ret;

	/*
	值得一提的是，名称中含有prepare、unprepare字符串的API是内核后来才加入的，
	过去只有clk_enable和clk_disable。只有clk_enable和clk_disable带来的问题是，
	有时候，某些硬件的enable/disable clk可能引起睡眠使得enable/disable不能在原子上下文进行。
	加上prepare后，把过去的clk_enable分解成不可在原子上下文调用的clk_prepare（该函数可能睡眠）
	和可以在原子上下文调用的clk_enable。而clk_prepare_enable则同时完成prepare和enable的工作，
	当然也只能在可能睡眠的上下文调用该API。
	*/
	ret = exynos_adc_prepare_clk(info);
	if (ret)
		goto err_disable_reg;

	ret = exynos_adc_enable_clk(info);
	if (ret)
		goto err_unprepare_clk;

	/* 设置私有数据 */
	platform_set_drvdata(pdev, indio_dev);

	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->info = &exynos_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = exynos_adc_iio_channels;/* 通道数据 */
	indio_dev->num_channels = info->data->num_channels;

	ret = request_irq(info->irq, exynos_adc_isr,
					0, dev_name(&pdev->dev), info);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed requesting irq, irq = %d\n",
							info->irq);
		goto err_disable_clk;
	}

	/* 注册iio设备 */
	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_irq;

	if (info->data->init_hw)
		info->data->init_hw(info);

	/* leave out any TS related code if unreachable */
	if (IS_REACHABLE(CONFIG_INPUT)) {
		has_ts = of_property_read_bool(pdev->dev.of_node,//判断是否有touchscreen
					       "has-touchscreen") || pdata;
	}

	if (pdata)
		info->delay = pdata->delay;
	else
		info->delay = 10000;

	if (has_ts)
		//初始化touchscreen
		ret = exynos_adc_ts_init(info);
	if (ret)
		goto err_iio;

	/*
	对于of_platform_populate如何选择性的加载device node为platform device在系统启动阶段，
	可以只关注该device node所属的parent node的compatible属于match id。
	一旦自身包含compatible，则会自动调用of_platform_device_create_pdata生成一个platform device。
	*/
	ret = of_platform_populate(np, exynos_adc_match, NULL, &indio_dev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed adding child nodes\n");
		goto err_of_populate;
	}

	return 0;

err_of_populate:
	device_for_each_child(&indio_dev->dev, NULL,
				exynos_adc_remove_devices);
	if (has_ts) {
		input_unregister_device(info->input);
		free_irq(info->tsirq, info);
	}
err_iio:
	iio_device_unregister(indio_dev);
err_irq:
	free_irq(info->irq, info);
err_disable_clk:
	if (info->data->exit_hw)
		info->data->exit_hw(info);
	exynos_adc_disable_clk(info);
err_unprepare_clk:
	exynos_adc_unprepare_clk(info);
err_disable_reg:
	regulator_disable(info->vdd);
	return ret;
}
/*
 在比较新的linux内核中，设备树dts已经取代了传统的machine board device description，
 dts在内核中以各种device node的形式存在，而这些device node对于大部分的内核驱动模块
 platform_driver来说，最终需要有对应的platform device来与他匹配才可以完成一次device和
 driver的probe过程。

所有有必要将dts中需要加载为device的device node转为platform device，而这个过程是交给
of_platform_populate来完成的（dts相关的device node tree是在start_kernel的
setup_arch->unflatten_device_tree来加载dtb并解析）。

 of_platform_populate的入口一般是处于init_machine中，对于arm架构而言位于board.c中的
 DT_MACHINE_START()的init_machine中定义，而init_machine的调用是以一个module的形式而存在的，
 该module被加载后的目的，就是做device的create，在以前旧的board中，会将board.c中定义的相关
 device info转换为对应的device，以便后面的driver加载时可以match到相应的device。在基于dts的
 设备管理中，这个功能由of_platform_populate来实现
*/
int of_platform_populate(struct device_node *root,
			const struct of_device_id *matches,
			const struct of_dev_auxdata *lookup,
			struct device *parent)
{
	struct device_node *child;
	int rc = 0;

	root = root ? of_node_get(root) : of_find_node_by_path("/");
	if (!root)
		return -EINVAL;

	pr_debug("%s()\n", __func__);
	pr_debug(" starting at: %pOF\n", root);

	for_each_child_of_node(root, child) {
		rc = of_platform_bus_create(child, matches, lookup, parent, true);
		if (rc) {
			of_node_put(child);
			break;
		}
	}
	of_node_set_flag(root, OF_POPULATED_BUS);

	of_node_put(root);
	return rc;
}
/*
该该函数的功能主要可以总结如下：
a.根据所选择的device node根节点，来递归式的遍历从root node开始以下的所有device node

b.将device node转变为一个platform_device并将其作为device 通过device_add到内核

c.可以判断哪些device node是需要转为device到内核的。

d. 如果传入的root=NULL，则表明从dts的\节点开始逐一的递归处理，否则根据所选择的device node
作为root，做递归处理。

e. struct of_device_id *matches，该match table重点是后续节点递归处理时，需要和该table mach后
才可以继续递归处理。*/

//设备树里的设备节点:
adc: adc@126C0000 {
                compatible = "samsung,exynos-adc-v1";
                reg = <0x126C0000 0x100>;
                interrupt-parent = <&combiner>;
                interrupts = <10 3>;
                clocks = <&clock CLK_TSADC>;
                clock-names = "adc";
                #io-channel-cells = <1>;
                io-channel-ranges;
                samsung,syscon-phandle = <&pmu_system_controller>;
                status = "disabled";
};
&adc {
	/*vdd-supply = <&ldo3_reg>;*/
	status = "okay";
};
//重启系统后，可以查看到:
~ # ls /sys/bus/iio/devices/                                                    
iio:device0
~ # cd /sys/bus/iio/devices/iio\:device0/                                       
/sys/devices/platform/126c0000.adc/iio:device0 # ls
dev              in_voltage3_raw  in_voltage7_raw  subsystem
in_voltage0_raw  in_voltage4_raw  name             uevent
in_voltage1_raw  in_voltage5_raw  of_node
in_voltage2_raw  in_voltage6_raw  power
/sys/devices/platform/126c0000.adc/iio:device0 # cat in_voltage0_raw            
2465
/sys/devices/platform/126c0000.adc/iio:device0 # cat in_voltage1_raw            
2567
/sys/devices/platform/126c0000.adc/iio:device0 # cat in_voltage2_raw            
2286
/sys/devices/platform/126c0000.adc/iio:device0 # cat in_voltage3_raw            
3039




























