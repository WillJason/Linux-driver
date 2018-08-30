/*
Ӳ��ƽ̨��itop4412
ϵͳ��linux-4.14.2
���豸����ֱ��ʹ���ж���Դ��ʵ��ADC�ɼ��װ��ϻ����������ĵ�ѹ��

��Linux kernel 3.2�Ѿ�������IIO���ļ�ϵͳ�������������࣬�Ͼ��տ�ʼ��һ���µ����ļ�ϵͳ ��
��kernel 3.x�汾��Ŀ¼��drivers/staging/iioĿ¼�£�����kernel 4.x�汾�±��Ƶ���drivers/iio��
��Ȼ����Ͽ���������ļ�ϵͳ�Ŀ�ܡ�

IIO��ϵͳȫ���� Industrial I/O subsystem����ҵ I/O ��ϵͳ���������ں�˵���ĵ���������
��ҵI/O��ϵͳּ���ṩ��Щ��ĳ����������Ϊģ��ת������ADC�����豸֧�֡�����ϵͳ��Ŀ���������Щ����ʱ��
��hwmon��Ӳ������������������ϵͳ֮����豸���͡���ĳЩ����£�IIO��hwmon��Input֮����൱����ص���

����IIO���豸���£��ο�ADI��WiKi����
ADCs(ģ��ת����)
���ٶȴ�����
������
IMUs(���Բ�����λ)
����-����ת����(CDCs)
ѹ�����¶Ⱥ͹��ߴ�����
��Դ���Ŀ¼�п��Կ��������У������ƴ����������ܹ��ʼơ���������ת����
���Կ�����Щ�����ȷ���ǹ�ҵ����ʹ�õģ���Ȼ���еĲ�������������Ҳ�õ��ˣ�����
���ٶȴ������������ǡ��¶Ⱥ͹��ߴ��������ƶ��豸���ֻ���ƽ�壩��Ҳ�й㷺Ӧ�á�
 
iio��ϵͳ���û��λ����������λ�����������һ�����ݽṹ����һ���̶���С���ɵ�����β��������
���ֽṹ�ǳ��ʺϻ�������������Щ������ͨ������������������������⣬��һЩӦ���У�
������Ƴ������߻�(����һ��ADC)����������(����һ���û��ռ�Ӧ�ó���)�޷���ʱ����Ĺ������ݡ�
����ͨ�����ֻ���ᱻ����Ϊ�ʵ��Ĵ�С����ʹ����������ᷢ����


*/
//��Ӧ���豸����Դ�ļ�: drivers/iio/adc/exynos_adc.c
//exynos_adc.cԴ�ļ����豸����ص���Ҫ����:
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
//ƥ���Ϻ� �豸������exynos_adc_probe�����ͻᱻ��������. ��probe������ͻ�ȡ���豸�����ṩ��Ӳ����Դ.
static int exynos_adc_probe(struct platform_device *pdev)
{
	struct exynos_adc *info = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct s3c2410_ts_mach_info *pdata = dev_get_platdata(&pdev->dev);//ʹ���豸���ķ�ʽ, pdataӦΪNULL;
	struct iio_dev *indio_dev = NULL;
	struct resource	*mem;
	bool has_ts = false;
	int ret = -ENODEV;
	int irq;

	 /* ��̬����iio�豸 */
	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(struct exynos_adc));
	if (!indio_dev) {
		dev_err(&pdev->dev, "failed allocating iio device\n");
		return -ENOMEM;
	}

	/*
	return (char *)indio_dev + ALIGN(sizeof(struct iio_dev), IIO_ALIGN);
	��linux�ں���,�����ῴ������ALIGN�ĵ���,���������ڴ������page���룬net_device��˽�����ݵĻ�ȡ��
	��˵ALIGN���÷���ALIGN(x,a) ��Ϊ��ʹx��aΪ�߽���룬ʵ��ԭ���Ǹ�x����һ����С������
	ʹx��aΪ�߽���롣�ٸ����ӣ�a = 8, x=0, ALIGN(x,a) ������Ϊ0�� a = 8, x = 3�� ������Ϊ8��
	a = 8, x = 11, ������Ϊ16�� 
	*/
	info = iio_priv(indio_dev);

	info->data = exynos_adc_get_data(pdev);//��ȡ�豸ID��ṹ�Լ�ƽ̨�豸��Դ
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

	irq = platform_get_irq(pdev, 0);//��ȡ��һ���ж�
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return irq;
	}
	info->irq = irq;

	irq = platform_get_irq(pdev, 1);//��ȡ�ڶ����ж�
	if (irq == -EPROBE_DEFER)
		return irq;

	info->tsirq = irq;

	info->dev = &pdev->dev;

	/*
	completion��һ���������Ļ���,������һ���̸߳�����һ���̹߳����Ѿ���ɡ�
	������������ĺ꾲̬����completion:
    DECLARE_COMPLETION(my_completion); 
       
  �������ʱ����completion,�����������·�����̬�����ͳ�ʼ��:
     struct compltion my_completion;
     init_completion(&;my_completion);
  completion����ض��������kernel/include/linux/completion.h��:
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
	ֵ��һ����ǣ������к���prepare��unprepare�ַ�����API���ں˺����ż���ģ�
	��ȥֻ��clk_enable��clk_disable��ֻ��clk_enable��clk_disable�����������ǣ�
	��ʱ��ĳЩӲ����enable/disable clk��������˯��ʹ��enable/disable������ԭ�������Ľ��С�
	����prepare�󣬰ѹ�ȥ��clk_enable�ֽ�ɲ�����ԭ�������ĵ��õ�clk_prepare���ú�������˯�ߣ�
	�Ϳ�����ԭ�������ĵ��õ�clk_enable����clk_prepare_enable��ͬʱ���prepare��enable�Ĺ�����
	��ȻҲֻ���ڿ���˯�ߵ������ĵ��ø�API��
	*/
	ret = exynos_adc_prepare_clk(info);
	if (ret)
		goto err_disable_reg;

	ret = exynos_adc_enable_clk(info);
	if (ret)
		goto err_unprepare_clk;

	/* ����˽������ */
	platform_set_drvdata(pdev, indio_dev);

	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->info = &exynos_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = exynos_adc_iio_channels;/* ͨ������ */
	indio_dev->num_channels = info->data->num_channels;

	ret = request_irq(info->irq, exynos_adc_isr,
					0, dev_name(&pdev->dev), info);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed requesting irq, irq = %d\n",
							info->irq);
		goto err_disable_clk;
	}

	/* ע��iio�豸 */
	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_irq;

	if (info->data->init_hw)
		info->data->init_hw(info);

	/* leave out any TS related code if unreachable */
	if (IS_REACHABLE(CONFIG_INPUT)) {
		has_ts = of_property_read_bool(pdev->dev.of_node,//�ж��Ƿ���touchscreen
					       "has-touchscreen") || pdata;
	}

	if (pdata)
		info->delay = pdata->delay;
	else
		info->delay = 10000;

	if (has_ts)
		//��ʼ��touchscreen
		ret = exynos_adc_ts_init(info);
	if (ret)
		goto err_iio;

	/*
	����of_platform_populate���ѡ���Եļ���device nodeΪplatform device��ϵͳ�����׶Σ�
	����ֻ��ע��device node������parent node��compatible����match id��
	һ���������compatible������Զ�����of_platform_device_create_pdata����һ��platform device��
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
 �ڱȽ��µ�linux�ں��У��豸��dts�Ѿ�ȡ���˴�ͳ��machine board device description��
 dts���ں����Ը���device node����ʽ���ڣ�����Щdevice node���ڴ󲿷ֵ��ں�����ģ��
 platform_driver��˵��������Ҫ�ж�Ӧ��platform device������ƥ��ſ������һ��device��
 driver��probe���̡�

�����б�Ҫ��dts����Ҫ����Ϊdevice��device nodeתΪplatform device������������ǽ���
of_platform_populate����ɵģ�dts��ص�device node tree����start_kernel��
setup_arch->unflatten_device_tree������dtb����������

 of_platform_populate�����һ���Ǵ���init_machine�У�����arm�ܹ�����λ��board.c�е�
 DT_MACHINE_START()��init_machine�ж��壬��init_machine�ĵ�������һ��module����ʽ�����ڵģ�
 ��module�����غ��Ŀ�ģ�������device��create������ǰ�ɵ�board�У��Ὣboard.c�ж�������
 device infoת��Ϊ��Ӧ��device���Ա�����driver����ʱ����match����Ӧ��device���ڻ���dts��
 �豸�����У����������of_platform_populate��ʵ��
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
�øú����Ĺ�����Ҫ�����ܽ����£�
a.������ѡ���device node���ڵ㣬���ݹ�ʽ�ı�����root node��ʼ���µ�����device node

b.��device nodeת��Ϊһ��platform_device��������Ϊdevice ͨ��device_add���ں�

c.�����ж���Щdevice node����ҪתΪdevice���ں˵ġ�

d. ��������root=NULL���������dts��\�ڵ㿪ʼ��һ�ĵݹ鴦�����������ѡ���device node
��Ϊroot�����ݹ鴦��

e. struct of_device_id *matches����match table�ص��Ǻ����ڵ�ݹ鴦��ʱ����Ҫ�͸�table mach��
�ſ��Լ����ݹ鴦��*/

//�豸������豸�ڵ�:
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
//����ϵͳ�󣬿��Բ鿴��:
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




























