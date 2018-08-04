/*
I2C�豸����ͨ��ֻ����Ҫ������I2C���ߣ���������I2C��ϵͳ����I2C��ϵͳ�����豸������˵
ֻ��һ�����塢��ʯ������豸����Ҫ�����ǽ�����������ϵͳ�ϣ������������������ᴫ����
��������ͨ����Ҫ����������INPUT��ϵͳ�У������ģ�顢FMģ�顢GPSģ������Ҫ������V4L2��
ϵͳ����Ҳ��ͨ��I2C�������֤����I2C�Ĳ�������Ϊ�˽�ʡ��Χ��·���Ӷȣ���CPUʹ�����޵�
IO�ڹ��ظ�����ⲿģ�顣����CPU����չIO���㹻�࣬����I2CҲûʲô��Ҫ�����ˣ��Ͼ�ֱ�Ӳ�
��IO�������豸��I2C���ĸ��򵥡�

I2C adapter
��CPU���ɻ���ӵ�I2C���������������Ƹ���I2C���豸����������Ҫ��ɶ�����������������������
Ҫ�Ĺ�������Ҫ���i2c_algorithm�ṹ�塣

I2C driver
�����I2C�豸����

I2C client
��I2C�豸��I2C�豸��ע��һ���ڰ弶������


*/
//��Ҫ���i2c��ƽ̨�豸��Ϣ
static void __init smdk4x12_machine_init(void)
{
	//...
	//��ʼ�����ƽ̨��Ϣ

	s3c_i2c0_set_platdata(&tiny4412_i2c0_data);
	/*
		i2c_register_board_info()����������__i2c_board_list�����������һ��i2c�豸
		��Ϣ����i2c adapterע���ʱ�򣬻�ɨ��__i2c_board_list����Ȼ�����i2c_new_device()
		������ע��i2c�豸,��ʵ��I2C�豸�����������ɹ�ע���ű����ɵ�.ע�⣬Ҫ����i2c adapter
		ע��֮ǰ����Ӻ�i2c�豸��Ϣ���������ֵ�����i2c_register_board_info()���������豸����
		ע��������
	*/
	i2c_register_board_info(0, smdk4x12_i2c_devs0,
			ARRAY_SIZE(smdk4x12_i2c_devs0));

	//ע��i2c��ƽ̨�豸��Ϣ

	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
	//...
}


static struct s3c2410_platform_i2c tiny4412_i2c0_data __initdata = {
	.flags			= 0,
	.bus_num		= 0,
	.slave_addr		= 0x10,
	.frequency		= 200*1000,
	.sda_delay		= 100,
};

static struct i2c_board_info smdk4x12_i2c_devs0[] __initdata = {
#ifdef CONFIG_SND_SOC_WM8960_TINY4412
	{
		I2C_BOARD_INFO("wm8960", 0x1a),
		.platform_data = &wm8960_pdata,
	},
#endif

void __init s3c_i2c0_set_platdata(struct s3c2410_platform_i2c *pd)
{
	struct s3c2410_platform_i2c *npd;

	if (!pd) {
		pd = &default_i2c_data;
		pd->bus_num = 0;
	}

	npd = s3c_set_platdata(pd, sizeof(struct s3c2410_platform_i2c),
			       &s3c_device_i2c0);

	if (!npd->cfg_gpio)
		npd->cfg_gpio = s3c_i2c0_cfg_gpio;
}

/* I2C */

static struct resource s3c_i2c0_resource[] = {
	[0] = DEFINE_RES_MEM(S3C_PA_IIC, SZ_4K),
	[1] = DEFINE_RES_IRQ(IRQ_IIC),
};

struct platform_device s3c_device_i2c0 = {
	.name		= "s3c2410-i2c",
#ifdef CONFIG_S3C_DEV_I2C1
	.id		= 0,
#else
	.id		= -1,
#endif
	.num_resources	= ARRAY_SIZE(s3c_i2c0_resource),
	.resource	= s3c_i2c0_resource,
};

/*-----------------------------------------------------------------------------------
samsung i2c-s3c2410.c����
-----------------------------------------------------------------------------------*/
static int s3c24xx_i2c_probe(struct platform_device *pdev)
{
	struct s3c24xx_i2c *i2c;
	struct s3c2410_platform_i2c *pdata = NULL;
	struct resource *res;
	int ret;

	if (!pdev->dev.of_node) {
		pdata = pdev->dev.platform_data;//��ȡplatform_data ��һ����ƽ̨��ص���������
		if (!pdata) {
			dev_err(&pdev->dev, "no platform data\n");
			return -EINVAL;
		}
	}

	//Ϊ�ṹ��i2c�����ڴ�ռ䲢���㡣
	i2c = devm_kzalloc(&pdev->dev, sizeof(struct s3c24xx_i2c), GFP_KERNEL);
	if (!i2c) {
		dev_err(&pdev->dev, "no memory for state\n");
		return -ENOMEM;
	}

	i2c->pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!i2c->pdata) {
		dev_err(&pdev->dev, "no memory for platform data\n");
		return -ENOMEM;
	}

	i2c->quirks = s3c24xx_get_device_quirks(pdev);
	if (pdata)
		memcpy(i2c->pdata, pdata, sizeof(*pdata));
	else
		s3c24xx_i2c_parse_dt(pdev->dev.of_node, i2c);

	//����������������������ں˼�һ��������
	strlcpy(i2c->adap.name, "s3c2410-i2c", sizeof(i2c->adap.name));
	i2c->adap.owner   = THIS_MODULE;
	i2c->adap.algo    = &s3c24xx_i2c_algorithm;//��������������
	i2c->adap.retries = 2;//���ݴ����쳣��Ϻ��ظ����ԵĴ���
	i2c->adap.class   = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	i2c->tx_setup     = 50;//����д��IICDS����ʱʱ��

	init_waitqueue_head(&i2c->wait);

	/* find the clock and enable it */

	i2c->dev = &pdev->dev;
	i2c->clk = clk_get(&pdev->dev, "i2c");//��ȡƽ̨ʱ��
	if (IS_ERR(i2c->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		ret = -ENOENT;
		goto err_noclk;
	}

	dev_dbg(&pdev->dev, "clock source %p\n", i2c->clk);

	clk_enable(i2c->clk);//ʹ��ʱ��

	/* map the registers */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);//��ȡI/O��Դ
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot find IO resource\n");
		ret = -ENOENT;
		goto err_clk;
	}

	i2c->ioarea = request_mem_region(res->start, resource_size(res),
					 pdev->name);//����IO�ڴ�

	if (i2c->ioarea == NULL) {
		dev_err(&pdev->dev, "cannot request IO\n");
		ret = -ENXIO;
		goto err_clk;
	}

	i2c->regs = ioremap(res->start, resource_size(res));//IOӳ��

	if (i2c->regs == NULL) {
		dev_err(&pdev->dev, "cannot map IO\n");
		ret = -ENXIO;
		goto err_ioarea;
	}

	dev_dbg(&pdev->dev, "registers %p (%p, %p)\n",
		i2c->regs, i2c->ioarea, res);

	/* setup info block for the i2c core */
//��i2c�ṹ����Ϊadap��˽�����ݳ�Ա
	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &pdev->dev;

	/* inititalise the i2c gpio lines */

	if (i2c->pdata->cfg_gpio) {
		i2c->pdata->cfg_gpio(to_platform_device(i2c->dev));
	} else if (s3c24xx_i2c_parse_dt_gpio(i2c)) {
		ret = -EINVAL;
		goto err_iomap;
	}

	/* initialise the i2c controller */

	ret = s3c24xx_i2c_init(i2c);
	if (ret != 0) {
		dev_err(&pdev->dev, "I2C controller init failed\n");
		goto err_iomap;
	}

	/* find the IRQ for this unit (note, this relies on the init call to
	 * ensure no current IRQs pending
	 */

	i2c->irq = ret = platform_get_irq(pdev, 0);
	if (ret <= 0) {
		dev_err(&pdev->dev, "cannot find IRQ\n");
		goto err_iomap;
	}

	//����һ���жϲ��������Ĵ�������IRQF_DISABLED���жϲ�����������Ϊ��
	ret = request_irq(i2c->irq, s3c24xx_i2c_irq, 0,
			  dev_name(&pdev->dev), i2c);

	if (ret != 0) {
		dev_err(&pdev->dev, "cannot claim IRQ %d\n", i2c->irq);
		goto err_iomap;
	}
	
	//��֪ͨ�ڵ�ע�ᵽ֪ͨ����
	ret = s3c24xx_i2c_register_cpufreq(i2c);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register cpufreq notifier\n");
		goto err_irq;
	}

	/* Note, previous versions of the driver used i2c_add_adapter()
	 * to add the bus at any number. We now pass the bus number via
	 * the platform data, so if unset it will now default to always
	 * being bus 0.
	 */

	i2c->adap.nr = i2c->pdata->bus_num;//�������ߺ�
	i2c->adap.dev.of_node = pdev->dev.of_node;

	//�ر���Ҫ��һ�����������һ���������豸
	/*
	i2c_add_numbered_adapter()->i2c_register_adapter()
	��i2c_register_adapter()�����е�����i2c_scan_static_board_info()����
	i2c_scan_static_board_info()������ɨ��__i2c_board_list��Ȼ�����i2c_new_device()
	������ע��i2c�豸������Ǿ�̬ע��i2c�豸�ķ�����һ���ڰ弶�ĳ�ʼ�������е���
	i2c_register_board_info()���������i2c�豸��Ϣ����ӡ�
	i2c_new_device()������ϸ����
	*/
	
	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add bus to i2c core\n");
		goto err_cpufreq;
	}

	of_i2c_register_devices(&i2c->adap);
	platform_set_drvdata(pdev, i2c);//��i2c��Ϊpdev��drvdata��

	pm_runtime_enable(&pdev->dev);
	pm_runtime_enable(&i2c->adap.dev);

	dev_info(&pdev->dev, "%s: S3C I2C adapter\n", dev_name(&i2c->adap.dev));
	clk_disable(i2c->clk);
	return 0;

 err_cpufreq:
	s3c24xx_i2c_deregister_cpufreq(i2c);

 err_irq:
	free_irq(i2c->irq, i2c);

 err_iomap:
	if (i2c->gpios[0])
		s3c24xx_i2c_dt_gpio_free(i2c);

	iounmap(i2c->regs);

 err_ioarea:
	release_resource(i2c->ioarea);
	kfree(i2c->ioarea);

 err_clk:
	clk_disable(i2c->clk);
	clk_put(i2c->clk);

 err_noclk:
	return ret;
}

/*
i2d_new_deviceûʲô�ö�˵�ģ�������i2c_register_board_info���̵棬����Ҳ�ܺ�����ˡ�
��i2c_new_device����ӡ֤��i2c_client��i2c_board_info�Ķ�Ӧ��ϵ����˳��������10bit��ַ
�豸��7bit��ַ�豸����΢��ͬ��ͨ�����ߵĶԱȣ��������ܽ���������𣬴Ӷ����õ����I2C
�豸��ע�᷽����

i2c_register_board_info���β���Ҫ�������ߺ�
i2c_new_device���β���Ҫ��ֱ������������ָ��
������Ҳ������������˵�����ߵĸ������𣬶��ڰ弶�豸���ں������������ߺţ���Ϊ���ǹ̶��ģ�
û������ġ������ڿɲ���豸����Ϊ�����������ܷǰ弶���ɵģ����Բ����ں������ߺţ�����ֻҪ
Ѱ����������ָ����а󶨼��ɡ����adapterע��ķ����ܸ��õ�֤����һ�㡣

i2c_register_board_info����ͬʱע����I2C�豸
i2c_new_deviceֻ��һ��ע��һ��I2C�豸
��Ҳ���������������ģ��弶�����г����������I2C�豸������i2c_register_board_info��Ҫ��ͬʱ
ע����I2C�豸������Ҳ����˵�Ǹ��衣��i2c_new_device��Ȼ���������ɲ���豸�õģ�����豸����
�����࣬��������ֻ��һ���������ѣ�����һ��һ���͹��ˣ�������������
*/
struct i2c_client *i2c_new_device(struct i2c_adapter *adap, struct i2c_board_info const *info)
{
    struct i2c_client    *client;
    int            status;

    client = kzalloc(sizeof *client, GFP_KERNEL);  //Ϊ����ע���client�����ڴ�
    if (!client)
        return NULL;

    client->adapter = adap;  //��ָ����adapter������

    client->dev.platform_data = info->platform_data;  //�����豸����

    if (info->archdata)  //�����Ͽ���DMA��ز�������
        client->dev.archdata = *info->archdata;

    client->flags = info->flags;  //���ͣ���һ����˵��������10λ��ַ������ʹ��SMBus���
    client->addr = info->addr;  //�豸�ӵ�ַ
    client->irq = info->irq;  //�豸�ն�

    strlcpy(client->name, info->type, sizeof(client->name));  //���豸��
            //�ƣ���һ����˵��i2c_board_info�е���Ϣ����i2c_client�ж�Ӧ��ϵ�ģ������˰ɣ�

    /* Check for address validity */
    status = i2c_check_client_addr_validity(client);  //����ַ�Ƿ���Ч��10λ��ַ�Ƿ����0x3ff��7λ��ַ�Ƿ����0x7f��Ϊ0
    if (status) {  //���㣨ʵ����Ϊ-22����Ч����Invalid argument
        dev_err(&adap->dev, "Invalid %d-bit I2C address 0x%02hx\n",
            client->flags & I2C_CLIENT_TEN ? 10 : 7, client->addr);
        goto out_err_silent;
    }

    /* Check for address business */
    status = i2c_check_addr_busy(adap, client->addr);  //���ָ���������ϸõ�ַ״̬
    if (status)
        goto out_err;

    client->dev.parent = &client->adapter->dev;  //�������豸���������ĸ��ӹ�ϵ
    client->dev.bus = &i2c_bus_type;
    client->dev.type = &i2c_client_type;
    client->dev.of_node = info->of_node;
    ACPI_HANDLE_SET(&client->dev, info->acpi_node.handle);

    /* For 10-bit clients, add an arbitrary offset to avoid collisions */
    dev_set_name(&client->dev, "%d-%04x", i2c_adapter_id(adap),
             client->addr | ((client->flags & I2C_CLIENT_TEN)
                     ? 0xa000 : 0));  //�����10λ��ַ�豸�������ָ�ʽ��7bit�Ļ��в�ͬ
    status = device_register(&client->dev);  //ע���ˣ�ע���ˣ�����
    if (status)
        goto out_err;

    dev_dbg(&adap->dev, "client [%s] registered with bus id %s\n",
        client->name, dev_name(&client->dev));

    return client;

out_err:
    dev_err(&adap->dev, "Failed to register i2c client %s at 0x%02x "
        "(%d)\n", client->name, client->addr, status);
out_err_silent:
    kfree(client);
    return NULL;
}





















