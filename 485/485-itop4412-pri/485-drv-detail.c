/*
485设备驱动属于字符设备驱动。
驱动文件在： 在kernel/drivers/char/目录下。
max485_crtl.c ——-总线驱动 
*/
////需要添加485的平台设备信息
static void __init smdk4x12_machine_init(void)
{
	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));//注册平台设备
}

static struct platform_device *smdk4x12_devices[] __initdata = {

	#ifdef CONFIG_MAX485_CTL
	&s3c_device_max485_ctl ,
	#endif
}

#ifdef CONFIG_MAX485_CTL
struct platform_device s3c_device_max485_ctl = {
        .name   = "max485_ctl",
        .id             = -1,
};
#endif

/*-----------------------------------------------------------------------------------
samsung max485_crtl.c 分析
-----------------------------------------------------------------------------------*/
static struct platform_driver max485_ctl_driver = {
	.probe = max485_ctl_probe,
	.remove = max485_ctl_remove,
	.suspend = max485_ctl_suspend,
	.resume = max485_ctl_resume,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init max485_ctl_init(void)
{
	return platform_driver_register(&max485_ctl_driver);
}


static int max485_ctl_probe(struct platform_device *pdev)
{
	int err = 0;
	
	int ret;
	char *banner = "max485_ctl Initialize\n";

	printk(banner);

	err = gpio_request(EXYNOS4_GPA0(7), "GPA0_7");
	if (err) {
		printk(KERN_ERR "failed to request GPA0_7 for "
			"max485_ctl control\n");
		return err;
	}
	gpio_direction_output(EXYNOS4_GPA0(7), 1);

	s3c_gpio_cfgpin(EXYNOS4_GPA0(7), S3C_GPIO_OUTPUT);
	gpio_free(EXYNOS4_GPA0(7));

	ret = misc_register(&max485_ctl_dev);
	if(ret<0)
	{
		printk("max485_ctl:register device failed!\n");
		goto exit;
	}

	return 0;

exit:
	misc_deregister(&max485_ctl_dev);
	return ret;
}






