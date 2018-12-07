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

//内核配置
Device Drivers --->
	Character devices --->
		Enable MAX485 pin config

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

	/*
	一般gpio_request封装了mem_request(),起保护作用，最后要调用mem_free之类的。主要是告诉内核这地址被占用了。
	当其它地方调用同一地址的gpio_request就会报告错误，该地址已被申请。在/proc/mem应该会有地址占用表描述。
	这种用法的保护作用前提是大家都遵守先申请再访问，有一个地方没遵守这个规则，这功能就失效了。好比进程互斥，
	必需大家在访问临界资源的时候都得先获取锁一样，其中一个没遵守约定，代码就废了。
	gpio则为你要申请的哪一个管脚，label则是为其取一个名字。
	*/
	err = gpio_request(EXYNOS4_GPA0(7), "GPA0_7");
	if (err) {
		printk(KERN_ERR "failed to request GPA0_7 for "
			"max485_ctl control\n");
		return err;
	}
	/*在某个GPIO口写上某个值之后，还会把这个端口设置为输出模式*/
	gpio_direction_output(EXYNOS4_GPA0(7), 1);

	s3c_gpio_cfgpin(EXYNOS4_GPA0(7), S3C_GPIO_OUTPUT);//配置为输出模式
	gpio_free(EXYNOS4_GPA0(7));//释放GPIO port 的使用权,由gpio 指定具体 port

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


static struct file_operations max485_ctl_ops = {
	.owner 	= THIS_MODULE,
	.open 	= max485_ctl_open,
	.release= max485_ctl_release,
	.unlocked_ioctl 	= max485_ctl_ioctl,
};

static struct miscdevice max485_ctl_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.fops	= &max485_ctl_ops,
	.name	= "max485_ctl_pin",
};

/*我们可以看到GPIO之前在probe函数被释放，ioctl函数中又重新注册申请*/
long max485_ctl_ioctl(struct file *filp,unsigned int cmd,unsigned long arg)
{
	printk("firecxx debug: max485_ctl_ioctl cmd is %d\n" , cmd);

	switch(cmd)
	{		
		case 1:
			if(gpio_request(EXYNOS4_GPA0(7) ,"GPA0_7"))
			{
				DPRINTK("max485_ctl GPIO err!\r\n");
			}
			else
			{
				gpio_direction_output(EXYNOS4_GPA0(7), 1);
				DPRINTK("max485_ctl Set High!\n");
				gpio_free(EXYNOS4_GPA0(7));

				mdelay(100);
			}
				
			break;
		case 0:
			if(gpio_request(EXYNOS4_GPA0(7) ,"GPA0_7"))
			{
				DPRINTK("max485_ctl GPIO err!\r\n");
			}
			else
			{			
				gpio_direction_output(EXYNOS4_GPA0(7),0);
				DPRINTK("max485_ctl Set Low!\n");
				gpio_free(EXYNOS4_GPA0(7));

				mdelay(100); 
			}
			
			break;
			
		default:
			DPRINTK("max485_ctl COMMAND ERROR!\n");
			return -ENOTTY;
	}
	return 0;
}







