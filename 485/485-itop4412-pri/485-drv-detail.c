/*
485�豸���������ַ��豸������
�����ļ��ڣ� ��kernel/drivers/char/Ŀ¼�¡�
max485_crtl.c ����-�������� 
*/
////��Ҫ���485��ƽ̨�豸��Ϣ
static void __init smdk4x12_machine_init(void)
{
	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));//ע��ƽ̨�豸
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

//�ں�����
Device Drivers --->
	Character devices --->
		Enable MAX485 pin config

/*-----------------------------------------------------------------------------------
samsung max485_crtl.c ����
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
	һ��gpio_request��װ��mem_request(),�𱣻����ã����Ҫ����mem_free֮��ġ���Ҫ�Ǹ����ں����ַ��ռ���ˡ�
	�������ط�����ͬһ��ַ��gpio_request�ͻᱨ����󣬸õ�ַ�ѱ����롣��/proc/memӦ�û��е�ַռ�ñ�������
	�����÷��ı�������ǰ���Ǵ�Ҷ������������ٷ��ʣ���һ���ط�û������������⹦�ܾ�ʧЧ�ˡ��ñȽ��̻��⣬
	�������ڷ����ٽ���Դ��ʱ�򶼵��Ȼ�ȡ��һ��������һ��û����Լ��������ͷ��ˡ�
	gpio��Ϊ��Ҫ�������һ���ܽţ�label����Ϊ��ȡһ�����֡�
	*/
	err = gpio_request(EXYNOS4_GPA0(7), "GPA0_7");
	if (err) {
		printk(KERN_ERR "failed to request GPA0_7 for "
			"max485_ctl control\n");
		return err;
	}
	/*��ĳ��GPIO��д��ĳ��ֵ֮�󣬻��������˿�����Ϊ���ģʽ*/
	gpio_direction_output(EXYNOS4_GPA0(7), 1);

	s3c_gpio_cfgpin(EXYNOS4_GPA0(7), S3C_GPIO_OUTPUT);//����Ϊ���ģʽ
	gpio_free(EXYNOS4_GPA0(7));//�ͷ�GPIO port ��ʹ��Ȩ,��gpio ָ������ port

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

/*���ǿ��Կ���GPIO֮ǰ��probe�������ͷţ�ioctl������������ע������*/
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







