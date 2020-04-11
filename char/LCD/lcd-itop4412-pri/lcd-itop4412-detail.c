/*
fbmem.c�ļ��ṩ��framebuffer���������ͨ���ļ������ӿڣ��Զ����framebuffer�����������ʹ��fbmem.c���ṩĬ�ϵĽӿڡ���EXPORT_SYMBOL�����������ļ���Ӧ��
lcd��Ӧ�ò� ͨ�� �ں˵�fbmem�ӿ� �ٵ�������xxxfb.c������ 
��fbmem�ӿ����ں��ṩ�ģ��������������Ա��Ҫ��������Ƕ���һ��fb_info �ṹ��(�ýṹ���ں��ṩ)��Ȼ�����ṹ���е�����������Ӧ�ĳ�ʼ�����ύ���ں˾Ϳ����ˡ� 

�����ļ��ڣ� ��kernel/drivers/video/samsung/Ŀ¼�¡�
s3cfb_main.c ����-�������� 
*/
////��Ҫ���lcd��ƽ̨�豸��Ϣ
static void __init smdk4x12_machine_init(void)
{
	s3cfb_set_platdata(NULL);//ѡ��Ĭ������
	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));//ע��ƽ̨�豸
}

static struct platform_device *smdk4x12_devices[] __initdata = {
	/* legacy fimd */
	#ifdef CONFIG_FB_S5P
	&s3c_device_fb,
	#ifdef CONFIG_FB_S5P_LMS501KF03//���δ���� ��ʹ�õ���config_for_android_scp_elite�����ļ�
	&s3c_device_spi_gpio,
	#endif
}

struct platform_device s3c_device_fb = {
	.name		= "s3cfb",////�豸����
#if defined(CONFIG_ARCH_EXYNOS4)
	.id		= 0,
#else
	.id		= -1,
#endif
	.num_resources	= ARRAY_SIZE(s3cfb_resource),
	.resource	= s3cfb_resource,
	.dev		= {
		.dma_mask		= &fb_dma_mask,
		.coherent_dma_mask	= 0xffffffffUL
	}
};


static struct resource s3cfb_resource[] = {
	[0] = {
		.start	= S5P_PA_FIMD0,
		.end	= S5P_PA_FIMD0 + SZ_32K - 1,
		.flags	= IORESOURCE_MEM,//��Դ��ַ
	},
	[1] = {
		.start	= IRQ_FIMD0_VSYNC,
		.end	= IRQ_FIMD0_VSYNC,
		.flags	= IORESOURCE_IRQ,//�жϵ�ַ
	},
	[2] = {
		.start	= IRQ_FIMD0_FIFO,
		.end	= IRQ_FIMD0_FIFO,
		.flags	= IORESOURCE_IRQ,////fifo��ַ
	},
};

static u64 fb_dma_mask = 0xffffffffUL;//dma ����

static struct s3c_platform_fb default_fb_data __initdata = {
#if defined(CONFIG_ARCH_EXYNOS4)
	.hw_ver	= 0x70,
#else
	.hw_ver	= 0x62,
#endif
	.nr_wins	= 5,//֧�ֵĴ�����
#if defined(CONFIG_FB_S5P_DEFAULT_WINDOW)
	.default_win	= CONFIG_FB_S5P_DEFAULT_WINDOW,//ָ��Ĭ����ʾ����
#else
	.default_win	= 0,
#endif
	.swap		= FB_SWAP_WORD | FB_SWAP_HWORD,
};

void __init s3cfb_set_platdata(struct s3c_platform_fb *pd)
{
	struct s3c_platform_fb *npd;
	int i;

	if (!pd)
		pd = &default_fb_data;//pd����NULLָ�룬��pdĬ��Ϊdefault_fb_data

	npd = kmemdup(pd, sizeof(struct s3c_platform_fb), GFP_KERNEL);
	if (!npd)
		printk(KERN_ERR "%s: no memory for platform data\n", __func__);
	else {
		for (i = 0; i < npd->nr_wins; i++)////��ʼ��ÿ�����ڵ�id
			npd->nr_buffers[i] = 1;

#if defined(CONFIG_FB_S5P_NR_BUFFERS)
		npd->nr_buffers[npd->default_win] = CONFIG_FB_S5P_NR_BUFFERS;
#else
		npd->nr_buffers[npd->default_win] = 1;
#endif

		//������Щ���������� driver ƥ������������
		s3cfb_get_clk_name(npd->clk_name);//��ȡʱ�ӻ���豸ʱ�ӣ����
		npd->cfg_gpio = s3cfb_cfg_gpio;//��ȡ���Ų�������
		npd->backlight_on = s3cfb_backlight_on;//������
		npd->backlight_off = s3cfb_backlight_off;//�ر���
		npd->lcd_on = s3cfb_lcd_on;//ʹ��lcd�豸
		npd->lcd_off = s3cfb_lcd_off;//�ر�lcd �豸
		npd->clk_on = s3cfb_clk_on;// ʱ�ӿ�
		npd->clk_off = s3cfb_clk_off;//ʱ�ӹ�

		s3c_device_fb.dev.platform_data = npd;
	}
}


/*-----------------------------------------------------------------------------------
samsung s3cfb_main.c ����
-----------------------------------------------------------------------------------*/

static const struct dev_pm_ops s3cfb_pm_ops = {
	.runtime_suspend = s3cfb_runtime_suspend,
	.runtime_resume = s3cfb_runtime_resume,
};
#endif

static struct platform_driver s3cfb_driver = {
	.probe		= s3cfb_probe,
	.remove		= s3cfb_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= s3cfb_suspend,
	.resume		= s3cfb_resume,
#endif
	.driver		= {
		.name	= S3CFB_NAME,//�����õ���s3cfb
		.owner	= THIS_MODULE,
#ifdef CONFIG_EXYNOS_DEV_PD
		.pm	= &s3cfb_pm_ops,
#endif
	},
};

static int s3cfb_register(void)
{
	platform_driver_register(&s3cfb_driver);
	return 0;
}



static int s3cfb_probe(struct platform_device *pdev)
{
	struct s3c_platform_fb *pdata = NULL;//s3cƽ̨��Դ�ṹ��
	struct resource *res = NULL;//��Դָ��
	struct s3cfb_global *fbdev[2];//lcd driverȫ��ָ��ṹ��ָ�룬�Ƿ��������ĺ���ָ��
	int ret = 0;
	int i = 0;

#ifdef CONFIG_EXYNOS_DEV_PD
	/* to use the runtime PM helper functions */
	pm_runtime_enable(&pdev->dev);
	/* enable the power domain */
	pm_runtime_get_sync(&pdev->dev);
#endif
#ifndef CONFIG_TC4_EVT
	lcd_regulator = regulator_get(NULL, "vdd33_lcd");  
	if (IS_ERR(lcd_regulator)) {
		printk("%s: failed to get %s\n", __func__, "vdd33_lcd");
		ret = -ENODEV;
		goto err_regulator;
	}
	regulator_enable(lcd_regulator); 	//yulu
#endif

	fbfimd = kzalloc(sizeof(struct s3cfb_fimd_desc), GFP_KERNEL);//����fimd �豸����

	if (FIMD_MAX == 2)// �����豸dual �ж�,������Ȼֻ��һ�����������s3cfb.h ��
		fbfimd->dual = 1;
	else
		fbfimd->dual = 0;

	for (i = 0; i < FIMD_MAX; i++) {//����Ҫ������fimd�����з���ռ䣬��ʼ���Ȳ���
		/* global structure *///����ȫ��������
		fbfimd->fbdev[i] = kzalloc(sizeof(struct s3cfb_global), GFP_KERNEL);
		fbdev[i] = fbfimd->fbdev[i];
		if (!fbdev[i]) {
			dev_err(fbdev[i]->dev, "failed to allocate for	\
				global fb structure fimd[%d]!\n", i);
			goto err0;
		}
		/*��globalָ����platform ���豸��Ҳ����ƽ̨�豸������struct platform_device s3c_device_fb �������豸��io
    �ж���Դ��*/
		fbdev[i]->dev = &pdev->dev;
		/*
		��������ǻ�ȡ�����豸�Ĳ����ģ���s3cfb_wa101s.c�й�,��������������ǿ������ͨ��Ӳ�����뿪��ѡ��lcdӲ��
    ��������ϸ����
		*/
		s3cfb_set_lcd_info(fbdev[i]);

		/* platform_data*/
        //ͨ������������������ܵ�s3c_platform_fb
        //��dev-fimd-s5p.c ���� s3cfb_set_platdat ������
        /*
            struct s3c_platform_fb *npd;........
            s3cfb_get_clk_name(npd->clk_name);
            npd->cfg_gpio = s3cfb_cfg_gpio;
            npd->backlight_on = s3cfb_backlight_on;
            npd->backlight_off = s3cfb_backlight_off;
            npd->lcd_on = s3cfb_lcd_on;
            npd->lcd_off = s3cfb_lcd_off;
            npd->clk_on = s3cfb_clk_on;
            npd->clk_off = s3cfb_clk_off;
            
            s3c_device_fb.dev.platform_data = npd;
            */
		*/
		pdata = to_fb_plat(&pdev->dev);
		if (pdata->cfg_gpio)
			pdata->cfg_gpio(pdev);//��ʼ��io��������ϸ����

		if (pdata->clk_on)
			pdata->clk_on(pdev, &fbdev[i]->clock);//��ʼ��ʱ�ӣ�������ϸ����

		/* io memory */
		//��ƽ̨�豸�л�ȡio��Դ������io��Դ��ӳ��io��Դ
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res) {
			dev_err(fbdev[i]->dev,
				"failed to get io memory region\n");
			ret = -EINVAL;
			goto err1;
		}
		res = request_mem_region(res->start,
					res->end - res->start + 1, pdev->name);
		if (!res) {
			dev_err(fbdev[i]->dev,
				"failed to request io memory region\n");
			ret = -EINVAL;
			goto err1;
		}
		fbdev[i]->regs = ioremap(res->start, res->end - res->start + 1);
		if (!fbdev[i]->regs) {
			dev_err(fbdev[i]->dev, "failed to remap io region\n");
			ret = -EINVAL;
			goto err1;
		}

		/* irq */
		//��ƽ̨������ȡframe�ж�
		fbdev[i]->irq = platform_get_irq(pdev, 0);
		if (request_irq(fbdev[i]->irq, s3cfb_irq_frame, IRQF_SHARED,
				pdev->name, fbdev[i])) {
			dev_err(fbdev[i]->dev, "request_irq failed\n");
			ret = -EINVAL;
			goto err2;
		}

		// ���б�Ҫ��ȡfifo�ж�
#ifdef CONFIG_FB_S5P_TRACE_UNDERRUN
		if (request_irq(platform_get_irq(pdev, 1), s3cfb_irq_fifo,
				IRQF_DISABLED, pdev->name, fbdev[i])) {
			dev_err(fbdev[i]->dev, "request_irq failed\n");
			ret = -EINVAL;
			goto err2;
		}

		s3cfb_set_fifo_interrupt(fbdev[i], 1);
		dev_info(fbdev[i]->dev, "fifo underrun trace\n");
#endif


		/* hw setting */
		/*����˼�����Ƕ�Ӳ���ĳ�ʼ����������Ҫ�Ƕ�exynos4412�ļĴ�����������
		����� s3cfb_fimd6x.c �еļĴ�������������������ϸ����*/
		s3cfb_init_global(fbdev[i]);
		fbdev[i]->system_state = POWER_ON;//�����豸����״̬power on

		/* alloc fb_info */
		/* ���������s3cfb_ops.c �ļ���   -------fb ��������*/
    /*�������������������struct fb_info �ṹ�壬��ʼ��fb_info �ṹ�����Ϣ*/
    /*fb_info �ṹ�� �����ں˽ӿ���ϵĹ�ϵ������дlcd�������ǳ��˳�ʼ�����Ӳ������*/
    /*���ǰ�fb_info �ṹ���ʼ���� ע�ᵽ�ںˣ��� fb_mem ���ã��ϲ���ܸ��ײ�������*/
    /*�ú����������ݽ��ڽ���������*/
		if (s3cfb_alloc_framebuffer(fbdev[i], i)) {
			dev_err(fbdev[i]->dev, "alloc error fimd[%d]\n", i);
			goto err3;
		}

		/* register fb_info */
		/*���������s3cfb_ops.c �ļ��� ��Ҫ�����ں�ע��fb_info ,�����Ժ����*/
		if (s3cfb_register_framebuffer(fbdev[i])) {
			dev_err(fbdev[i]->dev, "register error fimd[%d]\n", i);
			goto err3;
		}

		/* enable display */
		/*s3cfb_set_clock ��VIDCON0 ������Ӧ��ʱ�Ӳ���*/
		s3cfb_set_clock(fbdev[i]);
		/*ѡ��windowsͨ����ʹ��wins����*/
		s3cfb_enable_window(fbdev[0], pdata->default_win);
#ifdef CONFIG_FB_S5P_SOFTBUTTON_UI /* Add Menu UI */
		s3cfb_enable_window(fbdev[0], 4);
#endif

		/*���ô���״̬*/
		s3cfb_update_power_state(fbdev[i], pdata->default_win,
					FB_BLANK_UNBLANK);
	/*ʹ��lcdģ��,����exynos�Ļ�������������Ӳ���й�*/
  /*���庯���� s3cfb_fimd6x.c ���ڽ���������*/
		s3cfb_display_on(fbdev[i]);

#ifdef CONFIG_HAS_WAKELOCK
#ifdef CONFIG_HAS_EARLYSUSPEND
		fbdev[i]->early_suspend.suspend = s3cfb_early_suspend;
		fbdev[i]->early_suspend.resume = s3cfb_late_resume;
		fbdev[i]->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;

		register_early_suspend(&fbdev[i]->early_suspend);
#endif
#endif
	}
#ifdef CONFIG_FB_S5P_LCD_INIT
	/* panel control */
	if (pdata->backlight_on)
		pdata->backlight_on(pdev);

	if (pdata->lcd_on)
		pdata->lcd_on(pdev);
#endif

	/*��/sys/class/�´���һ�������ļ�*/  
	ret = device_create_file(&(pdev->dev), &dev_attr_win_power);
	if (ret < 0)
		dev_err(fbdev[0]->dev, "failed to add sysfs entries\n");

	dev_info(fbdev[0]->dev, "registered successfully\n");
	return 0;
#ifndef CONFIG_TC4_EVT
err_regulator:
	regulator_put(lcd_regulator);	//yulu
#endif
err3:
	for (i = 0; i < FIMD_MAX; i++)
		free_irq(fbdev[i]->irq, fbdev[i]);
err2:
	for (i = 0; i < FIMD_MAX; i++)
		iounmap(fbdev[i]->regs);
err1:
	for (i = 0; i < FIMD_MAX; i++)
		pdata->clk_off(pdev, &fbdev[i]->clock);
err0:
	return ret;
}
/*
probe��������Ҫ���ú�����:

��ȡƽ̨�豸 device�е���Դ
���豸����һ����Ӧ�ĳ�ʼ��
������fb_info ,����Ҫ����������
���ں��ύ��fb_info
ʹ���豸��
���������ļ�
����probe���ܣ�remove������Ӧ�������.
*/
/*-----------------------------------------------------------------------------------
probe ���漰������ϸ����
-----------------------------------------------------------------------------------*/
/* name should be fixed as 's3cfb_set_lcd_info' */
void s3cfb_set_lcd_info(struct s3cfb_global *ctrl)
{
    s3cfb_setup_lcd(); //��Ӳ��ѡ���豸����ʼ����Ӧ����
    wa101.init_ldi = NULL;
    ctrl->lcd = &wa101; //��ȫ�ֽṹ��ָ����豸
}
//wa101��ʲô
static struct s3cfb_lcd wa101 = {
/* add by cym 20130417 for TSC2007 TouchScreen */
#ifdef CONFIG_TOUCHSCREEN_TSC2007
	.width	= 800,
	.height = 480,
#else
	.width	= 1024,
	.height = 768,
#endif
#endif
	.bpp	= 24,
	.freq	= 70,//70,
    ......
}
//�ýṹ�����£�
struct s3cfb_lcd {
      int   width;//�豸��
      int   height;//�豸��
      int   bpp;//�豸��bpp
      int   freq;//ˢ���l��
      struct    s3cfb_lcd_timing timing;  //��Ӳ��ʱ�����
      struct    s3cfb_lcd_polarity polarity;
      void  (*init_ldi)(void);
      void  (*deinit_ldi)(void);
 };
 //�ɴ˿��Կ���wa101 ����һ������lcdӲ���豸�Ľṹ�塣
void s3cfb_setup_lcd()
{
	int type = get_lcd_type();
	......
	 else if(0x1 == type)   //7.0
        {
		wa101.width = 800;
                wa101.height = 1280;
                wa101.bpp       = 24;
                wa101.freq = 50;//70;
        }
        ......
}
 //���type��ѡ��ʲô����Ӳ����ʼ��,�ɴ˿��Կ���type ��Ӳ�� gpc0(3) gpx0(6) ����Ӳ������,����ǲ��뿪������Ӧ����ԭ��
int get_lcd_type()
{
	value1 = gpio_get_value(EXYNOS4_GPC0(3));

	value2 = gpio_get_value(EXYNOS4_GPX0(6));
#endif	
	type = (value1<<1)|value2;

	printk("value1 = %d, value2 = %d, type = 0x%x\n", value1, value2, type);

	return type;
}

//pdata->cfg_gpio(pdev); /��ʼ��io
void __init s3cfb_set_platdata(struct s3c_platform_fb *pd)
{
	......
		s3cfb_get_clk_name(npd->clk_name);
		npd->cfg_gpio = s3cfb_cfg_gpio;//�����ﱻָ����ʼ��
	......
}
void s3cfb_cfg_gpio(struct platform_device *pdev)
{   ......
    s3cfb_gpio_setup_24bpp(EXYNOS4_GPF0(0), 8, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV4);
    s3cfb_gpio_setup_24bpp(EXYNOS4_GPF1(0), 8, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV4);
    s3cfb_gpio_setup_24bpp(EXYNOS4_GPF2(0), 8, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV4);
    s3cfb_gpio_setup_24bpp(EXYNOS4_GPF3(0), 4, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV4);
    ......
    //�������ŵĺ�����Ҫ�ǰ�GFP0(0-7) GPF1(0-7) GPF2(0-7) GPF3(0-3) ���ó�lcdģʽ�� 
}  
//pdata->clk_on(pdev, &fbdev[i]->clock);ͬ��
void __init s3cfb_set_platdata(struct s3c_platform_fb *pd)
{
	......
		npd->clk_on = s3cfb_clk_on;//�����ﱻָ����ʼ��
	......
}
//��Ӳ���ĳ�ʼ����������Ҫ�Ƕ�exynos4412�ļĴ����������л���� s3cfb_fimd6x.c �еļĴ�������
int s3cfb_init_global(struct s3cfb_global *fbdev)
{
    fbdev->output = OUTPUT_RGB; //ָ���������ʽ
    fbdev->rgb_mode = MODE_RGB_P;//ָ����rgbģʽ

    fbdev->wq_count = 0; //�ȴ����м�������
    init_waitqueue_head(&fbdev->wq);//��ʼ���ȵ�����
    mutex_init(&fbdev->lock);//��ʼ����

    s3cfb_set_output(fbdev); //���������ʽ
    s3cfb_set_display_mode(fbdev);//����ģʽ
    s3cfb_set_polarity(fbdev);//�������ż���
    s3cfb_set_timing(fbdev);//����ʱ��
    s3cfb_set_lcd_size(fbdev);//����lcd��С

    return 0;
}
/*
�ۺϣ�Ҫ����lcd�豸�� 
exynos Ҫ���ó�lcd������Ч����Ӧģʽ 
de��-�ߵ�ƽ�����C����ת 
clk���½��ش��� ��-��Ӧλ���ó�0 
hsync���ǵ����崥����-��ת 
vsyns ���ǵ����崥������ת
*/

/*����������ʮ����Ҫ�ĺ���s3cfb_alloc_framebuffer��������ʵ����fbi*/
int s3cfb_alloc_framebuffer(struct s3cfb_global *fbdev, int fimd_id)
{
	struct s3c_platform_fb *pdata = to_fb_plat(fbdev->dev);
	int ret = 0;
	int i;

	fbdev->fb = kmalloc(pdata->nr_wins *
				sizeof(struct fb_info *), GFP_KERNEL);//���������Ҫ�Ľṹ�����ڴ�ռ� 
	if (!fbdev->fb) {
		dev_err(fbdev->dev, "not enough memory\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	for (i = 0; i < pdata->nr_wins; i++) {
		fbdev->fb[i] = framebuffer_alloc(sizeof(struct s3cfb_window),
						fbdev->dev);/* ����fb_info�ṹ�壬����һ��fb_info�ṹ���ַ������ṹ������ûʲô���ݣ�
	ֻ��ֵ��par(win����ʼ��ַ)��device (���豸)�������������Ұ� fb_info �е�dev ָ�� fbdev->dev 
�� fb_info->par ָ�� s3cfb_windows*/
		if (!fbdev->fb[i]) {
			dev_err(fbdev->dev, "not enough memory\n");
			ret = -ENOMEM;
			goto err_alloc_fb;
		}

		/*��Ҫ�ǳ�ʼ��fb_info �ṹ�壬��var ��fix��������....*/
		ret = s3cfb_init_fbinfo(fbdev, i);
		if (ret) {
			dev_err(fbdev->dev,
				"failed to allocate memory for fb%d\n", i);
			ret = -ENOMEM;
			goto err_alloc_fb;
		}

#ifdef CONFIG_FB_S5P_SOFTBUTTON_UI
		if (i == pdata->default_win || i == 4)
#else
		if (i == pdata->default_win)
#endif	
			 /*��Ҫ��Ϊ���������RGB���ݵĿռ䡣���÷���һ����DMA��*/
			if (s3cfb_map_default_video_memory(fbdev,
						fbdev->fb[i], fimd_id)) {
				dev_err(fbdev->dev,
				"failed to map video memory "
				"for default window (%d)\n", i);
			ret = -ENOMEM;
			goto err_alloc_fb;
		}
	}

	return 0;

err_alloc_fb:
	for (i = 0; i < pdata->nr_wins; i++) {
		if (fbdev->fb[i])
			framebuffer_release(fbdev->fb[i]);
	}
	kfree(fbdev->fb);

err_alloc:
	return ret;
}

/*����������s3cfb_init_fbinfo����*/
int s3cfb_init_fbinfo(struct s3cfb_global *fbdev, int id)
{
    /*
        �Ըոտ��ٵ�fb_info �ռ��е���Ӧ��ַ ������ȡ����������������
                (��fb_info ����س�Ա����Ϣ�������������ϣ����ﲻ������)
                ����������Ҫ���߾���һ���Ĺ��ף�
    */
    struct fb_info *fb = fbdev->fb[id];
    struct fb_fix_screeninfo *fix = &fb->fix;
    struct fb_var_screeninfo *var = &fb->var;
    struct s3cfb_window *win = fb->par;
    struct s3cfb_alpha *alpha = &win->alpha;
    struct s3cfb_lcd *lcd = fbdev->lcd;
    struct s3cfb_lcd_timing *timing = &lcd->timing;

    /*�Դ��������գ��ɴ˿ɼ� fb->par �����á�����fix ->id   */
    memset(win, 0, sizeof(struct s3cfb_window));
    platform_set_drvdata(to_platform_device(fbdev->dev), fb);
    strcpy(fix->id, S3CFB_NAME);

    /*    ָ������id����������·����dma burst ��win��win->power_state ״̬������͸���ȷ�ʽ*/
    /* fimd specific */
    win->id = id;
    /*ѡ��������Դ*/
    /*enum s3cfb_data_path_t {
    DATA_PATH_FIFO = 0,
    DATA_PATH_DMA = 1,
    DATA_PATH_IPC = 2,
    };*/
    win->path = DATA_PATH_DMA;
    /*����dma_burst ����С��Χ���Ը������������ֲ�������뿴��ͼ*/
    win->dma_burst = 16;
    s3cfb_update_power_state(fbdev, win->id, FB_BLANK_POWERDOWN);
    alpha->mode = PLANE_BLENDING;

    /* fbinfo */
    /*   ����fb->fbops = &s3cfb_ops;  s3cfb_ops ��һ��ȫ�ֱ����ڣ�s3cfb_main ��ָ����
    
				    struct fb_ops s3cfb_ops = {
					.owner		= THIS_MODULE,
					.fb_open	= s3cfb_open,
					.fb_release	= s3cfb_release,
					.fb_check_var	= s3cfb_check_var,���ɱ��������������֧�ֵ�ֵ
					.fb_set_par	= s3cfb_set_par,/����info->var����videoģʽ
					.fb_setcolreg	= s3cfb_setcolreg,����color�Ĵ���*
					.fb_blank	= s3cfb_blank,��ʾ�հ�
					.fb_pan_display	= s3cfb_pan_display,
					.fb_fillrect	= cfb_fillrect,
					.fb_copyarea	= cfb_copyarea,
					.fb_imageblit	= cfb_imageblit,
					.fb_cursor	= s3cfb_cursor,
					.fb_ioctl	= s3cfb_ioctl,
				};
    
    */
    fb->fbops = &s3cfb_ops;
    fb->flags = FBINFO_FLAG_DEFAULT;//  Ȼ��������FBINFO
    fb->pseudo_palette = &win->pseudo_pal;//��������ĵ�ɫ���ַ
#if (CONFIG_FB_S5P_NR_BUFFERS != 1) // ���� ƫ�ƣ� һ��Ϊ0
    fix->xpanstep = 2;
    fix->ypanstep = 1;
#else
    fix->xpanstep = 0;
    fix->ypanstep = 0;
#endif
    /*  ����type --- FB_TYPE_PACKED_PIXELS   -----�������ڴ��Ӧ��TFT���ǻ�����������ڴ档�����豸����ͬѡ��*/
     fix->type = FB_TYPE_PACKED_PIXELS;
    fix->accel = FB_ACCEL_NONE;//�޴��豸
    /*-----������ʾ��ʽ��ʣ���Ȼ���кڰ�*/
    /*��������ʾ��ʽ��������������*/
    fix->visual = FB_VISUAL_TRUECOLOR;
    /*���ÿɱ������ߣ�lcd ��һ��ָ�룬ָ����wa101 �ṹ�壬�����ǵ÷񣿣�*/
    var->xres = lcd->width;
    var->yres = lcd->height;

     /*��������ֱ���,Ƕ��ʽ�豸һ�㲻�÷ֱ���*/
     /*����ͨ�����ó�  xres��yres һ��*/
     /*������Ϊ����������*/
#if defined(CONFIG_FB_S5P_VIRTUAL)
    var->xres_virtual = CONFIG_FB_S5P_X_VRES;
    var->yres_virtual = CONFIG_FB_S5P_Y_VRES * CONFIG_FB_S5P_NR_BUFFERS;
#else
    var->xres_virtual = var->xres;
    var->yres_virtual = var->yres * CONFIG_FB_S5P_NR_BUFFERS;
#endif
    /* ���ó� 32 bpp �ķֱ��ʣ������һ�䣺 �ֱ����� WINCONx �Ĵ�����BBPMODE����������Կ�������֧�ֵĸ�ʽ�кܶࡣ������д����(��ϧ,Ҳ�����Ǹ�����Ӳ������,Ҳ������Ϊ��һ�����������Ŀռ䣬��bpp���Բ���
��������ռ䣬bpp���¼���(�²�)
    */
    var->bits_per_pixel = 32;
    /*����xoffset ��yoffset ƫ�� ��Ϊ0*/
    /*��Ϊ0 �Ļ���var->xoffset = var->xres_virtual - var->xres - 1*/
    var->xoffset = 0;
    /*��Ϊ0�Ļ���var->yoffset = var->yres_virtual - var->yres - 1;*/
    var->yoffset = 0;
    var->width = 0;
    var->height = 0;
    var->transp.length = 0;

    fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;
    fix->smem_len = fix->line_length * var->yres_virtual;

    var->nonstd = 0;//----��׼��ʽ ,!=0��Ϊ�Ǳ�׼��ʽ
    var->activate = FB_ACTIVATE_NOW;//----��ȫӦ��
    var->vmode = FB_VMODE_NONINTERLACED;//-----����ɨ��
    // ����������timing
    var->hsync_len = timing->h_sw;
    var->vsync_len = timing->v_sw;
    var->left_margin = timing->h_bp;
    var->right_margin = timing->h_fp;
    var->upper_margin = timing->v_bp;
    var->lower_margin = timing->v_fp;
    //������Ӧ��������pixclock��ֵ��
    var->pixclock = (lcd->freq *
        (var->left_margin + var->right_margin
        + var->hsync_len + var->xres) *
        (var->upper_margin + var->lower_margin
        + var->vsync_len + var->yres));
    var->pixclock = KHZ2PICOS(var->pixclock/1000);

    //����fb��R/G/Bλ��,-----Ҳ����RGB�ĸ�ʽ������ǳ��򵥣���������
    s3cfb_set_bitfield(var);
    /*
    ����͸���� ģʽ
    ����channel---0 ͨ��
    ����value -------���ֵ��͸��
    */
    s3cfb_set_alpha_info(var, win);

    return 0;
}












