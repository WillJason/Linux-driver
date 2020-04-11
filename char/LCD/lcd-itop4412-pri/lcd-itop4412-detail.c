/*
fbmem.c文件提供了framebuffer驱动程序的通用文件操作接口，自定义的framebuffer驱动程序可以使用fbmem.c中提供默认的接口。用EXPORT_SYMBOL导出到其他文件中应用
lcd的应用层 通过 内核的fbmem接口 再调用驱动xxxfb.c的内容 
而fbmem接口是内核提供的，所有驱动设计人员主要的任务就是定义一个fb_info 结构体(该结构由内核提供)，然后填充结构体中的内容做好相应的初始化后，提交给内核就可以了。 

驱动文件在： 在kernel/drivers/video/samsung/目录下。
s3cfb_main.c ——-总线驱动 
*/
////需要添加lcd的平台设备信息
static void __init smdk4x12_machine_init(void)
{
	s3cfb_set_platdata(NULL);//选用默认配置
	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));//注册平台设备
}

static struct platform_device *smdk4x12_devices[] __initdata = {
	/* legacy fimd */
	#ifdef CONFIG_FB_S5P
	&s3c_device_fb,
	#ifdef CONFIG_FB_S5P_LMS501KF03//这个未定义 我使用的是config_for_android_scp_elite配置文件
	&s3c_device_spi_gpio,
	#endif
}

struct platform_device s3c_device_fb = {
	.name		= "s3cfb",////设备名字
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
		.flags	= IORESOURCE_MEM,//资源地址
	},
	[1] = {
		.start	= IRQ_FIMD0_VSYNC,
		.end	= IRQ_FIMD0_VSYNC,
		.flags	= IORESOURCE_IRQ,//中断地址
	},
	[2] = {
		.start	= IRQ_FIMD0_FIFO,
		.end	= IRQ_FIMD0_FIFO,
		.flags	= IORESOURCE_IRQ,////fifo地址
	},
};

static u64 fb_dma_mask = 0xffffffffUL;//dma 掩码

static struct s3c_platform_fb default_fb_data __initdata = {
#if defined(CONFIG_ARCH_EXYNOS4)
	.hw_ver	= 0x70,
#else
	.hw_ver	= 0x62,
#endif
	.nr_wins	= 5,//支持的窗口数
#if defined(CONFIG_FB_S5P_DEFAULT_WINDOW)
	.default_win	= CONFIG_FB_S5P_DEFAULT_WINDOW,//指定默认显示窗口
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
		pd = &default_fb_data;//pd传入NULL指针，故pd默认为default_fb_data

	npd = kmemdup(pd, sizeof(struct s3c_platform_fb), GFP_KERNEL);
	if (!npd)
		printk(KERN_ERR "%s: no memory for platform data\n", __func__);
	else {
		for (i = 0; i < npd->nr_wins; i++)////初始化每个窗口的id
			npd->nr_buffers[i] = 1;

#if defined(CONFIG_FB_S5P_NR_BUFFERS)
		npd->nr_buffers[npd->default_win] = CONFIG_FB_S5P_NR_BUFFERS;
#else
		npd->nr_buffers[npd->default_win] = 1;
#endif

		//下面这些函数都是在 driver 匹配后，在里面调用
		s3cfb_get_clk_name(npd->clk_name);//获取时钟获得设备时钟ｎａｍｅ
		npd->cfg_gpio = s3cfb_cfg_gpio;//获取引脚操作函数
		npd->backlight_on = s3cfb_backlight_on;//开背光
		npd->backlight_off = s3cfb_backlight_off;//关背光
		npd->lcd_on = s3cfb_lcd_on;//使能lcd设备
		npd->lcd_off = s3cfb_lcd_off;//关闭lcd 设备
		npd->clk_on = s3cfb_clk_on;// 时钟开
		npd->clk_off = s3cfb_clk_off;//时钟关

		s3c_device_fb.dev.platform_data = npd;
	}
}


/*-----------------------------------------------------------------------------------
samsung s3cfb_main.c 分析
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
		.name	= S3CFB_NAME,//这里用的是s3cfb
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
	struct s3c_platform_fb *pdata = NULL;//s3c平台资源结构体
	struct resource *res = NULL;//资源指针
	struct s3cfb_global *fbdev[2];//lcd driver全局指针结构体指针，是分析驱动的核心指针
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

	fbfimd = kzalloc(sizeof(struct s3cfb_fimd_desc), GFP_KERNEL);//创建fimd 设备描述

	if (FIMD_MAX == 2)// 接入设备dual 判断,这里显然只有一个，这个宏在s3cfb.h 中
		fbfimd->dual = 1;
	else
		fbfimd->dual = 0;

	for (i = 0; i < FIMD_MAX; i++) {//根据要描述的fimd，进行分配空间，初始化等操作
		/* global structure *///分配全局数据区
		fbfimd->fbdev[i] = kzalloc(sizeof(struct s3cfb_global), GFP_KERNEL);
		fbdev[i] = fbfimd->fbdev[i];
		if (!fbdev[i]) {
			dev_err(fbdev[i]->dev, "failed to allocate for	\
				global fb structure fimd[%d]!\n", i);
			goto err0;
		}
		/*让global指向了platform 的设备，也就是平台设备描述，struct platform_device s3c_device_fb 里面有设备的io
    中断资源等*/
		fbdev[i]->dev = &pdev->dev;
		/*
		这个函数是获取具体设备的参数的，跟s3cfb_wa101s.c有关,在这个函数中我们看到如何通过硬件拨码开关选择lcd硬件
    下面再详细介绍
		*/
		s3cfb_set_lcd_info(fbdev[i]);

		/* platform_data*/
        //通过这个函数获得上面介绍的s3c_platform_fb
        //在dev-fimd-s5p.c 中有 s3cfb_set_platdat 函数中
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
			pdata->cfg_gpio(pdev);//初始化io，下面详细介绍

		if (pdata->clk_on)
			pdata->clk_on(pdev, &fbdev[i]->clock);//初始化时钟，下面详细介绍

		/* io memory */
		//从平台设备中获取io资源，申请io资源，映射io资源
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
		//从平台驱动获取frame中断
		fbdev[i]->irq = platform_get_irq(pdev, 0);
		if (request_irq(fbdev[i]->irq, s3cfb_irq_frame, IRQF_SHARED,
				pdev->name, fbdev[i])) {
			dev_err(fbdev[i]->dev, "request_irq failed\n");
			ret = -EINVAL;
			goto err2;
		}

		// 如有必要获取fifo中断
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
		/*故名思议这是对硬件的初始化，里面主要是对exynos4412的寄存器设置所有
		会调到 s3cfb_fimd6x.c 中的寄存器操作，将在下面详细介绍*/
		s3cfb_init_global(fbdev[i]);
		fbdev[i]->system_state = POWER_ON;//设置设备工作状态power on

		/* alloc fb_info */
		/* 这个函数在s3cfb_ops.c 文件中   -------fb 操作集合*/
    /*这个函数的作用是申请struct fb_info 结构体，初始化fb_info 结构体的信息*/
    /*fb_info 结构体 上与内核接口耦合的关系，我们写lcd驱动就是除了初始化相关硬件以外*/
    /*就是把fb_info 结构体初始化后 注册到内核，供 fb_mem 调用，上层才能跟底层结合起来*/
    /*该函数具体内容将在接下来介绍*/
		if (s3cfb_alloc_framebuffer(fbdev[i], i)) {
			dev_err(fbdev[i]->dev, "alloc error fimd[%d]\n", i);
			goto err3;
		}

		/* register fb_info */
		/*这个函数在s3cfb_ops.c 文件中 主要是向内核注册fb_info ,将在以后介绍*/
		if (s3cfb_register_framebuffer(fbdev[i])) {
			dev_err(fbdev[i]->dev, "register error fimd[%d]\n", i);
			goto err3;
		}

		/* enable display */
		/*s3cfb_set_clock 向VIDCON0 配置相应的时钟参数*/
		s3cfb_set_clock(fbdev[i]);
		/*选择windows通道，使能wins窗口*/
		s3cfb_enable_window(fbdev[0], pdata->default_win);
#ifdef CONFIG_FB_S5P_SOFTBUTTON_UI /* Add Menu UI */
		s3cfb_enable_window(fbdev[0], 4);
#endif

		/*设置窗口状态*/
		s3cfb_update_power_state(fbdev[i], pdata->default_win,
					FB_BLANK_UNBLANK);
	/*使能lcd模块,设置exynos的基础器，所以与硬件有关*/
  /*具体函数在 s3cfb_fimd6x.c 将在接下来介绍*/
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

	/*在/sys/class/下创建一个属性文件*/  
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
probe函数的主要作用和流程:

获取平台设备 device中的资源
对设备做了一下相应的初始化
申请了fb_info ,根据要求进行了填充
向内核提交了fb_info
使能设备等
创建属性文件
这是probe功能，remove是做相应的逆操作.
*/
/*-----------------------------------------------------------------------------------
probe 内涉及函数详细分析
-----------------------------------------------------------------------------------*/
/* name should be fixed as 's3cfb_set_lcd_info' */
void s3cfb_set_lcd_info(struct s3cfb_global *ctrl)
{
    s3cfb_setup_lcd(); //由硬件选择设备，初始化相应参数
    wa101.init_ldi = NULL;
    ctrl->lcd = &wa101; //让全局结构体指向该设备
}
//wa101是什么
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
//该结构体如下：
struct s3cfb_lcd {
      int   width;//设备宽
      int   height;//设备高
      int   bpp;//设备的bpp
      int   freq;//刷新頻度
      struct    s3cfb_lcd_timing timing;  //与硬件时序参数
      struct    s3cfb_lcd_polarity polarity;
      void  (*init_ldi)(void);
      void  (*deinit_ldi)(void);
 };
 //由此可以看出wa101 就是一个描述lcd硬件设备的结构体。
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
 //获得type来选择什么样的硬件初始化,由此可以看出type 由硬件 gpc0(3) gpx0(6) 两个硬件决定,这就是拨码开关自适应屏的原理
int get_lcd_type()
{
	value1 = gpio_get_value(EXYNOS4_GPC0(3));

	value2 = gpio_get_value(EXYNOS4_GPX0(6));
#endif	
	type = (value1<<1)|value2;

	printk("value1 = %d, value2 = %d, type = 0x%x\n", value1, value2, type);

	return type;
}

//pdata->cfg_gpio(pdev); /初始化io
void __init s3cfb_set_platdata(struct s3c_platform_fb *pd)
{
	......
		s3cfb_get_clk_name(npd->clk_name);
		npd->cfg_gpio = s3cfb_cfg_gpio;//在这里被指定初始化
	......
}
void s3cfb_cfg_gpio(struct platform_device *pdev)
{   ......
    s3cfb_gpio_setup_24bpp(EXYNOS4_GPF0(0), 8, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV4);
    s3cfb_gpio_setup_24bpp(EXYNOS4_GPF1(0), 8, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV4);
    s3cfb_gpio_setup_24bpp(EXYNOS4_GPF2(0), 8, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV4);
    s3cfb_gpio_setup_24bpp(EXYNOS4_GPF3(0), 4, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV4);
    ......
    //设置引脚的函数主要是把GFP0(0-7) GPF1(0-7) GPF2(0-7) GPF3(0-3) 设置成lcd模式： 
}  
//pdata->clk_on(pdev, &fbdev[i]->clock);同理
void __init s3cfb_set_platdata(struct s3c_platform_fb *pd)
{
	......
		npd->clk_on = s3cfb_clk_on;//在这里被指定初始化
	......
}
//对硬件的初始化，里面主要是对exynos4412的寄存器设置所有会调到 s3cfb_fimd6x.c 中的寄存器操作
int s3cfb_init_global(struct s3cfb_global *fbdev)
{
    fbdev->output = OUTPUT_RGB; //指定了输出格式
    fbdev->rgb_mode = MODE_RGB_P;//指定了rgb模式

    fbdev->wq_count = 0; //等待队列技术清零
    init_waitqueue_head(&fbdev->wq);//初始化等到队列
    mutex_init(&fbdev->lock);//初始化锁

    s3cfb_set_output(fbdev); //设置输出格式
    s3cfb_set_display_mode(fbdev);//设置模式
    s3cfb_set_polarity(fbdev);//设置引脚极性
    s3cfb_set_timing(fbdev);//设置时序
    s3cfb_set_lcd_size(fbdev);//设置lcd大小

    return 0;
}
/*
综合：要驱动lcd设备： 
exynos 要设置成lcd触发有效的相应模式 
de—-高电平触发–不反转 
clk—下降沿触发 —-相应位设置成0 
hsync—是低脉冲触发—-反转 
vsyns —是低脉冲触发—反转
*/

/*接下来分析十分重要的函数s3cfb_alloc_framebuffer，在这里实现了fbi*/
int s3cfb_alloc_framebuffer(struct s3cfb_global *fbdev, int fimd_id)
{
	struct s3c_platform_fb *pdata = to_fb_plat(fbdev->dev);
	int ret = 0;
	int i;

	fbdev->fb = kmalloc(pdata->nr_wins *
				sizeof(struct fb_info *), GFP_KERNEL);//给这个最重要的结构分配内存空间 
	if (!fbdev->fb) {
		dev_err(fbdev->dev, "not enough memory\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	for (i = 0; i < pdata->nr_wins; i++) {
		fbdev->fb[i] = framebuffer_alloc(sizeof(struct s3cfb_window),
						fbdev->dev);/* 分配fb_info结构体，返回一个fb_info结构体地址，这个结构体现在没什么内容，
	只赋值了par(win的起始地址)和device (父设备)两个变量，并且把 fb_info 中的dev 指向 fbdev->dev 
把 fb_info->par 指向 s3cfb_windows*/
		if (!fbdev->fb[i]) {
			dev_err(fbdev->dev, "not enough memory\n");
			ret = -ENOMEM;
			goto err_alloc_fb;
		}

		/*主要是初始化fb_info 结构体，对var ，fix进行填充等....*/
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
			 /*主要是为窗体分配存放RGB数据的空间。（该分配一般用DMA）*/
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

/*现在来看看s3cfb_init_fbinfo函数*/
int s3cfb_init_fbinfo(struct s3cfb_global *fbdev, int id)
{
    /*
        对刚刚开辟的fb_info 空间中的相应地址 进行提取，方便接下来的填充
                (对fb_info 中相关成员的信息，请查阅相关资料，这里不做阐述)
                （所这里需要读者具有一定的功底）
    */
    struct fb_info *fb = fbdev->fb[id];
    struct fb_fix_screeninfo *fix = &fb->fix;
    struct fb_var_screeninfo *var = &fb->var;
    struct s3cfb_window *win = fb->par;
    struct s3cfb_alpha *alpha = &win->alpha;
    struct s3cfb_lcd *lcd = fbdev->lcd;
    struct s3cfb_lcd_timing *timing = &lcd->timing;

    /*对窗体进行清空，由此可见 fb->par 的作用。设置fix ->id   */
    memset(win, 0, sizeof(struct s3cfb_window));
    platform_set_drvdata(to_platform_device(fbdev->dev), fb);
    strcpy(fix->id, S3CFB_NAME);

    /*    指定窗体id，窗体数据路径，dma burst ，win的win->power_state 状态，设置透明度方式*/
    /* fimd specific */
    win->id = id;
    /*选择数据来源*/
    /*enum s3cfb_data_path_t {
    DATA_PATH_FIFO = 0,
    DATA_PATH_DMA = 1,
    DATA_PATH_IPC = 2,
    };*/
    win->path = DATA_PATH_DMA;
    /*设置dma_burst ，大小范围可以根据数据数据手册决定，请看下图*/
    win->dma_burst = 16;
    s3cfb_update_power_state(fbdev, win->id, FB_BLANK_POWERDOWN);
    alpha->mode = PLANE_BLENDING;

    /* fbinfo */
    /*   设置fb->fbops = &s3cfb_ops;  s3cfb_ops 是一个全局变量在，s3cfb_main 中指定：
    
				    struct fb_ops s3cfb_ops = {
					.owner		= THIS_MODULE,
					.fb_open	= s3cfb_open,
					.fb_release	= s3cfb_release,
					.fb_check_var	= s3cfb_check_var,检测可变参数，并调整到支持的值
					.fb_set_par	= s3cfb_set_par,/根据info->var设置video模式
					.fb_setcolreg	= s3cfb_setcolreg,设置color寄存器*
					.fb_blank	= s3cfb_blank,显示空白
					.fb_pan_display	= s3cfb_pan_display,
					.fb_fillrect	= cfb_fillrect,
					.fb_copyarea	= cfb_copyarea,
					.fb_imageblit	= cfb_imageblit,
					.fb_cursor	= s3cfb_cursor,
					.fb_ioctl	= s3cfb_ioctl,
				};
    
    */
    fb->fbops = &s3cfb_ops;
    fb->flags = FBINFO_FLAG_DEFAULT;//  然后是设置FBINFO
    fb->pseudo_palette = &win->pseudo_pal;//设置虚拟的调色板地址
#if (CONFIG_FB_S5P_NR_BUFFERS != 1) // 设置 偏移： 一般为0
    fix->xpanstep = 2;
    fix->ypanstep = 1;
#else
    fix->xpanstep = 0;
    fix->ypanstep = 0;
#endif
    /*  设置type --- FB_TYPE_PACKED_PIXELS   -----像素与内存对应，TFT就是基于这个管理内存。根据设备需求不同选择*/
     fix->type = FB_TYPE_PACKED_PIXELS;
    fix->accel = FB_ACCEL_NONE;//无此设备
    /*-----设置显示格式真彩，当然还有黑白*/
    /*索引等显示方式，请查阅相关资料*/
    fix->visual = FB_VISUAL_TRUECOLOR;
    /*设置可变参数宽高，lcd 是一个指针，指向了wa101 结构体，（还记得否？）*/
    var->xres = lcd->width;
    var->yres = lcd->height;

     /*设置虚拟分辨率,嵌入式设备一般不该分辨率*/
     /*所以通常设置成  xres和yres 一样*/
     /*这里是为了驱动兼容*/
#if defined(CONFIG_FB_S5P_VIRTUAL)
    var->xres_virtual = CONFIG_FB_S5P_X_VRES;
    var->yres_virtual = CONFIG_FB_S5P_Y_VRES * CONFIG_FB_S5P_NR_BUFFERS;
#else
    var->xres_virtual = var->xres;
    var->yres_virtual = var->yres * CONFIG_FB_S5P_NR_BUFFERS;
#endif
    /* 设置成 32 bpp 的分辨率，这里啰嗦一句： 分辨率由 WINCONx 寄存器中BBPMODE决定，你可以看到这里支持的格式有很多。而代码写死了(可惜,也可能是跟其他硬件兼容,也可能是为了一口气开辟最大的空间，低bpp可以不用
重新申请空间，bpp向下兼容(猜测)
    */
    var->bits_per_pixel = 32;
    /*设置xoffset ，yoffset 偏移 都为0*/
    /*不为0 的话：var->xoffset = var->xres_virtual - var->xres - 1*/
    var->xoffset = 0;
    /*不为0的话：var->yoffset = var->yres_virtual - var->yres - 1;*/
    var->yoffset = 0;
    var->width = 0;
    var->height = 0;
    var->transp.length = 0;

    fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;
    fix->smem_len = fix->line_length * var->yres_virtual;

    var->nonstd = 0;//----标准格式 ,!=0则为非标准格式
    var->activate = FB_ACTIVATE_NOW;//----完全应用
    var->vmode = FB_VMODE_NONINTERLACED;//-----正常扫描
    // 以下是设置timing
    var->hsync_len = timing->h_sw;
    var->vsync_len = timing->v_sw;
    var->left_margin = timing->h_bp;
    var->right_margin = timing->h_fp;
    var->upper_margin = timing->v_bp;
    var->lower_margin = timing->v_fp;
    //根据相应参数计算pixclock的值，
    var->pixclock = (lcd->freq *
        (var->left_margin + var->right_margin
        + var->hsync_len + var->xres) *
        (var->upper_margin + var->lower_margin
        + var->vsync_len + var->yres));
    var->pixclock = KHZ2PICOS(var->pixclock/1000);

    //设置fb的R/G/B位域,-----也就是RGB的格式，这里非常简单，不做描述
    s3cfb_set_bitfield(var);
    /*
    设置透明度 模式
    设置channel---0 通道
    设置value -------最大值不透明
    */
    s3cfb_set_alpha_info(var, win);

    return 0;
}












