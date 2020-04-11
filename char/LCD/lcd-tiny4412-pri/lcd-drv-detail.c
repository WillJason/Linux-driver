/*
fbmem.c文件提供了framebuffer驱动程序的通用文件操作接口，自定义的framebuffer驱动程序可以使用fbmem.c中提供默认的接口。用EXPORT_SYMBOL导出到其他文件中应用

s3c-fb.c是针对的三星开发板的lcd驱动文件接口（s3c_fb_probe等）。

fbmem_init 中实现了一个字符设备驱动，并创建了class，但是没有生成设备文件。

这个字符设备驱动的file_operations里面的函数，实质上都是从struct fb_info *registered_fb[FB_MAX]   这个

fb_info的结构体数组中去调用 fb_ops 这个结构体中函数指针。数组下标为次设备号。那么这个结构体是如何赋值的

    呢？

fbmem.c里定义 register_framebuffer这个函数。真正的显示设备都是调用这个函数来给registered_fb这个数组赋值，

然后再去创建设备文件。
*/

//需要添加lcd的平台设备信息
static void __init smdk4x12_machine_init(void)
{
	//...
	//初始化相关平台信息

	tiny4412_fb_init_pdata(&smdk4x12_lcd0_pdata);

	//需要添加lcd的平台设备信息

	s5p_fimd0_set_platdata(&smdk4x12_lcd0_pdata);
	//...
}

static struct s3c_fb_platdata smdk4x12_lcd0_pdata __initdata = {
	.win[0]		= &smdk4x12_fb_win0,
	.win[1]		= &smdk4x12_fb_win1,
	.win[2]		= &smdk4x12_fb_win2,
	.win[3]		= &smdk4x12_fb_win3,
	.win[4]		= &smdk4x12_fb_win4,
	.vtiming	= &smdk4x12_lcd_timing,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
	.setup_gpio	= exynos4_fimd0_gpio_setup_24bpp,
};

/*
在common.c里是这句 s5p_fb_setname(0,"exynos4-fb");
展开实际是这样s5p_device_fimd0.name = name;
这块被改为了 "exynos4-fb"
*/
struct platform_device s5p_device_fimd0 = {
	.name = "s5p-fb",
	.id = 0,
	.num_resources= ARRAY_SIZE(s5p_fimd0_resource),
	.resource  = s5p_fimd0_resource,
	.dev = {
		.dma_mask  = &samsung_device_dma_mask,
		.coherent_dma_mask= DMA_BIT_MASK(32),
		},
};

void __init s5p_fimd0_set_platdata(struct s3c_fb_platdata *pd)
{
s3c_set_platdata(pd, sizeof(struct s3c_fb_platdata),
&s5p_device_fimd0);
}

//lcd 所占用io口及终端资源。

static struct resource s5p_fimd0_resource[] = {
[0] = DEFINE_RES_MEM(S5P_PA_FIMD0, SZ_32K),
[1] = DEFINE_RES_IRQ(IRQ_FIMD0_VSYNC),
[2] = DEFINE_RES_IRQ(IRQ_FIMD0_FIFO),
[3] = DEFINE_RES_IRQ(IRQ_FIMD0_SYSTEM),
};

//放到struct platform_device *smdk4x12_device【】结构体中，接着调用platform_add_devices将lcd平台设备注册到内核。
//从以上分析，实际定义了两个设备，"s3c-fb","exynos4-fb"。
/*
现在，找到了，平台总线的设备和驱动后，我们要做的主要事情就是去修改lcd的各种参数，主要是fb_info结构体

    的fb_var_screeninfo结构体，这里面记录了lcd的主要9个参数。

   行前肩，行后肩，行同步信号脉宽，帧前肩，帧后肩，帧同步信号脉宽，像素时钟频率，x轴像素点，y轴像素点。
*/
/*-----------------------------------------------------------------------------------
samsung s3c-fb.c分析
-----------------------------------------------------------------------------------*/
static const struct dev_pm_ops s3cfb_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(s3c_fb_suspend, s3c_fb_resume)
	SET_RUNTIME_PM_OPS(s3c_fb_runtime_suspend, s3c_fb_runtime_resume,
			   NULL)
};

static struct platform_driver s3c_fb_driver = {
	.probe		= s3c_fb_probe,
	.remove		= __devexit_p(s3c_fb_remove),
	.id_table	= s3c_fb_driver_ids,
	.driver		= {
		.name	= "s3c-fb",
		.owner	= THIS_MODULE,
		.pm	= &s3cfb_pm_ops,
	},
};

module_platform_driver(s3c_fb_driver);


static struct platform_device_id s3c_fb_driver_ids[] = {
	{
		.name		= "s3c-fb",
		.driver_data	= (unsigned long)&s3c_fb_data_64xx,
	}, {
		.name		= "s5pc100-fb",
		.driver_data	= (unsigned long)&s3c_fb_data_s5pc100,
	}, {
		.name		= "s5pv210-fb",
		.driver_data	= (unsigned long)&s3c_fb_data_s5pv210,
	}, {
		.name		= "exynos4-fb",//本驱动使用这个*
		.driver_data	= (unsigned long)&s3c_fb_data_exynos4,
	}, {
		.name		= "exynos5-fb",
		.driver_data	= (unsigned long)&s3c_fb_data_exynos5,
	}, {
		.name		= "s3c2443-fb",
		.driver_data	= (unsigned long)&s3c_fb_data_s3c2443,
	}, {
		.name		= "s5p64x0-fb",
		.driver_data	= (unsigned long)&s3c_fb_data_s5p64x0,
	},
	{},
};

static struct s3c_fb_driverdata s3c_fb_data_exynos4 = {
	.variant = {
		.nr_windows	= 5,
		.vidtcon	= VIDTCON0,
		.wincon		= WINCON(0),
		.winmap		= WINxMAP(0),
		.keycon		= WKEYCON,
		.osd		= VIDOSD_BASE,
		.osd_stride	= 16,
		.buf_start	= VIDW_BUF_START(0),
		.buf_size	= VIDW_BUF_SIZE(0),
		.buf_end	= VIDW_BUF_END(0),

		.palette = {
			[0] = 0x2400,
			[1] = 0x2800,
			[2] = 0x2c00,
			[3] = 0x3000,
			[4] = 0x3400,
		},

		.has_shadowcon	= 1,
		.has_blendcon	= 1,
		.has_fixvclk	= 1,
	},
	.win[0]	= &s3c_fb_data_s5p_wins[0],
	.win[1]	= &s3c_fb_data_s5p_wins[1],
	.win[2]	= &s3c_fb_data_s5p_wins[2],
	.win[3]	= &s3c_fb_data_s5p_wins[3],
	.win[4]	= &s3c_fb_data_s5p_wins[4],
};


static int __devinit s3c_fb_probe(struct platform_device *pdev)
{
	/*因为一个驱动要适合很多版本的设备，每个版本的设备的设置 参数都
	不一样，所以要用到platid来选择哪个版本的设备，像“s5pv210-fb”,
	 "s3c2443-fb"..., 这些就是platid(也在s3c-fb.c中定义了)， 也表明了，这个驱动能适合于这些设备。  */
	const struct platform_device_id *platid;
	struct s3c_fb_driverdata *fbdrv;//driver data 
	struct device *dev = &pdev->dev;
	struct s3c_fb_platdata *pd;	// platform data 
	/*一个最重要的数据结据， 它代表了一个显示控制器，显示控制器的所有东东都放在这里了。
	但这里把它做成一个局部变量了。  */
	struct s3c_fb *sfb;
	struct s3c_fb_win *fbwin;
	struct resource *res;
	int win;
	int ret = 0;
	u32 reg;
	platid = platform_get_device_id(pdev);//从platform device 里 的id_entry 变量中获取platid，由一个宏实现  
	fbdrv = (struct s3c_fb_driverdata *)platid->driver_data;//获取platid对应的driver data，driver data在s3c-fb.c中定义，主要是一定设置参数  

	if (fbdrv->variant.nr_windows > S3C_FB_MAX_WIN) {
		dev_err(dev, "too many windows, cannot attach\n");
		return -EINVAL;
	}

	pd = pdev->dev.platform_data;// 获取platform data，它在板文件中定义，这个data里包含了关于显示控制器的数据，也包含了win的数据。  
	if (!pd) {
		dev_err(dev, "no platform data specified\n");
		return -EINVAL;
	}

	sfb = devm_kzalloc(dev, sizeof(struct s3c_fb), GFP_KERNEL);//给这个最重要的结构分配内存空间  
	if (!sfb) {
		dev_err(dev, "no memory for framebuffers\n");
		return -ENOMEM;
	}

	dev_dbg(dev, "allocate new framebuffer %p\n", sfb);

	sfb->dev = dev;//向sfb 填入 显示控制器的 device 结构体  
	sfb->pdata = pd;// 向sfb 填入 显示控制器的 platform data 结构体  
	sfb->variant = fbdrv->variant;// driver data结构体里 有 variant成员， 具体variant可以看下面s3c_fb_variant结构。

	spin_lock_init(&sfb->slock);
#if defined()
	mutex_init(&sfb->output_lock);
	INIT_LIST_HEAD(&sfb->update_regs_list);
	mutex_init(&sfb->upCONFIG_FB_ION_EXYNOSdate_regs_list_lock);
	init_kthread_worker(&sfb->update_regs_worker);

	sfb->update_regs_thread = kthread_run(kthread_worker_fn,
			&sfb->update_regs_worker, "s3c-fb");
	if (IS_ERR(sfb->update_regs_thread)) {
		int err = PTR_ERR(sfb->update_regs_thread);
		sfb->update_regs_thread = NULL;

		dev_err(dev, "failed to run update_regs thread\n");
		return err;
	}
	init_kthread_work(&sfb->update_regs_work, s3c_fb_update_regs_handler);
	sfb->timeline = sw_sync_timeline_create("s3c-fb");
	sfb->timeline_max = 1;
	/* XXX need to cleanup on errors */
#endif

	sfb->bus_clk = clk_get(dev, "lcd");//用"lcd"这个名字，去clock文件中找到自己的bus clock  
	if (IS_ERR(sfb->bus_clk)) {
		dev_err(dev, "failed to get bus clock\n");
		ret = PTR_ERR(sfb->bus_clk);
		goto err_sfb;
	}

	clk_enable(sfb->bus_clk);

	if (!sfb->variant.has_clksel) {
		sfb->lcd_clk = clk_get(dev, "sclk_fimd");//如果driver data里没定义 源时钟， 就用“sclk_fimd”此名字去clock文件找到自己的源时钟
		if (IS_ERR(sfb->lcd_clk)) {
			dev_err(dev, "failed to get lcd clock\n");
			ret = PTR_ERR(sfb->lcd_clk);
			goto err_bus_clk;
		}

		clk_enable(sfb->lcd_clk);
	}

	pm_runtime_enable(sfb->dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);/*获取资源的物理起始地址，终地址，
	大小，类型等，放在res结构中。实际上是寄存器们的物理起，终地址。  */
	if (!res) {
		dev_err(dev, "failed to find registers\n");
		ret = -ENOENT;
		goto err_lcd_clk;
	}

	sfb->regs = devm_request_and_ioremap(dev, res);/*分配内存, 内存映射，将寄
	存器的访问地址映射到刚才分配的内存上， sfb->regs为起始地址。   */
	if (!sfb->regs) {
		dev_err(dev, "failed to map registers\n");
		ret = -ENXIO;
		goto err_lcd_clk;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);//获取使用的中断号的开始地址和结束值
	if (!res) {
		dev_err(dev, "failed to acquire irq resource\n");
		ret = -ENOENT;
		goto err_lcd_clk;
	}
	sfb->irq_no = res->start;
	ret = devm_request_irq(dev, sfb->irq_no, s3c_fb_irq,
			  0, "s3c_fb", sfb);//注册中断
	if (ret) {
		dev_err(dev, "irq request failed\n");
		goto err_lcd_clk;
	}

	dev_dbg(dev, "got resources (regs %p), probing windows\n", sfb->regs);

	platform_set_drvdata(pdev, sfb);//将sfb 填入pdev->dev->p->driverdata 结构体中  
#if defined(CONFIG_FB_ION_EXYNOS)
	mutex_init(&sfb->vsync_info.irq_lock);

	ret = device_create_file(sfb->dev, &dev_attr_vsync);
	if (ret) {
		dev_err(sfb->dev, "failed to create vsync file\n");
	}
#endif

	/* setup gpio and output polarity controls */

	pd->setup_gpio();// 执行setup_gpio函数，此函数在setup_fimd0.c 中定义了。用来配置GPIO端口给FIMD使用。  

	writel(pd->vidcon1, sfb->regs + VIDCON1);//设置VIDCON1 寄存器 

	/* set video clock running at under-run */
	if (sfb->variant.has_fixvclk) {
		reg = readl(sfb->regs + VIDCON1);
		reg &= ~VIDCON1_VCLK_MASK;
		reg |= VIDCON1_VCLK_RUN;
		writel(reg, sfb->regs + VIDCON1);
	}

	/* zero all windows before we do anything */

	for (win = 0; win < fbdrv->variant.nr_windows; win++)
		s3c_fb_clear_win(sfb, win);/*将各个window的wincon寄存器清0，
		VIDOSDxA, VIDOSDxB, VIDOSDxC清0，禁止update各个window的shadow  */

	/* initialise colour key controls */
	for (win = 0; win < (fbdrv->variant.nr_windows - 1); win++) {
		void __iomem *regs = sfb->regs + sfb->variant.keycon;

		regs += (win * 8);
		writel(0xffffff, regs + WKEYCON0);
		writel(0xffffff, regs + WKEYCON1);
	}

#if defined(CONFIG_FB_ION_EXYNOS)
	sfb->fb_ion_client = ion_client_create(exynos_ion_dev,
			"fimd");
	if (IS_ERR(sfb->fb_ion_client)) {
		dev_err(sfb->dev, "failed to ion_client_create\n");
		goto err_pm_runtime;
	}
#endif

	s3c_fb_set_rgb_timing(sfb);

	/* we have the register setup, start allocating framebuffers */
	init_waitqueue_head(&sfb->vsync_info.wait);

	for (win = 0; win < fbdrv->variant.nr_windows; win++) {
		if (!pd->win[win])
			continue;

		ret = s3c_fb_probe_win(sfb, win, fbdrv->win[win],
				       &sfb->windows[win]);//分配 及 注册framebuffer的重要函数
		if (ret < 0) {
			dev_err(dev, "failed to create window %d\n", win);
			for (; win >= 0; win--)
				s3c_fb_release_win(sfb, sfb->windows[win]);//注册不成功的话就释放之前注册成功过的window  
			goto err_pm_runtime;
		}
	}

#if defined(CONFIG_FB_ION_EXYNOS)
	sfb->vsync_info.thread = kthread_run(s3c_fb_wait_for_vsync_thread,
					sfb, "s3c-fb-vsync");
	if (sfb->vsync_info.thread == ERR_PTR(-ENOMEM)) {
		dev_err(sfb->dev, "failed to run vsync thread\n");
		sfb->vsync_info.thread = NULL;
	}
#endif

#if defined(CONFIG_LOGO) && !defined(CONFIG_FRAMEBUFFER_CONSOLE)
	/* Start display and show logo on boot */
	fbwin = sfb->windows[0];

	/*显示开机logo*/
	if (fb_prepare_logo(fbwin->fbinfo, FB_ROTATE_UR)) {
#if defined(CONFIG_FB_ION_EXYNOS)
		fbwin->fbinfo->screen_base = ion_map_kernel(sfb->fb_ion_client,
				fbwin->dma_buf_data.ion_handle);
#endif

		printk("Start display and show logo\n");
		fb_set_cmap(&fbwin->fbinfo->cmap, fbwin->fbinfo);
		fb_show_logo(fbwin->fbinfo, FB_ROTATE_UR);

#if defined(CONFIG_FB_ION_EXYNOS)
		ion_unmap_kernel(sfb->fb_ion_client, fbwin->dma_buf_data.ion_handle);
#endif
	}
#endif

	platform_set_drvdata(pdev, sfb);/*再一次将sfb 填入pdev->dev->p->driverdata 结构体中，
	之前曾经这样操作过一次，现在再来一次，是因为sfb里的数据更新了很多  */

	return 0;

err_pm_runtime:
	pm_runtime_put_sync(sfb->dev);

err_lcd_clk:
	pm_runtime_disable(sfb->dev);

	if (!sfb->variant.has_clksel) {
		clk_disable(sfb->lcd_clk);
		clk_put(sfb->lcd_clk);
	}

err_bus_clk:
	clk_disable(sfb->bus_clk);
	clk_put(sfb->bus_clk);

err_sfb:
	return ret;
}

/*将显示控制器的结构体作为参数传进来，它有寄存器起始地址等丰富信息；还将win的号码也作为参数传进来  */
static int __devinit s3c_fb_probe_win(struct s3c_fb *sfb, unsigned int win_no,
		struct s3c_fb_win_variant *variant,
		struct s3c_fb_win **res)
		//driver data中win的部份, 它是由fbdrv->win[win]作参数传过来的。
		//per window private data for each framebuffer，它里面含有指向FBI(fb_info)结构体的指针
{
	struct fb_var_screeninfo *var;
	struct fb_videomode initmode;
	struct s3c_fb_pd_win *windata;//per window setup data,  也就是platform data中win的部份  
	struct s3c_fb_win *win;
	struct fb_info *fbinfo;
	int palette_size;
	int ret;

	dev_dbg(sfb->dev, "probing window %d, variant %p\n", win_no, variant);

	palette_size = variant->palette_sz * 4;//设置调色板大小 

	/* 分配fb_info结构体，返回一个fb_info结构体地址，这个结构体现在没什么内容，
	只赋值了par(win的起始地址)和device (父设备)两个变量，*/
	fbinfo = framebuffer_alloc(sizeof(struct s3c_fb_win) +
				   palette_size * sizeof(u32), sfb->dev);
	if (!fbinfo) {
		dev_err(sfb->dev, "failed to allocate framebuffer\n");
		return -ENOENT;
	}

	windata = sfb->pdata->win[win_no];//windata指向 platform data中win的部份   
	initmode = *sfb->pdata->vtiming;

	WARN_ON(windata->max_bpp == 0);
	WARN_ON(windata->xres == 0);
	WARN_ON(windata->yres == 0);

	win = fbinfo->par;
	*res = win;//par就是win的起始地址，现在把起始地址给*res，那么*res就是指向s3c_fb_win的指针   
	var = &fbinfo->var;//现在fbinfo->var还是空的， 只是将地址给var而已 
	win->variant = *variant;//将win的参数填进win->variant里   
	win->fbinfo = fbinfo;//-让win->fbinfo指向这个FBI结构实体   
	win->parent = sfb;//win的parent是显示控制器，所以它指向sfb结构体>   
	win->windata = windata;//>让win->windata指向 platform data中win的部分
	win->index = win_no;
	win->palette_buffer = (u32 *)(win + 1);

	ret = s3c_fb_alloc_memory(sfb, win);
	if (ret) {
		dev_err(sfb->dev, "failed to allocate display memory\n");
		return ret;
	}

	/* setup the r/b/g positions for the window's palette */
	if (win->variant.palette_16bpp) {
		/* Set RGB 5:6:5 as default */
		win->palette.r.offset = 11;
		win->palette.r.length = 5;
		win->palette.g.offset = 5;
		win->palette.g.length = 6;
		win->palette.b.offset = 0;
		win->palette.b.length = 5;

	} else {
		/* Set 8bpp or 8bpp and 1bit alpha */
		win->palette.r.offset = 16;
		win->palette.r.length = 8;
		win->palette.g.offset = 8;
		win->palette.g.length = 8;
		win->palette.b.offset = 0;
		win->palette.b.length = 8;
	}

	/* setup the initial video mode from the window */
	initmode.xres = windata->xres;
	initmode.yres = windata->yres;
	fb_videomode_to_var(&fbinfo->var, &initmode);//给FBI填上各个参数，此函数详见appendix  

	fbinfo->var.width	= windata->width;
	fbinfo->var.height	= windata->height;
	fbinfo->fix.type	= FB_TYPE_PACKED_PIXELS;
	fbinfo->fix.accel	= FB_ACCEL_NONE;
	fbinfo->var.activate	= FB_ACTIVATE_NOW;
	fbinfo->var.vmode	= FB_VMODE_NONINTERLACED;
	fbinfo->var.bits_per_pixel = windata->default_bpp;
	fbinfo->fbops		= &s3c_fb_ops;//对framebuffer的操作
	fbinfo->flags		= FBINFO_FLAG_DEFAULT;
	fbinfo->pseudo_palette = &win->pseudo_palette;

	/* prepare to actually start the framebuffer */

	ret = s3c_fb_check_var(&fbinfo->var, fbinfo);
	if (ret < 0) {
		dev_err(sfb->dev, "check_var failed on initial video params\n");
		return ret;
	}

	/* create initial colour map */

	ret = fb_alloc_cmap(&fbinfo->cmap, win->variant.palette_sz, 1);
	if (ret == 0)
		fb_set_cmap(&fbinfo->cmap, fbinfo);
	else
		dev_err(sfb->dev, "failed to allocate fb cmap\n");

	s3c_fb_set_par(fbinfo);

	dev_dbg(sfb->dev, "about to register framebuffer\n");

	/* run the check_var and set_par on our configuration. */

	ret = register_framebuffer(fbinfo);
	if (ret < 0) {
		dev_err(sfb->dev, "failed to register framebuffer\n");
		return ret;
	}

	dev_info(sfb->dev, "window %d: fb %s\n", win_no, fbinfo->fix.id);

	return 0;
}


static struct fb_ops s3c_fb_ops = {
	.owner			= THIS_MODULE,
	.fb_check_var	= s3c_fb_check_var,//检测可变参数，并调整到支持的值*/ 
	.fb_set_par		= s3c_fb_set_par,//根据info->var设置video模式*/
	.fb_blank		= s3c_fb_blank,//显示空白*/  
	.fb_setcolreg	= s3c_fb_setcolreg,//设置color寄存器*/  
	.fb_fillrect	= cfb_fillrect,//矩形填充drivers/video/cfblillrect.c里实现，把FB_CIRRUS打开*/
	.fb_copyarea	= cfb_copyarea,//数据复制drivers/video/cfbcopyarea.c*/  
	.fb_imageblit	= cfb_imageblit,//图形填充drivers/video/cfbimgblt.c*/  
	.fb_pan_display	= s3c_fb_pan_display,
	.fb_ioctl		= s3c_fb_ioctl,//**
#if defined(CONFIG_FB_ION_EXYNOS)
	.fb_mmap		= s3c_fb_mmap,
#endif
};



static int __devinit s3c_fb_alloc_memory(struct s3c_fb *sfb,
		struct s3c_fb_win *win)
{
	struct s3c_fb_pd_win *windata = win->windata;//platform data中win的部分 
	unsigned int real_size, virt_size, size;
	struct fb_info *fbi = win->fbinfo;// 让fbi指向FBI结构体  
	dma_addr_t map_dma;
#if defined(CONFIG_FB_ION_EXYNOS)
	struct ion_handle *handle;
	struct dma_buf *buf;
	int ret;
#endif

	dev_dbg(sfb->dev, "allocating memory for display\n");

	real_size = windata->xres * windata->yres;
	virt_size = windata->virtual_x * windata->virtual_y;

	dev_dbg(sfb->dev, "real_size=%u (%u.%u), virt_size=%u (%u.%u)\n",
		real_size, windata->xres, windata->yres,
		virt_size, windata->virtual_x, windata->virtual_y);

	size = (real_size > virt_size) ? real_size : virt_size;
	size *= (windata->max_bpp > 16) ? 32 : windata->max_bpp;
	size /= 8;

	fbi->fix.smem_len = size;//要分配的内存大小
	size = PAGE_ALIGN(size);

	dev_dbg(sfb->dev, "want %u bytes for window\n", size);
#if defined(CONFIG_FB_ION_EXYNOS) 
	handle = ion_alloc(sfb->fb_ion_client, (size_t)size, 0,
			ION_HEAP_SYSTEM_MASK/*ION_HEAP_EXYNOS_MASK*/, 0);
	if (IS_ERR(handle)) {
		dev_err(sfb->dev, "failed to ion_alloc\n");
		return -ENOMEM;
	}

	buf = ion_share_dma_buf(sfb->fb_ion_client, handle);
	if (IS_ERR_OR_NULL(buf)) {
		dev_err(sfb->dev, "ion_share_dma_buf() failed\n");
		goto err_share_dma_buf;
	}

	ret = s3c_fb_map_ion_handle(sfb, &win->dma_buf_data, handle, buf);
	if (!ret)
		goto err_map;
	map_dma = win->dma_buf_data.dma_addr;

#if defined(CONFIG_FRAMEBUFFER_CONSOLE)
	fbi->screen_base = ion_map_kernel(sfb->fb_ion_client,
			win->dma_buf_data.ion_handle);
#endif
#else
	fbi->screen_base = dma_alloc_writecombine(sfb->dev, size,
			&map_dma, GFP_KERNEL);//分配framebuffer的内存
	if (!fbi->screen_base)
		return -ENOMEM;

	dev_dbg(sfb->dev, "mapped %x to %p\n",
		(unsigned int)map_dma, fbi->screen_base);

	memset(fbi->screen_base, 0x0, size);
#endif
	fbi->fix.smem_start = map_dma;

	return 0;

#if defined(CONFIG_FB_ION_EXYNOS)
err_map:
	dma_buf_put(buf);
err_share_dma_buf:
	ion_free(sfb->fb_ion_client, handle);
	return -ENOMEM;
#endif
}








