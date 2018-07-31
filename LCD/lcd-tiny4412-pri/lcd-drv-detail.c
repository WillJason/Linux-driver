/*
fbmem.c�ļ��ṩ��framebuffer���������ͨ���ļ������ӿڣ��Զ����framebuffer�����������ʹ��fbmem.c���ṩĬ�ϵĽӿڡ���EXPORT_SYMBOL�����������ļ���Ӧ��

s3c-fb.c����Ե����ǿ������lcd�����ļ��ӿڣ�s3c_fb_probe�ȣ���

fbmem_init ��ʵ����һ���ַ��豸��������������class������û�������豸�ļ���

����ַ��豸������file_operations����ĺ�����ʵ���϶��Ǵ�struct fb_info *registered_fb[FB_MAX]   ���

fb_info�Ľṹ��������ȥ���� fb_ops ����ṹ���к���ָ�롣�����±�Ϊ���豸�š���ô����ṹ������θ�ֵ��

    �أ�

fbmem.c�ﶨ�� register_framebuffer�����������������ʾ�豸���ǵ��������������registered_fb������鸳ֵ��

Ȼ����ȥ�����豸�ļ���
*/

//��Ҫ���lcd��ƽ̨�豸��Ϣ
static void __init smdk4x12_machine_init(void)
{
	//...
	//��ʼ�����ƽ̨��Ϣ

	tiny4412_fb_init_pdata(&smdk4x12_lcd0_pdata);

	//��Ҫ���lcd��ƽ̨�豸��Ϣ

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
��common.c������� s5p_fb_setname(0,"exynos4-fb");
չ��ʵ��������s5p_device_fimd0.name = name;
��鱻��Ϊ�� "exynos4-fb"
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

//lcd ��ռ��io�ڼ��ն���Դ��

static struct resource s5p_fimd0_resource[] = {
[0] = DEFINE_RES_MEM(S5P_PA_FIMD0, SZ_32K),
[1] = DEFINE_RES_IRQ(IRQ_FIMD0_VSYNC),
[2] = DEFINE_RES_IRQ(IRQ_FIMD0_FIFO),
[3] = DEFINE_RES_IRQ(IRQ_FIMD0_SYSTEM),
};

//�ŵ�struct platform_device *smdk4x12_device�����ṹ���У����ŵ���platform_add_devices��lcdƽ̨�豸ע�ᵽ�ںˡ�
//�����Ϸ�����ʵ�ʶ����������豸��"s3c-fb","exynos4-fb"��
/*
���ڣ��ҵ��ˣ�ƽ̨���ߵ��豸������������Ҫ������Ҫ�������ȥ�޸�lcd�ĸ��ֲ�������Ҫ��fb_info�ṹ��

    ��fb_var_screeninfo�ṹ�壬�������¼��lcd����Ҫ9��������

   ��ǰ�磬�к�磬��ͬ���ź�����֡ǰ�磬֡��磬֡ͬ���ź���������ʱ��Ƶ�ʣ�x�����ص㣬y�����ص㡣
*/
/*-----------------------------------------------------------------------------------
samsung s3c-fb.c����
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
		.name		= "exynos4-fb",//������ʹ�����*
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
	/*��Ϊһ������Ҫ�ʺϺܶ�汾���豸��ÿ���汾���豸������ ������
	��һ��������Ҫ�õ�platid��ѡ���ĸ��汾���豸����s5pv210-fb��,
	 "s3c2443-fb"..., ��Щ����platid(Ҳ��s3c-fb.c�ж�����)�� Ҳ�����ˣ�����������ʺ�����Щ�豸��  */
	const struct platform_device_id *platid;
	struct s3c_fb_driverdata *fbdrv;//driver data 
	struct device *dev = &pdev->dev;
	struct s3c_fb_platdata *pd;	// platform data 
	/*һ������Ҫ�����ݽ�ݣ� ��������һ����ʾ����������ʾ�����������ж��������������ˡ�
	�������������һ���ֲ������ˡ�  */
	struct s3c_fb *sfb;
	struct s3c_fb_win *fbwin;
	struct resource *res;
	int win;
	int ret = 0;
	u32 reg;
	platid = platform_get_device_id(pdev);//��platform device �� ��id_entry �����л�ȡplatid����һ����ʵ��  
	fbdrv = (struct s3c_fb_driverdata *)platid->driver_data;//��ȡplatid��Ӧ��driver data��driver data��s3c-fb.c�ж��壬��Ҫ��һ�����ò���  

	if (fbdrv->variant.nr_windows > S3C_FB_MAX_WIN) {
		dev_err(dev, "too many windows, cannot attach\n");
		return -EINVAL;
	}

	pd = pdev->dev.platform_data;// ��ȡplatform data�����ڰ��ļ��ж��壬���data������˹�����ʾ�����������ݣ�Ҳ������win�����ݡ�  
	if (!pd) {
		dev_err(dev, "no platform data specified\n");
		return -EINVAL;
	}

	sfb = devm_kzalloc(dev, sizeof(struct s3c_fb), GFP_KERNEL);//���������Ҫ�Ľṹ�����ڴ�ռ�  
	if (!sfb) {
		dev_err(dev, "no memory for framebuffers\n");
		return -ENOMEM;
	}

	dev_dbg(dev, "allocate new framebuffer %p\n", sfb);

	sfb->dev = dev;//��sfb ���� ��ʾ�������� device �ṹ��  
	sfb->pdata = pd;// ��sfb ���� ��ʾ�������� platform data �ṹ��  
	sfb->variant = fbdrv->variant;// driver data�ṹ���� �� variant��Ա�� ����variant���Կ�����s3c_fb_variant�ṹ��

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

	sfb->bus_clk = clk_get(dev, "lcd");//��"lcd"������֣�ȥclock�ļ����ҵ��Լ���bus clock  
	if (IS_ERR(sfb->bus_clk)) {
		dev_err(dev, "failed to get bus clock\n");
		ret = PTR_ERR(sfb->bus_clk);
		goto err_sfb;
	}

	clk_enable(sfb->bus_clk);

	if (!sfb->variant.has_clksel) {
		sfb->lcd_clk = clk_get(dev, "sclk_fimd");//���driver data��û���� Դʱ�ӣ� ���á�sclk_fimd��������ȥclock�ļ��ҵ��Լ���Դʱ��
		if (IS_ERR(sfb->lcd_clk)) {
			dev_err(dev, "failed to get lcd clock\n");
			ret = PTR_ERR(sfb->lcd_clk);
			goto err_bus_clk;
		}

		clk_enable(sfb->lcd_clk);
	}

	pm_runtime_enable(sfb->dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);/*��ȡ��Դ��������ʼ��ַ���յ�ַ��
	��С�����͵ȣ�����res�ṹ�С�ʵ�����ǼĴ����ǵ��������յ�ַ��  */
	if (!res) {
		dev_err(dev, "failed to find registers\n");
		ret = -ENOENT;
		goto err_lcd_clk;
	}

	sfb->regs = devm_request_and_ioremap(dev, res);/*�����ڴ�, �ڴ�ӳ�䣬����
	�����ķ��ʵ�ַӳ�䵽�ղŷ�����ڴ��ϣ� sfb->regsΪ��ʼ��ַ��   */
	if (!sfb->regs) {
		dev_err(dev, "failed to map registers\n");
		ret = -ENXIO;
		goto err_lcd_clk;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);//��ȡʹ�õ��жϺŵĿ�ʼ��ַ�ͽ���ֵ
	if (!res) {
		dev_err(dev, "failed to acquire irq resource\n");
		ret = -ENOENT;
		goto err_lcd_clk;
	}
	sfb->irq_no = res->start;
	ret = devm_request_irq(dev, sfb->irq_no, s3c_fb_irq,
			  0, "s3c_fb", sfb);//ע���ж�
	if (ret) {
		dev_err(dev, "irq request failed\n");
		goto err_lcd_clk;
	}

	dev_dbg(dev, "got resources (regs %p), probing windows\n", sfb->regs);

	platform_set_drvdata(pdev, sfb);//��sfb ����pdev->dev->p->driverdata �ṹ����  
#if defined(CONFIG_FB_ION_EXYNOS)
	mutex_init(&sfb->vsync_info.irq_lock);

	ret = device_create_file(sfb->dev, &dev_attr_vsync);
	if (ret) {
		dev_err(sfb->dev, "failed to create vsync file\n");
	}
#endif

	/* setup gpio and output polarity controls */

	pd->setup_gpio();// ִ��setup_gpio�������˺�����setup_fimd0.c �ж����ˡ���������GPIO�˿ڸ�FIMDʹ�á�  

	writel(pd->vidcon1, sfb->regs + VIDCON1);//����VIDCON1 �Ĵ��� 

	/* set video clock running at under-run */
	if (sfb->variant.has_fixvclk) {
		reg = readl(sfb->regs + VIDCON1);
		reg &= ~VIDCON1_VCLK_MASK;
		reg |= VIDCON1_VCLK_RUN;
		writel(reg, sfb->regs + VIDCON1);
	}

	/* zero all windows before we do anything */

	for (win = 0; win < fbdrv->variant.nr_windows; win++)
		s3c_fb_clear_win(sfb, win);/*������window��wincon�Ĵ�����0��
		VIDOSDxA, VIDOSDxB, VIDOSDxC��0����ֹupdate����window��shadow  */

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
				       &sfb->windows[win]);//���� �� ע��framebuffer����Ҫ����
		if (ret < 0) {
			dev_err(dev, "failed to create window %d\n", win);
			for (; win >= 0; win--)
				s3c_fb_release_win(sfb, sfb->windows[win]);//ע�᲻�ɹ��Ļ����ͷ�֮ǰע��ɹ�����window  
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

	/*��ʾ����logo*/
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

	platform_set_drvdata(pdev, sfb);/*��һ�ν�sfb ����pdev->dev->p->driverdata �ṹ���У�
	֮ǰ��������������һ�Σ���������һ�Σ�����Ϊsfb������ݸ����˺ܶ�  */

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

/*����ʾ�������Ľṹ����Ϊ���������������мĴ�����ʼ��ַ�ȷḻ��Ϣ������win�ĺ���Ҳ��Ϊ����������  */
static int __devinit s3c_fb_probe_win(struct s3c_fb *sfb, unsigned int win_no,
		struct s3c_fb_win_variant *variant,
		struct s3c_fb_win **res)
		//driver data��win�Ĳ���, ������fbdrv->win[win]�������������ġ�
		//per window private data for each framebuffer�������溬��ָ��FBI(fb_info)�ṹ���ָ��
{
	struct fb_var_screeninfo *var;
	struct fb_videomode initmode;
	struct s3c_fb_pd_win *windata;//per window setup data,  Ҳ����platform data��win�Ĳ���  
	struct s3c_fb_win *win;
	struct fb_info *fbinfo;
	int palette_size;
	int ret;

	dev_dbg(sfb->dev, "probing window %d, variant %p\n", win_no, variant);

	palette_size = variant->palette_sz * 4;//���õ�ɫ���С 

	/* ����fb_info�ṹ�壬����һ��fb_info�ṹ���ַ������ṹ������ûʲô���ݣ�
	ֻ��ֵ��par(win����ʼ��ַ)��device (���豸)����������*/
	fbinfo = framebuffer_alloc(sizeof(struct s3c_fb_win) +
				   palette_size * sizeof(u32), sfb->dev);
	if (!fbinfo) {
		dev_err(sfb->dev, "failed to allocate framebuffer\n");
		return -ENOENT;
	}

	windata = sfb->pdata->win[win_no];//windataָ�� platform data��win�Ĳ���   
	initmode = *sfb->pdata->vtiming;

	WARN_ON(windata->max_bpp == 0);
	WARN_ON(windata->xres == 0);
	WARN_ON(windata->yres == 0);

	win = fbinfo->par;
	*res = win;//par����win����ʼ��ַ�����ڰ���ʼ��ַ��*res����ô*res����ָ��s3c_fb_win��ָ��   
	var = &fbinfo->var;//����fbinfo->var���ǿյģ� ֻ�ǽ���ַ��var���� 
	win->variant = *variant;//��win�Ĳ������win->variant��   
	win->fbinfo = fbinfo;//-��win->fbinfoָ�����FBI�ṹʵ��   
	win->parent = sfb;//win��parent����ʾ��������������ָ��sfb�ṹ��>   
	win->windata = windata;//>��win->windataָ�� platform data��win�Ĳ���
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
	fb_videomode_to_var(&fbinfo->var, &initmode);//��FBI���ϸ����������˺������appendix  

	fbinfo->var.width	= windata->width;
	fbinfo->var.height	= windata->height;
	fbinfo->fix.type	= FB_TYPE_PACKED_PIXELS;
	fbinfo->fix.accel	= FB_ACCEL_NONE;
	fbinfo->var.activate	= FB_ACTIVATE_NOW;
	fbinfo->var.vmode	= FB_VMODE_NONINTERLACED;
	fbinfo->var.bits_per_pixel = windata->default_bpp;
	fbinfo->fbops		= &s3c_fb_ops;//��framebuffer�Ĳ���
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
	.fb_check_var	= s3c_fb_check_var,//���ɱ��������������֧�ֵ�ֵ*/ 
	.fb_set_par		= s3c_fb_set_par,//����info->var����videoģʽ*/
	.fb_blank		= s3c_fb_blank,//��ʾ�հ�*/  
	.fb_setcolreg	= s3c_fb_setcolreg,//����color�Ĵ���*/  
	.fb_fillrect	= cfb_fillrect,//�������drivers/video/cfblillrect.c��ʵ�֣���FB_CIRRUS��*/
	.fb_copyarea	= cfb_copyarea,//���ݸ���drivers/video/cfbcopyarea.c*/  
	.fb_imageblit	= cfb_imageblit,//ͼ�����drivers/video/cfbimgblt.c*/  
	.fb_pan_display	= s3c_fb_pan_display,
	.fb_ioctl		= s3c_fb_ioctl,//**
#if defined(CONFIG_FB_ION_EXYNOS)
	.fb_mmap		= s3c_fb_mmap,
#endif
};



static int __devinit s3c_fb_alloc_memory(struct s3c_fb *sfb,
		struct s3c_fb_win *win)
{
	struct s3c_fb_pd_win *windata = win->windata;//platform data��win�Ĳ��� 
	unsigned int real_size, virt_size, size;
	struct fb_info *fbi = win->fbinfo;// ��fbiָ��FBI�ṹ��  
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

	fbi->fix.smem_len = size;//Ҫ������ڴ��С
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
			&map_dma, GFP_KERNEL);//����framebuffer���ڴ�
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








