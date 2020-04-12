/*
Ӳ��ƽ̨��s3c2440
ϵͳ��linux-2.6.22.6
	��Ƶ���������������Ƶ�ź���ת��Ϊ���������������ģ�������źţ���ͨ����˷�¼��ʱ��
ִ���෴�Ĺ��̡����������������������ӵ���Ƶ�����������ͷ��ʽ���󡢶�������Ͳ����Ƶ
��������ߡ��������Ҳ�ṩ��������mixer�����ܣ�����������Щ��Ƶ����������ϣ�������
�й��źŵ�������

��һ��Ӳ��    IIS�ӿ�--����    ����GPIO L3  ����

mini2440��TQ2440ѡ������Ϊuda1341�����������������������
����λ��sound\soc\s3c24xx\Ŀ¼��uda1341��������Ϊ/dev/dsp: ���ڲ���/¼����/dev/mixer: ����������
�����ļ�Ϊsound\soc\s3c24xx\s3c2410-uda1341.c��
*/
////��Ҫ���uda1341��ƽ̨�豸��Ϣ
static void __init smdk2440_machine_init(void)
{
	platform_add_devices(smdk2440_devices, ARRAY_SIZE(smdk2440_devices));
}

static struct platform_device *smdk2440_devices[] __initdata = {
	&s3c_device_iis,
};
/* IIS */

static struct resource s3c_iis_resource[] = {
	[0] = {
		.start = S3C24XX_PA_IIS,
		.end   = S3C24XX_PA_IIS + S3C24XX_SZ_IIS -1,
		.flags = IORESOURCE_MEM,
	}
};

static u64 s3c_device_iis_dmamask = 0xffffffffUL;

struct platform_device s3c_device_iis = {
	.name		  = "s3c2410-iis",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_iis_resource),
	.resource	  = s3c_iis_resource,
	.dev              = {
		.dma_mask = &s3c_device_iis_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};
//�ں�����
-> Device Drivers
  -> Sound
    -> Advanced Linux Sound Architecture
      -> Advanced Linux Sound Architecture
        -> System on Chip audio support
        <*> I2S of the Samsung S3C24XX chips
/*-----------------------------------------------------------------------------------
samsung sound\soc\s3c24xx\s3c2410-uda1341.c����
-----------------------------------------------------------------------------------*/
static struct device_driver s3c2410iis_driver = {
	.name = "s3c2410-iis",
	.bus = &platform_bus_type,
	.probe = s3c2410iis_probe,
	.remove = s3c2410iis_remove,
};

static int __init s3c2410_uda1341_init(void) {
	return driver_register(&s3c2410iis_driver);
}   
  
static int s3c2410iis_probe(struct device *dev)       
{
	struct platform_device *pdev = to_platform_device(dev);
	struct resource *res;
	unsigned long flags;

	printk ("s3c2410iis_probe...\n");
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk(KERN_INFO PFX "failed to get memory region resouce\n");
		return -ENOENT;
	}

	iis_base = (void *)S3C24XX_VA_IIS ;
	if (iis_base == 0) {
		printk(KERN_INFO PFX "failed to ioremap() region\n");
		return -EINVAL;
	}

	iis_clock = clk_get(dev, "iis");
	if (iis_clock == NULL) {
		printk(KERN_INFO PFX "failed to find clock source\n");
		return -ENOENT;
	}

	clk_enable(iis_clock);/* ʹ��ʱ�� */

	local_irq_save(flags);
	
	 /* ����GPIO */
	/* GPB 4: L3CLOCK, OUTPUT */
	s3c2410_gpio_cfgpin(S3C2410_GPB4, S3C2410_GPB4_OUTP);
	s3c2410_gpio_pullup(S3C2410_GPB4,1);
	/* GPB 3: L3DATA, OUTPUT */
	s3c2410_gpio_cfgpin(S3C2410_GPB3,S3C2410_GPB3_OUTP);
	/* GPB 2: L3MODE, OUTPUT */
	s3c2410_gpio_cfgpin(S3C2410_GPB2,S3C2410_GPB2_OUTP);
	s3c2410_gpio_pullup(S3C2410_GPB2,1);
	/* GPE 3: I2SSDI */
	s3c2410_gpio_cfgpin(S3C2410_GPE3,S3C2410_GPE3_I2SSDI);
	s3c2410_gpio_pullup(S3C2410_GPE3,0);
	/* GPE 0: I2SLRCK */
	s3c2410_gpio_cfgpin(S3C2410_GPE0,S3C2410_GPE0_I2SLRCK);
	s3c2410_gpio_pullup(S3C2410_GPE0,0);
	/* GPE 1: I2SSCLK */
	s3c2410_gpio_cfgpin(S3C2410_GPE1,S3C2410_GPE1_I2SSCLK);
	s3c2410_gpio_pullup(S3C2410_GPE1,0);
	/* GPE 2: CDCLK */
	s3c2410_gpio_cfgpin(S3C2410_GPE2,S3C2410_GPE2_CDCLK);
	s3c2410_gpio_pullup(S3C2410_GPE2,0);
	/* GPE 4: I2SSDO */
	s3c2410_gpio_cfgpin(S3C2410_GPE4,S3C2410_GPE4_I2SSDO);
	s3c2410_gpio_pullup(S3C2410_GPE4,0);

	local_irq_restore(flags);

	init_s3c2410_iis_bus();/* ����S3C2440��IIS������ */

	init_uda1341();/* ʹ��L3�ӿڳ�ʼ��uda1341оƬ */

	/* ��������DMAͨ��:һ�����ڲ���,��һ������¼�� */
	output_stream.dma_ch = DMACH_I2S_OUT;
	if (audio_init_dma(&output_stream, "UDA1341 out")) {
		audio_clear_dma(&output_stream,&s3c2410iis_dma_out);
		printk( KERN_WARNING AUDIO_NAME_VERBOSE
				": unable to get DMA channels\n" );
		return -EBUSY;
	}
    
	input_stream.dma_ch = DMACH_I2S_IN;
	if (audio_init_dma(&input_stream, "UDA1341 in")) {
		audio_clear_dma(&input_stream,&s3c2410iis_dma_in);
		printk( KERN_WARNING AUDIO_NAME_VERBOSE
				": unable to get DMA channels\n" );
		return -EBUSY;
	}

	audio_dev_dsp = register_sound_dsp(&smdk2410_audio_fops, -1);
	audio_dev_mixer = register_sound_mixer(&smdk2410_mixer_fops, -1);

	printk(AUDIO_NAME_VERBOSE " initialized\n"); 

	return 0;
}
//˳�㿴һ��dma���ݴ��亯��audio_init_dma
static int __init audio_init_dma(audio_stream_t * s, char *desc)
{
	int ret ;
	enum s3c2410_dmasrc source;
	int hwcfg;
	unsigned long devaddr;
	dmach_t channel;
	int dcon;
	unsigned int flags = 0;

	if(s->dma_ch == DMACH_I2S_OUT){
		channel = DMACH_I2S_OUT;
		source  = S3C2410_DMASRC_MEM;//���Դ���ڴ�
		hwcfg   = BUF_ON_APB;
		devaddr = 0x55000010;
		dcon    = S3C2410_DCON_HANDSHAKE|S3C2410_DCON_SYNC_PCLK|S3C2410_DCON_INTREQ|S3C2410_DCON_TSZUNIT|S3C2410_DCON_SSERVE|S3C2410_DCON_CH2_I2SSDO|S3C2410_DCON_HWTRIG; // VAL: 0xa0800000;
		flags   = S3C2410_DMAF_AUTOSTART;

		ret = s3c2410_dma_request(s->dma_ch, &s3c2410iis_dma_out, NULL);
		s3c2410_dma_devconfig(channel, source, hwcfg, devaddr);
		s3c2410_dma_config(channel, 2, dcon);
		s3c2410_dma_set_buffdone_fn(channel, audio_dmaout_done_callback);
		s3c2410_dma_setflags(channel, flags);
        s->dma_ok = 1;
		return ret;
	}
	else if(s->dma_ch == DMACH_I2S_IN){
		channel = DMACH_I2S_IN;
		source  = S3C2410_DMASRC_HW;//����Դ������
		hwcfg   = BUF_ON_APB;
		devaddr = 0x55000010;
		dcon    = S3C2410_DCON_HANDSHAKE|S3C2410_DCON_SYNC_PCLK|S3C2410_DCON_INTREQ|S3C2410_DCON_TSZUNIT|S3C2410_DCON_SSERVE|S3C2410_DCON_CH1_I2SSDI|S3C2410_DCON_HWTRIG; // VAL: 0xa2800000;
		flags   = S3C2410_DMAF_AUTOSTART;

		ret = s3c2410_dma_request(s->dma_ch, &s3c2410iis_dma_in, NULL);
		s3c2410_dma_devconfig(channel, source, hwcfg, devaddr);
		s3c2410_dma_config(channel, 2, dcon);
		s3c2410_dma_set_buffdone_fn(channel, audio_dmain_done_callback);
		s3c2410_dma_setflags(channel, flags);
		s->dma_ok =1;
		return ret ;
	}
	else
		return 1;
}
//test
//play:cat test.wav > /dev/dsp
//record:cat /dev/dsp > sound.bin
//			 cat sound.bin > /dev/dsp   


//jz2440����Ϊwm8976��������IIS��uda1341һ�����������ǿ����޸�uda1341��ʹ��
//jz2440û��L3�ӿڣ����ǿ���GPIOģ��һ��



































