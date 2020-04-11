static struct platform_device tiny4412_device_adc = {
	.name			= "tiny4412_adc",
	.id				= -1,
	.num_resources	= 0,
};

static struct platform_device *smdk4x12_devices[] __initdata = {
	&tiny4412_device_adc,
}

static void __init smdk4x12_machine_init(void)
{
		platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));

}

MACHINE_START(TINY4412, "TINY4412")
	/* Maintainer: FriendlyARM (www.arm9.net) */
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	/* Maintainer: Changhwan Youn <chaos.youn@samsung.com> */
	.atag_offset	= 0x100,
	.init_irq	= exynos4_init_irq,
	.map_io		= smdk4x12_map_io,
	.handle_irq	= gic_handle_irq,
	.init_machine	= smdk4x12_machine_init,
	.init_late	= exynos_init_late,
	.timer		= &exynos4_timer,
	.restart	= exynos4_restart,
	.reserve	= &smdk4x12_reserve,
MACHINE_END