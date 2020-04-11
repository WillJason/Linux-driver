#ifdef CONFIG_LEDS_CTL
struct platform_device s3c_device_leds_ctl = {
        .name   = "leds",
        .id             = -1,
};
#endif

static struct platform_device *smdk4x12_devices[] __initdata = {

	#ifdef CONFIG_LEDS_CTL
		&s3c_device_leds_ctl,
	#endif
};

static void __init smdk4x12_machine_init(void)
{
	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));
}

MACHINE_START(SMDK4412, "SMDK4X12")
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= exynos4_init_irq,
	.map_io		= smdk4x12_map_io,
	.init_machine	= smdk4x12_machine_init,
	.timer		= &exynos4_timer,

	#if defined(CONFIG_KERNEL_PANIC_DUMP)		//mj for panic-dump
	.reserve		= reserve_panic_dump_area,
	#endif

#ifdef CONFIG_EXYNOS_C2C
	.reserve	= &exynos_c2c_reserve,
#endif
MACHINE_END
#endif