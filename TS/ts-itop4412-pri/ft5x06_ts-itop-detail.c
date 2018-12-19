/*
	FT5x06系列ICs是单芯片电容式触摸屏控制器IC，带有一个内置的8位微控制器单元（MCU）。
采用互电容的方法，在配合的相互的电容式触摸面板，它支持真正的多点触摸功能。FT5x06具有用户友好
的输入的功能，这可以应用在许多便携式设备，例如蜂窝式电话，移动互联网设备，上网本和笔记本个人
电脑。FT5x06系列IC包括FT5206/FT5306/FT5406。
	从FT5X06的datasheet中，我们可以看到，FT5X06既可以工作的SPI的接口方式，也可以工作在I2C的接口
方式，不管工作在SPI，还是工作在I2C，从硬件的接口设计上来说，这下面的几个控制口，都是需要要接的。
	1）：INT引脚，这个脚是一个中端信号，它用来通知HOST端FT5X06已经准备好，可以进行读操作了。
	2）：WAKE引脚：这个功能主要的作用是将FT5X06从睡眠状态转换到工作状态。
	3）：/RST引脚：FT5X06的芯片复位信号。
	根据FT5406数据手册上的指令，我们先了解下驱动如何实现电容屏的多点触摸，其实很简单，主要需要触
摸屏IC FT5406 能够捕获多点数据，这点电容屏基本多能支持到捕获2点以上，而FT5406 可以捕获5个触摸点,
编写驱动时，只要去获取这几个点的数据，然后上报就可以了。
	
*/
/*
  I2C的驱动需要根据具体的ARM芯片，一般来说，IC原厂，一般会将在linux的bsp中都会有I2C的驱动，这
个部分不需要我们去写的，我们只需要将FT5X06和BSP包中的I2C驱动匹配起来就好了。
*/
//需要添加i2c的平台设备信息
#ifdef CONFIG_TOUCHSCREEN_FT5X0X
#include <plat/ft5x0x_touch.h>
static struct ft5x0x_i2c_platform_data ft5x0x_pdata = {
        .gpio_irq               = EXYNOS4_GPX0(4),
        .irq_cfg                = S3C_GPIO_SFN(0xf),
        .screen_max_x   = 768,
        .screen_max_y   = 1024,
        .pressure_max   = 255,
};

static struct i2c_board_info i2c_devs3[] __initdata = {
	/* support for FT5X0X TouchScreen */
#if defined(CONFIG_TOUCHSCREEN_FT5X0X)
	{
		I2C_BOARD_INFO("ft5x0x_ts", 0x70>>1),
		.irq = IRQ_EINT(4),
		.platform_data = &ft5x0x_pdata,
	},
#endif
	/* end add */
};

static void __init smdk4x12_machine_init(void)
{
	s3c_i2c3_set_platdata(NULL);
	i2c_register_board_info(3, i2c_devs3, ARRAY_SIZE(i2c_devs3));
}






