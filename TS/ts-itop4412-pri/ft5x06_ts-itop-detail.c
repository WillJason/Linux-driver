/*
	FT5x06ϵ��ICs�ǵ�оƬ����ʽ������������IC������һ�����õ�8λ΢��������Ԫ��MCU����
���û����ݵķ���������ϵ��໥�ĵ���ʽ������壬��֧�������Ķ�㴥�����ܡ�FT5x06�����û��Ѻ�
������Ĺ��ܣ������Ӧ��������Яʽ�豸���������ʽ�绰���ƶ��������豸���������ͱʼǱ�����
���ԡ�FT5x06ϵ��IC����FT5206/FT5306/FT5406��
	��FT5X06��datasheet�У����ǿ��Կ�����FT5X06�ȿ��Թ�����SPI�Ľӿڷ�ʽ��Ҳ���Թ�����I2C�Ľӿ�
��ʽ�����ܹ�����SPI�����ǹ�����I2C����Ӳ���Ľӿ��������˵��������ļ������ƿڣ�������ҪҪ�ӵġ�
	1����INT���ţ��������һ���ж��źţ�������֪ͨHOST��FT5X06�Ѿ�׼���ã����Խ��ж������ˡ�
	2����WAKE���ţ����������Ҫ�������ǽ�FT5X06��˯��״̬ת��������״̬��
	3����/RST���ţ�FT5X06��оƬ��λ�źš�
	����FT5406�����ֲ��ϵ�ָ��������˽����������ʵ�ֵ������Ķ�㴥������ʵ�ܼ򵥣���Ҫ��Ҫ��
����IC FT5406 �ܹ����������ݣ�����������������֧�ֵ�����2�����ϣ���FT5406 ���Բ���5��������,
��д����ʱ��ֻҪȥ��ȡ�⼸��������ݣ�Ȼ���ϱ��Ϳ����ˡ�
	
*/
/*
  I2C��������Ҫ���ݾ����ARMоƬ��һ����˵��ICԭ����һ��Ὣ��linux��bsp�ж�����I2C����������
�����ֲ���Ҫ����ȥд�ģ�����ֻ��Ҫ��FT5X06��BSP���е�I2C����ƥ�������ͺ��ˡ�
*/
//��Ҫ���i2c��ƽ̨�豸��Ϣ
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






