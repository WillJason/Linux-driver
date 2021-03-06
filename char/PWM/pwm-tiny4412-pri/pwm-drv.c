#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>

#include <linux/gpio.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>


#define DEVICE_NAME				"pwm"

#define PWM_IOCTL_SET_FREQ		1
#define PWM_IOCTL_STOP			0

#define NS_IN_1HZ				(1000000000UL)


#define BUZZER_PWM_ID			0
#define BUZZER_PMW_GPIO			EXYNOS4_GPD0(0)


static struct pwm_device *pwm4buzzer;
static struct semaphore lock;

static void pwm_set_freq(unsigned long freq) {
	int period_ns = NS_IN_1HZ / freq;

	//Pwm_config函数有三个参数，第一个是当前设置的pwm设备，第二个和第三个分别是占空比与周期。 
	pwm_config(pwm4buzzer, period_ns / 2, period_ns);
	pwm_enable(pwm4buzzer);

	s3c_gpio_cfgpin(BUZZER_PMW_GPIO, S3C_GPIO_SFN(2));//复用功能选择
}


static void pwm_stop(void) {
	s3c_gpio_cfgpin(BUZZER_PMW_GPIO, S3C_GPIO_OUTPUT);

	pwm_config(pwm4buzzer, 0, NS_IN_1HZ / 100);//pwm_config - change a PWM device configuration 
	pwm_disable(pwm4buzzer);//不使能PWM
}


static int tiny4412_pwm_open(struct inode *inode, struct file *file) {
	if (!down_trylock(&lock))//获取锁
		return 0;
	else
		return -EBUSY;
}

static int tiny4412_pwm_close(struct inode *inode, struct file *file) {
	up(&lock);//释放锁
	return 0;
}

static long tiny4412_pwm_ioctl(struct file *filep, unsigned int cmd,
		unsigned long arg)
{
	switch (cmd) {
		case PWM_IOCTL_SET_FREQ:
			if (arg == 0)
				return -EINVAL;
			pwm_set_freq(arg);
			break;

		case PWM_IOCTL_STOP:
		default:
			pwm_stop();
			break;
	}

	return 0;
}


static struct file_operations tiny4412_pwm_ops = {
	.owner			= THIS_MODULE,
	.open			= tiny4412_pwm_open,
	.release		= tiny4412_pwm_close, 
	.unlocked_ioctl	= tiny4412_pwm_ioctl,
};

static struct miscdevice tiny4412_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &tiny4412_pwm_ops,
};

static int __init tiny4412_pwm_dev_init(void)
{
	int ret;

	ret = gpio_request(BUZZER_PMW_GPIO, DEVICE_NAME);
	if (ret) {
		printk("request GPIO %d for pwm failed\n", BUZZER_PMW_GPIO);
		return ret;
	}
	
	gpio_set_value(BUZZER_PMW_GPIO, 0);//设置初始值
	s3c_gpio_cfgpin(BUZZER_PMW_GPIO, S3C_GPIO_OUTPUT);
	
	pwm4buzzer = pwm_request(BUZZER_PWM_ID, DEVICE_NAME);//pwm_request - request a PWM device 
	if (IS_ERR(pwm4buzzer)) {
		printk("request pwm %d for %s failed\n", BUZZER_PWM_ID, DEVICE_NAME);
		return -ENODEV;
	}
	
	pwm_stop();
	
	sema_init(&lock, 1);//初始化一个定位在 sem 的匿名信号量。value 参数指定信号量的初始值。初始化一个互斥锁，但它把信号量sem的值设置为0，即一开始就处在已锁状态。
	ret = misc_register(&tiny4412_misc_dev);

	printk(DEVICE_NAME "\tinitialized\n");

	return ret;
}

static void __exit tiny4412_pwm_dev_exit(void)
{
	pwm_stop();

	misc_deregister(&tiny4412_misc_dev);
	gpio_free(BUZZER_PMW_GPIO);
}


module_init(tiny4412_pwm_dev_init);
module_exit(tiny4412_pwm_dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("FriendlyARM Inc.");
MODULE_DESCRIPTION("Exynos4 PWM Driver");