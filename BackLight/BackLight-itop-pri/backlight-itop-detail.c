/*
	linux的背光(backlight)子系统用于在/sys目录下提供用户空间控制LCD或者其他显示设备的
背光亮度的接口。这里的亮度并不是亮和灭两个状态，可有很多个等级的亮度，便于用户空间根
据节能、可视范围等需求调节背光的亮度。
	Linux内核中有一个backlight背光子系统，该系统就是为满足用户这种需求设计的，用户只要
根据自己的LCD背光电路中PWM输出引脚，对内核backlight子系统代码进行相应的配置，就可以
实现LCD的背光。
	LCD的背光原理主要是由核心板的一根引脚控制背光电源，一根PWM引脚控制背光亮度组成，
应用程序可以通过改变PWM的频率达到改变背光亮度的目的。
	综上所述，backlight子系统是基于pwm核心的一种驱动接口，如果你使用的一种设备也是基于
pwm的，并且需要用户可以调节pwm的频率以达到诸如改变背光亮度，改变蜂鸣器频率的效果，那
么你可以使用这个backlight背光子系统。

	下面我们讲讲backlight子系统。背光子系统目录在/driver/video/backlight下，其中背光子
系统核心代码是backlight.c

*/

/*
先查看/driver/video/backlight/Makefile
obj-$(CONFIG_BACKLIGHT_CLASS_DEVICE) += backlight.o

继续查看/driver/video/backlight/Kconfig
config BACKLIGHT_CLASS_DEVICE
tristate "Lowlevel Backlight controls"
depends on BACKLIGHT_LCD_SUPPORT
default m
*/
//下面看backlight背光的核心代码backlight.c
static int __init backlight_class_init(void)
{
	backlight_class = class_create(THIS_MODULE, "backlight");//注册backlight类
	if (IS_ERR(backlight_class)) {
		printk(KERN_WARNING "Unable to create backlight class; errno = %ld\n",
				PTR_ERR(backlight_class));
		return PTR_ERR(backlight_class);
	}

	backlight_class->dev_attrs = bl_device_attributes;//添加类属性
	backlight_class->suspend = backlight_suspend;
	backlight_class->resume = backlight_resume;
	return 0;
}
/*
	我们知道backlight背光子系统的主要就是靠这个类属性，当我们设置背光值就是向类属性中
某个成员写背光值，这个类属性就是给用户的一种接口，我们重点看看
*/
static struct device_attribute bl_device_attributes[] = {
	__ATTR(bl_power, 0644, backlight_show_power, backlight_store_power),
	__ATTR(brightness, 0644, backlight_show_brightness,
		     backlight_store_brightness),
	__ATTR(actual_brightness, 0444, backlight_show_actual_brightness,
		     NULL),
	__ATTR(max_brightness, 0444, backlight_show_max_brightness, NULL),
	__ATTR(type, 0444, backlight_show_type, NULL),
	__ATTR_NULL,
};
/*
	很明显，在backlight类中我们创建了bl_power，brightness，actural_brightness，max_brightness
四个成员，其中brightness是当前亮度，max_brightness是最大亮度。当用户层通过cat或者echo命令
就会触发这些成员。对于这些属性的读写函数，我们先看看读的函数backlight_show_max_brightness吧

*/
static ssize_t backlight_show_max_brightness(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	return sprintf(buf, "%d\n", bd->props.max_brightness);//输出最大亮度
}
//再看看背光操作函数结构体

struct backlight_ops {

unsigned int options;

#define BL_CORE_SUSPENDRESUME?????? (1 << 0)

int (*update_status)(struct backlight_device *);?? //改变背光状态

int (*get_brightness)(struct backlight_device *);? //获取背光值

int (*check_fb)(struct fb_info *);

};
//我们继续看backlight类属性中写的函数，例如设置当前背光值函数backlight_store_brightness吧
static ssize_t backlight_store_brightness(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct backlight_device *bd = to_backlight_device(dev);
	unsigned long brightness;

	rc = strict_strtoul(buf, 0, &brightness);
	if (rc)
		return rc;

	rc = -ENXIO;

	mutex_lock(&bd->ops_lock);
	if (bd->ops) {
		if (brightness > bd->props.max_brightness)
			rc = -EINVAL;
		else {
			pr_debug("backlight: set brightness to %lu\n",
				 brightness);
			bd->props.brightness = brightness;//传入背光值
			backlight_update_status(bd);//调用backlight_update_status设备背光值
			rc = count;
		}
	}
	mutex_unlock(&bd->ops_lock);

	backlight_generate_event(bd, BACKLIGHT_UPDATE_SYSFS);

	return rc;
}

EXPORT_SYMBOL(backlight_device_register);? //注册背光设备

EXPORT_SYMBOL(backlight_device_unregister); //注销背光设备

//这些接口很简单，就不细说了，这样我们的backlight子系统的核心层就介绍完了,这只是一个子系统的核
//心文件，在这里面并没有注册什么设备，仅仅注册一个类并提供了一个接口。




