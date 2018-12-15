/*
	linux�ı���(backlight)��ϵͳ������/sysĿ¼���ṩ�û��ռ����LCD����������ʾ�豸��
�������ȵĽӿڡ���������Ȳ���������������״̬�����кܶ���ȼ������ȣ������û��ռ��
�ݽ��ܡ����ӷ�Χ��������ڱ�������ȡ�
	Linux�ں�����һ��backlight������ϵͳ����ϵͳ����Ϊ�����û�����������Ƶģ��û�ֻҪ
�����Լ���LCD�����·��PWM������ţ����ں�backlight��ϵͳ���������Ӧ�����ã��Ϳ���
ʵ��LCD�ı��⡣
	LCD�ı���ԭ����Ҫ���ɺ��İ��һ�����ſ��Ʊ����Դ��һ��PWM���ſ��Ʊ���������ɣ�
Ӧ�ó������ͨ���ı�PWM��Ƶ�ʴﵽ�ı䱳�����ȵ�Ŀ�ġ�
	����������backlight��ϵͳ�ǻ���pwm���ĵ�һ�������ӿڣ������ʹ�õ�һ���豸Ҳ�ǻ���
pwm�ģ�������Ҫ�û����Ե���pwm��Ƶ���Դﵽ����ı䱳�����ȣ��ı������Ƶ�ʵ�Ч������
ô�����ʹ�����backlight������ϵͳ��

	�������ǽ���backlight��ϵͳ��������ϵͳĿ¼��/driver/video/backlight�£����б�����
ϵͳ���Ĵ�����backlight.c

*/

/*
�Ȳ鿴/driver/video/backlight/Makefile
obj-$(CONFIG_BACKLIGHT_CLASS_DEVICE) += backlight.o

�����鿴/driver/video/backlight/Kconfig
config BACKLIGHT_CLASS_DEVICE
tristate "Lowlevel Backlight controls"
depends on BACKLIGHT_LCD_SUPPORT
default m
*/
//���濴backlight����ĺ��Ĵ���backlight.c
static int __init backlight_class_init(void)
{
	backlight_class = class_create(THIS_MODULE, "backlight");//ע��backlight��
	if (IS_ERR(backlight_class)) {
		printk(KERN_WARNING "Unable to create backlight class; errno = %ld\n",
				PTR_ERR(backlight_class));
		return PTR_ERR(backlight_class);
	}

	backlight_class->dev_attrs = bl_device_attributes;//���������
	backlight_class->suspend = backlight_suspend;
	backlight_class->resume = backlight_resume;
	return 0;
}
/*
	����֪��backlight������ϵͳ����Ҫ���ǿ���������ԣ����������ñ���ֵ��������������
ĳ����Աд����ֵ����������Ծ��Ǹ��û���һ�ֽӿڣ������ص㿴��
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
	�����ԣ���backlight�������Ǵ�����bl_power��brightness��actural_brightness��max_brightness
�ĸ���Ա������brightness�ǵ�ǰ���ȣ�max_brightness��������ȡ����û���ͨ��cat����echo����
�ͻᴥ����Щ��Ա��������Щ���ԵĶ�д�����������ȿ������ĺ���backlight_show_max_brightness��

*/
static ssize_t backlight_show_max_brightness(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	return sprintf(buf, "%d\n", bd->props.max_brightness);//����������
}
//�ٿ���������������ṹ��

struct backlight_ops {

unsigned int options;

#define BL_CORE_SUSPENDRESUME?????? (1 << 0)

int (*update_status)(struct backlight_device *);?? //�ı䱳��״̬

int (*get_brightness)(struct backlight_device *);? //��ȡ����ֵ

int (*check_fb)(struct fb_info *);

};
//���Ǽ�����backlight��������д�ĺ������������õ�ǰ����ֵ����backlight_store_brightness��
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
			bd->props.brightness = brightness;//���뱳��ֵ
			backlight_update_status(bd);//����backlight_update_status�豸����ֵ
			rc = count;
		}
	}
	mutex_unlock(&bd->ops_lock);

	backlight_generate_event(bd, BACKLIGHT_UPDATE_SYSFS);

	return rc;
}

EXPORT_SYMBOL(backlight_device_register);? //ע�ᱳ���豸

EXPORT_SYMBOL(backlight_device_unregister); //ע�������豸

//��Щ�ӿںܼ򵥣��Ͳ�ϸ˵�ˣ��������ǵ�backlight��ϵͳ�ĺ��Ĳ�ͽ�������,��ֻ��һ����ϵͳ�ĺ�
//���ļ����������沢û��ע��ʲô�豸������ע��һ���ಢ�ṩ��һ���ӿڡ�




