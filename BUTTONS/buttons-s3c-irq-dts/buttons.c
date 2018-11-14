#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
//#include <asm/arch/regs-gpio.h>
//#include <asm/hardware.h>
#include <linux/device.h>
//#include <mach/gpio.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <mach/regs-gpio.h>
#include <mach/gpio-samsung.h>
#include <plat/gpio-cfg.h>

#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

static struct class *sixthdrv_class;
static struct device	*sixthdrv_class_dev;

volatile unsigned long *gpfcon;
volatile unsigned long *gpfdat;

volatile unsigned long *gpgcon;
volatile unsigned long *gpgdat;

static struct timer_list buttons_timer;


static DECLARE_WAIT_QUEUE_HEAD(button_waitq);

/* �ж��¼���־, �жϷ����������1��sixth_drv_read������0 */
static volatile int ev_press = 0;

static struct fasync_struct *button_async;


struct pin_desc{
	unsigned int pin;
	unsigned int key_val;
	int irq;
};


/* ��ֵ: ����ʱ, 0x01, 0x02, 0x03, 0x04 */
/* ��ֵ: �ɿ�ʱ, 0x81, 0x82, 0x83, 0x84 */
static unsigned char key_val;

struct pin_desc pins_desc[4] = {
	{S3C2410_GPF(0), 0x01},
	{S3C2410_GPF(2), 0x02},
	{S3C2410_GPG(3), 0x03},
	{S3C2410_GPG(11), 0x04},
};

static struct pin_desc *irq_pd;

//static atomic_t canopen = ATOMIC_INIT(1);     //����ԭ�ӱ�������ʼ��Ϊ1

//static DECLARE_MUTEX(button_lock);     //���廥����
struct semaphore button_lock;


/*
  * ȷ������ֵ
  */
static irqreturn_t buttons_irq(int irq, void *dev_id)
{
	/* 10ms��������ʱ�� */
	irq_pd = (struct pin_desc *)dev_id;
	mod_timer(&buttons_timer, jiffies+HZ/100);
	return IRQ_RETVAL(IRQ_HANDLED);
}

static int sixth_drv_open(struct inode *inode, struct file *file)
{
	int ret;
	
#if 0	
	if (!atomic_dec_and_test(&canopen))
	{
		atomic_inc(&canopen);
		return -EBUSY;
	}
#endif		

	if (file->f_flags & O_NONBLOCK)
	{
		if (down_trylock(&button_lock))
			return -EBUSY;
	}
	else
	{
		/* ��ȡ�ź��� */
		down(&button_lock);
	}

	/* ����GPF0,2Ϊ�������� */
	/* ����GPG3,11Ϊ�������� */
	ret = request_irq(pins_desc[0].irq,  buttons_irq, 0, "S2", &pins_desc[0]);
	if (ret) {
		printk("reqeust_irq %d for EINT0 err : %d!\n", pins_desc[0].irq, ret);
		//return ret;
	}
	
	ret = request_irq(pins_desc[1].irq,  buttons_irq, 0, "S3", &pins_desc[1]);
	if (ret) {
		printk("reqeust_irq for EINT2 err : %d!\n", ret);
		//return ret;
	}

	ret = request_irq(pins_desc[2].irq, buttons_irq, 0, "S4", &pins_desc[2]);
	if (ret) {
		printk("reqeust_irq for EINT11 err : %d!\n", ret);
		//return ret;
	}

	ret = request_irq(pins_desc[3].irq, buttons_irq, 0, "S5", &pins_desc[3]);	
	if (ret) {
		printk("reqeust_irq for EINT19 err : %d!\n", ret);
		//return ret;
	}
	

	return 0;
}

ssize_t sixth_drv_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	if (size != 1)
		return -EINVAL;

	if (file->f_flags & O_NONBLOCK)
	{
		if (!ev_press)
			return -EAGAIN;
	}
	else
	{
		/* ���û�а�������, ���� */
		wait_event_interruptible(button_waitq, ev_press);
	}

	/* ����а�������, ���ؼ�ֵ */
	copy_to_user(buf, &key_val, 1);
	ev_press = 0;
	
	return 1;
}


int sixth_drv_close(struct inode *inode, struct file *file)
{
	int i;
	int ret;
	
	//atomic_inc(&canopen);
	for (i = 0; i < sizeof(pins_desc)/sizeof(pins_desc[0]); i++) {
		free_irq(pins_desc[i].irq, &pins_desc[i]);
	}
	up(&button_lock);
	return 0;
}

static unsigned sixth_drv_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	poll_wait(file, &button_waitq, wait); // ������������

	if (ev_press)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static int sixth_drv_fasync (int fd, struct file *filp, int on)
{
	printk("driver: sixth_drv_fasync\n");
	return fasync_helper (fd, filp, on, &button_async);
}


static struct file_operations sencod_drv_fops = {
    .owner   =  THIS_MODULE,    /* ����һ���꣬�������ģ��ʱ�Զ�������__this_module���� */
    .open    =  sixth_drv_open,     
	.read	 =	sixth_drv_read,	   
	.release =  sixth_drv_close,
	.poll    =  sixth_drv_poll,
	.fasync	 =  sixth_drv_fasync,
};


int major;

static void buttons_timer_function(struct timer_list *t)
{
	struct pin_desc * pindesc = irq_pd;
	unsigned int pinval;

	if (!pindesc)
		return;
	
	pinval = gpio_get_value(pindesc->pin);

	if (pinval)
	{
		/* �ɿ� */
		key_val = 0x80 | pindesc->key_val;
	}
	else
	{
		/* ���� */
		key_val = pindesc->key_val;
	}

    ev_press = 1;                  /* ��ʾ�жϷ����� */
    wake_up_interruptible(&button_waitq);   /* �������ߵĽ��� */
	
	kill_fasync (&button_async, SIGIO, POLL_IN);
}


static int sixth_drv_init(void)
{
	//init_timer(&buttons_timer);
	//buttons_timer.function = buttons_timer_function;
	//buttons_timer.expires  = 0;
	timer_setup(&buttons_timer, buttons_timer_function, 0);
	add_timer(&buttons_timer); 

	major = register_chrdev(0, "sixth_drv", &sencod_drv_fops);

	sixthdrv_class = class_create(THIS_MODULE, "sixth_drv");

	/* Ϊ����mdev������Щ��Ϣ�������豸�ڵ� */
	sixthdrv_class_dev = device_create(sixthdrv_class, NULL, MKDEV(major, 0), NULL, "buttons"); /* /dev/buttons */

	gpfcon = (volatile unsigned long *)ioremap(0x56000050, 16);
	gpfdat = gpfcon + 1;

	gpgcon = (volatile unsigned long *)ioremap(0x56000060, 16);
	gpgdat = gpgcon + 1;

	sema_init(&button_lock, 1);

	return 0;
}

static void sixth_drv_exit(void)
{
	del_timer(&buttons_timer); 
	unregister_chrdev(major, "sixth_drv");
	device_destroy(sixthdrv_class, MKDEV(major, 0));
	class_destroy(sixthdrv_class);
	iounmap(gpfcon);
	iounmap(gpgcon);
}

static int buttons_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;		
	struct device_node *dp_node = dev->of_node;
	struct resource		*res;
	int i;

	for (i = 0; i < sizeof(pins_desc)/sizeof(pins_desc[0]); i++)
	{
		/* ����platform_device����Դ���л���жϺ�,�������� */
		res = platform_get_resource(pdev, IORESOURCE_IRQ, i);
		if (res) {
			pins_desc[i].irq  = res->start;
			printk("get irq %d\n", pins_desc[i].irq);
		}
		else {
			printk("can not get irq res for eint0\n");
			return -1;
		}

		pins_desc[i].pin = of_get_named_gpio(dp_node, "eint-pins", i);
		printk("pins_desc[%d].pin = %d\n", i, pins_desc[i].pin);
	}

	return sixth_drv_init();
}

static int buttons_remove(struct platform_device *pdev)
{
	sixth_drv_exit();
	return 0;
}



static const struct of_device_id of_match_buttons[] = {
	{ .compatible = "jz2440_button", .data = NULL },
	{ /* sentinel */ }
};


struct platform_driver buttons_drv = {
	.probe		= buttons_probe,
	.remove		= buttons_remove,
	.driver		= {
		.name	= "mybuttons",
		.of_match_table = of_match_buttons, /* ��֧����Щ������dts��platform_device */
	}
};


static int buttons_init(void)
{
	platform_driver_register(&buttons_drv);
	return 0;
}

static void buttons_exit(void)
{
	platform_driver_unregister(&buttons_drv);
}



module_init(buttons_init);
module_exit(buttons_exit);
MODULE_LICENSE("GPL");

