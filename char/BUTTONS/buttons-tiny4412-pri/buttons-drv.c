/*
 * linux/drivers/char/tiny4412_buttons.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>

#include <mach/map.h>
#include <mach/gpio.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>

#define DEVICE_NAME		"buttons"

struct button_desc {
	int gpio;
	int number;
	char *name;	
	struct timer_list timer;
};

static struct button_desc buttons[] = {
	{ EXYNOS4_GPX3(2), 0, "KEY0" },
	{ EXYNOS4_GPX3(3), 1, "KEY1" },
	{ EXYNOS4_GPX3(4), 2, "KEY2" },
	{ EXYNOS4_GPX3(5), 3, "KEY3" },
};

static volatile char key_values[] = {
	'0', '0', '0', '0', '0', '0', '0', '0'
};


static DECLARE_WAIT_QUEUE_HEAD(button_waitq);

static volatile int ev_press = 0;

static void tiny4412_buttons_timer(unsigned long _data)
{
	struct button_desc *bdata = (struct button_desc *)_data;
	int down;
	int number;
	unsigned tmp;

	tmp = gpio_get_value(bdata->gpio);//��ȡ�˿�����ֵ

	/* active low */
	down = !tmp;
	printk(KERN_DEBUG "KEY %d: %08x\n", bdata->number, down);//���������Ϣ

	number = bdata->number;
	//����а������£���downֵ�ı䣬��¼
	if (down != (key_values[number] & 1)) {
		key_values[number] = '0' + down;

		ev_press = 1;
		wake_up_interruptible(&button_waitq);//����
	}
}

static irqreturn_t button_interrupt(int irq, void *dev_id)
{
	struct button_desc *bdata = (struct button_desc *)dev_id;
	
	mod_timer(&bdata->timer, jiffies + msecs_to_jiffies(40));//������ʱ����40ms
	
	return IRQ_HANDLED;
	
}


static int tiny4412_buttons_open(struct inode *inode, struct file *file)
{
	int irq;
	int i;
	int err = 0;
	
	for(i=0;i<ARRAY_SIZE(buttons);i++)
	{
		if (!buttons[i].gpio)//����˿�δ���壬�������ж�
			continue;
		
		//timer_setup(timer, callback, flags) ����Ϊ��һ��ʹ��timer����׼����һ���add_timer ���ʹ�ã������Ǻ�mod_timer���ʹ��	
		setup_timer(&buttons[i].timer, tiny4412_buttons_timer,
				(unsigned long)&buttons[i]);
		
		irq = gpio_to_irq(buttons[i].gpio);//��gpioתΪ��Ӧ��irq
		err = request_irq(irq, button_interrupt, IRQ_TYPE_EDGE_BOTH, 
				buttons[i].name, (void *)&buttons[i]);//�����ж�
		if (err)
			break;
	}
	
	if (err) {
		i--;
		for (; i >= 0; i--) {
			if (!buttons[i].gpio)
				continue;

			irq = gpio_to_irq(buttons[i].gpio);
			disable_irq(irq);
			free_irq(irq, (void *)&buttons[i]);

			del_timer_sync(&buttons[i].timer);
			//del_timer_sync����Ҫ��ɵ��������ͬdel_timerһ����
			//��ʱ��������ɾ��һ����ʱ�������⣬����ȷ������������
			//ʱϵͳ��û���κδ���������ִ�ж�ʱ�������ϵĶ�ʱ������
		}

		return -EBUSY;
	}

	ev_press = 1;
	return 0;
}

static int tiny4412_buttons_close(struct inode *inode, struct file *file)
{
	int irq, i;

	for (i = 0; i < ARRAY_SIZE(buttons); i++) {
		if (!buttons[i].gpio)
			continue;

		irq = gpio_to_irq(buttons[i].gpio);
		free_irq(irq, (void *)&buttons[i]);

		del_timer_sync(&buttons[i].timer);
	}

	return 0;
}

static int tiny4412_buttons_read(struct file *filp, char __user *buff,
		size_t count, loff_t *offp)
{
	unsigned long err;

	if (!ev_press) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		else
			wait_event_interruptible(button_waitq, ev_press);
	}
	
	ev_press=0;
	
	err = copy_to_user((void *)buff, (const void *)(&key_values),
			min(sizeof(key_values), count));
			
	return err ? -EFAULT : min(sizeof(key_values), count);
}

static unsigned int tiny4412_buttons_poll( struct file *file,
		struct poll_table_struct *wait)
{
	unsigned int mask = 0;

	/*��ѯ�Ĺ�����Ҳ�ǿ������ߣ�����һ��ʱ���������ߣ�����а�������
	�Ļ��������̷��أ����û�а������µĻ��������һ��ʱ�䷵�ء�*/
	poll_wait(file, &button_waitq, wait);
	if (ev_press)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static struct file_operations dev_fops = {
	.owner		= THIS_MODULE,
	.open		= tiny4412_buttons_open,
	.release	= tiny4412_buttons_close, 
	.read		= tiny4412_buttons_read,
	.poll		= tiny4412_buttons_poll,
};

static struct miscdevice misc = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= DEVICE_NAME,
	.fops		= &dev_fops,
};

static int __init button_dev_init(void)
{
	int ret;

	ret = misc_register(&misc);

	printk(DEVICE_NAME"\tinitialized\n");

	return ret;
}

static void __exit button_dev_exit(void)
{
	misc_deregister(&misc);
}

module_init(button_dev_init);
module_exit(button_dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("FriendlyARM Inc.");