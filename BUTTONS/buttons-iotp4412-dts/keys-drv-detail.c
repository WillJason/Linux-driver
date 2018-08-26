/*
Ӳ��ƽ̨��itop4412
ϵͳ��linux-4.14.2
Gpio-keys�ǻ���input�ܹ�ʵ�ֵ�һ��ͨ��GPIO����������
����������platform_driver�ܹ���ʵ�����������豸���룬����Linux�豸����ģ�͵�˼�롣
�����еİ�����������һ�㶼�����gpio-keys��д�����������б�Ҫ��gpio_keys���з�����
*/
//���豸�������ں�������:
make menuconfig ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
  Device Drivers  ---> 
     Input device support  ---> 
        [*]   Keyboards  ---> 
           <*>   GPIO Buttons
//��Ӧ���豸����Դ�ļ�: drivers/input/kerboard/leds/gpio-keys.c

//gpio-keys.cԴ�ļ����豸����ص���Ҫ����:
static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.driver		= {
		.name	= "gpio-keys",
		.pm	= &gpio_keys_pm_ops,
		.of_match_table = gpio_keys_of_match,
	}
};
static const struct of_device_id gpio_keys_of_match[] = {
	{ .compatible = "gpio-keys", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_keys_of_match);
//�����������ݿɵ�֪�����豸������һ��platform_driver�Ķ��󣬿�ͨ��platform_device��
//�豸������豸�ڵ���ƥ�䡣�豸�ڵ����compatible����ֵӦ��"gpio-keys".//
//ƥ���Ϻ� �豸������gpio_keys_probe�����ͻᱻ��������. ��probe������ͻ�ȡ���豸�����ṩ��Ӳ����Դ.
static int gpio_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct gpio_keys_platform_data *pdata = dev_get_platdata(dev);//ʹ���豸���ķ�ʽ, pdataӦΪNULL;
	struct fwnode_handle *child = NULL;
	struct gpio_keys_drvdata *ddata;
	struct input_dev *input;
	size_t size;
	int i, error;
	int wakeup = 0;

	if (!pdata) {
		pdata = gpio_keys_get_devtree_pdata(dev);//��ȡ�豸�����豸�ڵ��ṩ����Դ
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	size = sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data);
	ddata = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!ddata) {
		dev_err(dev, "failed to allocate state\n");
		return -ENOMEM;
	}

	ddata->keymap = devm_kcalloc(dev,
				     pdata->nbuttons, sizeof(ddata->keymap[0]),
				     GFP_KERNEL);
	if (!ddata->keymap)
		return -ENOMEM;

	input = devm_input_allocate_device(dev);//����input�豸
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	//��ʼ��input_dev����ĳ�Ա
	ddata->pdata = pdata;
	ddata->input = input;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdata->name ? : pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = dev;
	input->open = gpio_keys_open;
	input->close = gpio_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	input->keycode = ddata->keymap;
	input->keycodesize = sizeof(ddata->keymap[0]);
	input->keycodemax = pdata->nbuttons;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	for (i = 0; i < pdata->nbuttons; i++) {
		const struct gpio_keys_button *button = &pdata->buttons[i];

		if (!dev_get_platdata(dev)) {
			child = device_get_next_child_node(dev, child);//��ȡ�豸�ڵ�����ӽڵ�
			if (!child) {
				dev_err(dev,
					"missing child device node for entry %d\n",
					i);
				return -EINVAL;
			}
		}

		//�����ӽڵ��ṩ������ֵ����input_dev������֧�ֵļ���,�����ò���ȡio����Ϣ.
		error = gpio_keys_setup_key(pdev, input, ddata,
					    button, i, child);
		if (error) {
			fwnode_handle_put(child);
			return error;
		}

		if (button->wakeup)
			wakeup = 1;
	}

	fwnode_handle_put(child);

	/*
	devm_device_add_group-����->
		error = sysfs_create_group(&dev->kobj, grp);
		sysfs_create_group()��kobjĿ¼�´���һ�����Լ��ϣ�����ʾ�����е������ļ���
		����ļ��Ѵ��ڣ��ᱨ�������Ҫ��/sys/bus/......Ŀ¼�´�����Ӧ�ļ�, ����Ҫ�õ�sysfs_create_group()������ 
	*/
	error = devm_device_add_group(dev, &gpio_keys_attr_group);
	if (error) {
		dev_err(dev, "Unable to export keys/switches, error: %d\n",
			error);
		return error;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		return error;
	}

	device_init_wakeup(dev, wakeup);

	return 0;
}


static struct gpio_keys_platform_data *
gpio_keys_get_devtree_pdata(struct device *dev)
{
	struct gpio_keys_platform_data *pdata;
	struct gpio_keys_button *button;
	struct fwnode_handle *child;
	int nbuttons;

	nbuttons = device_get_child_node_count(dev);//ͨ���豸�ڵ���ӽڵ����ṩ��Դ
	if (nbuttons == 0)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev,
			     sizeof(*pdata) + nbuttons * sizeof(*button),
			     GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	button = (struct gpio_keys_button *)(pdata + 1);

	pdata->buttons = button;
	pdata->nbuttons = nbuttons;

	/*
	��ʾ�豸�ڵ������һ��autorepeat���ԣ�����ֵΪbool����(0/1). 
	���ڱ�ʾ�����豸�Ƿ��Զ�������ظ��ύ����.
	*/
	pdata->rep = device_property_read_bool(dev, "autorepeat");
	//��ʾ�豸�ڵ������һ��label����, ������Ϊstring����.����ָ�������豸������
	device_property_read_string(dev, "label", &pdata->name);

	device_for_each_child_node(dev, child) {//�����ӽڵ�
		if (is_of_node(child))
			button->irq =
				irq_of_parse_and_map(to_of_node(child), 0);

		/*
		��ζ��ÿ���ӽڵ㶼������"linux,code"���ԣ�����ֵΪu32���͡�����ָ�����ӽڵ��Ӧ�İ����ļ���.
		*/
		if (fwnode_property_read_u32(child, "linux,code",
					     &button->code)) {
			dev_err(dev, "Button without keycode\n");
			fwnode_handle_put(child);
			return ERR_PTR(-EINVAL);
		}
		//ÿ���ӽڵ㻹������һ��label���ԣ�����ֵΪstring����
		fwnode_property_read_string(child, "label", &button->desc);
		
		//ÿ���ֽڵ㻹����ͨ��"linux,input-type"������ָ�������豸��֧�ֵ��¼�����.  
		//������ṩ������ΪEV_KEY
		if (fwnode_property_read_u32(child, "linux,input-type",
					     &button->type))
			button->type = EV_KEY;

		button->wakeup =
			fwnode_property_read_bool(child, "wakeup-source") ||
			/* legacy name */
			fwnode_property_read_bool(child, "gpio-key,wakeup");

		button->can_disable =
			fwnode_property_read_bool(child, "linux,can-disable");

	//�����жϴ����İ�������"debounce-interval"�����������ã����Ƕ�ʱ��ѯ��ʽ������Ҫ���ô˼��ʱ��.
		if (fwnode_property_read_u32(child, "debounce-interval",
					 &button->debounce_interval))
			button->debounce_interval = 5;

		button++;
	}

	return pdata;
}

static int gpio_keys_setup_key(struct platform_device *pdev,
				struct input_dev *input,
				struct gpio_keys_drvdata *ddata,
				const struct gpio_keys_button *button,
				int idx,
				struct fwnode_handle *child)
{
	const char *desc = button->desc ? button->desc : "gpio_keys";
	struct device *dev = &pdev->dev;
	struct gpio_button_data *bdata = &ddata->data[idx];
	irq_handler_t isr;
	unsigned long irqflags;
	int irq;
	int error;

	bdata->input = input;
	bdata->button = button;
	spin_lock_init(&bdata->lock);

	if (child) {
		// con_idΪNULL, ���ʾ�ӽڵ���Ӧ��gpios�������ṩio����Դ.
		bdata->gpiod = devm_fwnode_get_gpiod_from_child(dev, NULL,
								child,
								GPIOD_IN,
								desc);
		if (IS_ERR(bdata->gpiod)) {
			error = PTR_ERR(bdata->gpiod);
			if (error == -ENOENT) {
				/*
				 * GPIO is optional, we may be dealing with
				 * purely interrupt-driven setup.
				 */
				bdata->gpiod = NULL;
			} else {
				if (error != -EPROBE_DEFER)
					dev_err(dev, "failed to get gpio: %d\n",
						error);
				return error;
			}
		}
	} else if (gpio_is_valid(button->gpio)) {
		/*
		 * Legacy GPIO number, so request the GPIO here and
		 * convert it to descriptor.
		 */
		unsigned flags = GPIOF_IN;

		if (button->active_low)
			flags |= GPIOF_ACTIVE_LOW;

		error = devm_gpio_request_one(dev, button->gpio, flags, desc);
		if (error < 0) {
			dev_err(dev, "Failed to request GPIO %d, error %d\n",
				button->gpio, error);
			return error;
		}

		bdata->gpiod = gpio_to_desc(button->gpio);
		if (!bdata->gpiod)
			return -EINVAL;
	}

	if (bdata->gpiod) {
		if (button->debounce_interval) {
			error = gpiod_set_debounce(bdata->gpiod,
					button->debounce_interval * 1000);
			/* use timer if gpiolib doesn't provide debounce */
			if (error < 0)
				bdata->software_debounce =
						button->debounce_interval;
		}

		if (button->irq) {
			bdata->irq = button->irq;
		} else {
			irq = gpiod_to_irq(bdata->gpiod);
			if (irq < 0) {
				error = irq;
				dev_err(dev,
					"Unable to get irq number for GPIO %d, error %d\n",
					button->gpio, error);
				return error;
			}
			bdata->irq = irq;
		}

		INIT_DELAYED_WORK(&bdata->work, gpio_keys_gpio_work_func);

		isr = gpio_keys_gpio_isr;
		irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	} else {
		if (!button->irq) {
			dev_err(dev, "Found button without gpio or irq\n");
			return -EINVAL;
		}

		bdata->irq = button->irq;

		if (button->type && button->type != EV_KEY) {
			dev_err(dev, "Only EV_KEY allowed for IRQ buttons.\n");
			return -EINVAL;
		}

		bdata->release_delay = button->debounce_interval;
		setup_timer(&bdata->release_timer,
			    gpio_keys_irq_timer, (unsigned long)bdata);

		isr = gpio_keys_irq_isr;
		irqflags = 0;
	}

	bdata->code = &ddata->keymap[idx];
	*bdata->code = button->code;
	input_set_capability(input, button->type ?: EV_KEY, *bdata->code);

	/*
	 * Install custom action to cancel release timer and
	 * workqueue item.
	 */
	error = devm_add_action(dev, gpio_keys_quiesce_key, bdata);
	if (error) {
		dev_err(dev, "failed to register quiesce action, error: %d\n",
			error);
		return error;
	}

	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = devm_request_any_context_irq(dev, bdata->irq, isr, irqflags,
					     desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			bdata->irq, error);
		return error;
	}

	return 0;
}

//�豸������豸�ڵ�:
gpio-keys {
		compatible = "gpio-keys";

		home {
			label = "GPIO Key Home";
			linux,code = <KEY_HOME>;
			gpios = <&gpx1 1 GPIO_ACTIVE_LOW>;
		};

		back {
			label = "GPIO Key Back";
			linux,code = <KEY_BACK>;
			gpios = <&gpx1 2 GPIO_ACTIVE_LOW>;
		};

		sleep {
			label = "GPIO Key Sleep";
			linux,code = <KEY_POWER>;
			gpios = <&gpx3 3 GPIO_ACTIVE_LOW>;
		};

		vol-up {
			label = "GPIO Key Vol+";
			linux,code = <KEY_UP>;
			gpios = <&gpx2 1 GPIO_ACTIVE_LOW>;
		};

		vol-down {
			label = "GPIO Key Vol-";
			linux,code = <KEY_DOWN>;
			gpios = <&gpx2 0 GPIO_ACTIVE_LOW>;
		};
};
//����ϵͳ�󣬿��Բ鿴��:
~ # cat /sys/class/input/                                                       
event0/  input0/
~ # cat /proc/bus/input/devices                                                 
I: Bus=0019 Vendor=0001 Product=0001 Version=0100
N: Name="gpio-keys"
P: Phys=gpio-keys/input0
S: Sysfs=/devices/platform/gpio-keys/input/input0
U: Uniq=
H: Handlers=kbd event0 
B: PROP=0
B: EV=3
B: KEY=40000000 1010c0 0 0 0


~ # cat /dev/input/event0                                                       
YfYf    ?      It
g
NgN
//���°����������Ϣ��������
//���ļ�ϵͳdevĿ¼����event0�豸�ڵ㣬��gpio-keys�����ķ��ʿ���ͨ��event0����ɡ�
/*�ο���Ӧ�ò��Դ���*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <time.h>
#include <fcntl.h>
#include <linux/input.h>
int main(int argc, char **argv)
{
    int key_state;
     int fd;
     int ret;
     int code;
     struct input_event buf;
     int repeat_param[2];
     fd = open("/dev/input/event0", O_RDONLY);
     if (fd < 0)
     {
         printf("Open gpio-keys failed.\n");
         return -1;
  }
  else
  {
     printf("Open gpio-keys success.\n");
  }
     repeat_param[0]=500;//ms�ظ�������һ�μ��
     repeat_param[1]=66;//ms�ظ������������
     ret = ioctl(fd,EVIOCSREP,(int *)repeat_param);//�����ظ���������
     if(ret != 0)
         {
                   printf("set repeat_param fail!\n");
         }
     else
         {
                printf("set repeat_param ok.\n");
         }
     while(1)
     {
         ret = read(fd,&buf,sizeof(struct input_event));
         if(ret <= 0)
              {
                   printf("read fail!\n");
                   return -1;
              }
             
       code = buf.code;
       key_state = buf.value;
       switch(code)
       {
          case KEY_DOWN:
              code = '1';
              break;
          case KEY_ENTER:
              code = '2';
              break;
          case KEY_HOME:
              code = '3';
              break;
          case KEY_POWER:
              code = '4';
              break;
          default:
              code = 0;
              break;
       }
       if(code!=0)
          {
           printf("Key_%c state= %d.\n",code,key_state);
         }
     }
     close(fd);
     printf("Key test finished.\n"); 
     return 0;
}




























