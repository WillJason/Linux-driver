/*
硬件平台：itop4412
系统：linux-4.14.2
Gpio-keys是基于input架构实现的一个通用GPIO按键驱动。
该驱动基于platform_driver架构，实现了驱动和设备分离，符合Linux设备驱动模型的思想。
工程中的按键驱动我们一般都会基于gpio-keys来写，所以我们有必要对gpio_keys进行分析。
*/
//此设备驱动在内核里配置:
make menuconfig ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
  Device Drivers  ---> 
     Input device support  ---> 
        [*]   Keyboards  ---> 
           <*>   GPIO Buttons
//对应的设备驱动源文件: drivers/input/kerboard/leds/gpio-keys.c

//gpio-keys.c源文件里设备树相关的主要内容:
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
//这两部分内容可得知，此设备驱动是一个platform_driver的对象，可通过platform_device或
//设备树里的设备节点来匹配。设备节点里的compatible属性值应是"gpio-keys".//
//匹配上后， 设备驱动里gpio_keys_probe函数就会被触发调用. 在probe函数里就会取出设备树里提供的硬件资源.
static int gpio_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct gpio_keys_platform_data *pdata = dev_get_platdata(dev);//使用设备树的方式, pdata应为NULL;
	struct fwnode_handle *child = NULL;
	struct gpio_keys_drvdata *ddata;
	struct input_dev *input;
	size_t size;
	int i, error;
	int wakeup = 0;

	if (!pdata) {
		pdata = gpio_keys_get_devtree_pdata(dev);//获取设备树里设备节点提供的资源
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

	input = devm_input_allocate_device(dev);//分配input设备
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	//初始化input_dev对象的成员
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
			child = device_get_next_child_node(dev, child);//获取设备节点里的子节点
			if (!child) {
				dev_err(dev,
					"missing child device node for entry %d\n",
					i);
				return -EINVAL;
			}
		}

		//根据子节点提供的属性值设置input_dev对象所支持的键码,来设置并获取io口信息.
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
	devm_device_add_group-调用->
		error = sysfs_create_group(&dev->kobj, grp);
		sysfs_create_group()在kobj目录下创建一个属性集合，并显示集合中的属性文件。
		如果文件已存在，会报错。如果需要在/sys/bus/......目录下创建相应文件, 则需要用到sysfs_create_group()函数。 
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

	nbuttons = device_get_child_node_count(dev);//通过设备节点的子节点来提供资源
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
	表示设备节点可以有一个autorepeat属性，属性值为bool类型(0/1). 
	用于表示输入设备是否自动间隔地重复提交按键.
	*/
	pdata->rep = device_property_read_bool(dev, "autorepeat");
	//表示设备节点可以有一个label属性, 属性性为string类型.用于指定输入设备的名字
	device_property_read_string(dev, "label", &pdata->name);

	device_for_each_child_node(dev, child) {//遍历子节点
		if (is_of_node(child))
			button->irq =
				irq_of_parse_and_map(to_of_node(child), 0);

		/*
		意味着每个子节点都必须有"linux,code"属性，属性值为u32类型。用于指定此子节点对应的按键的键码.
		*/
		if (fwnode_property_read_u32(child, "linux,code",
					     &button->code)) {
			dev_err(dev, "Button without keycode\n");
			fwnode_handle_put(child);
			return ERR_PTR(-EINVAL);
		}
		//每个子节点还可以有一个label属性，属性值为string类型
		fwnode_property_read_string(child, "label", &button->desc);
		
		//每个字节点还可以通过"linux,input-type"属性来指定输入设备所支持的事件类型.  
		//如果不提供则设置为EV_KEY
		if (fwnode_property_read_u32(child, "linux,input-type",
					     &button->type))
			button->type = EV_KEY;

		button->wakeup =
			fwnode_property_read_bool(child, "wakeup-source") ||
			/* legacy name */
			fwnode_property_read_bool(child, "gpio-key,wakeup");

		button->can_disable =
			fwnode_property_read_bool(child, "linux,can-disable");

	//如是中断触发的按键，则"debounce-interval"属性无需设置，如是定时轮询方式的则需要设置此间隔时间.
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
		// con_id为NULL, 则表示子节点里应用gpios属性来提供io口资源.
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

//设备树里的设备节点:
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
//重启系统后，可以查看到:
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
//按下按键有输出信息，但乱码
//在文件系统dev目录下有event0设备节点，对gpio-keys按键的访问可以通过event0来完成。
/*参考的应用测试代码*/
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
     repeat_param[0]=500;//ms重复按键第一次间隔
     repeat_param[1]=66;//ms重复按键后续间隔
     ret = ioctl(fd,EVIOCSREP,(int *)repeat_param);//设置重复按键参数
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




























