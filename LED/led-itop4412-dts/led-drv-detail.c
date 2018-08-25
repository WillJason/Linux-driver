/*
硬件平台：itop4412
系统：linux-4.14.2
linux内核的leds-gpio是使用GPIO控制LED的驱动，只要将板子上LED灯对接的GPIO引脚号进行适
当的配置，就能使用这个驱动了，十分方便。
leds-gpio封装得十分好，只需要提供可正常使用的GPIO即可。另外还具备触发器功能，其实就是
控制LED的亮灭(及频率)。比如default-on是点亮LED灯的触发器，没有取消前一直亮着。
heartbeat是心跳触发器，经笔者实践，此触发器是快速闪烁2次，然后灭掉，灭掉时间较亮的时间长。
timer为定时触发器，即1HZ内亮灭。其它还有如ide硬盘、mmc、CPU触发器，就不一一介绍了。

leds驱动位于drivers/leds目录。leds-gpio驱动名称为“leds-gpio”，驱动文件为drivers/leds/leds-gpio.c。

触发器驱动位于drivers/leds/trigger目录。
*/
//内核里的leds-gpio设备驱动配置选项:
make menuconfig ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
  Device Drivers  --->
    -*- LED Support  --->
      <*>   LED Support for GPIO connected LEDs
//通过配置选项的帮助信息可以查出对应的设备驱动源文件: drivers/leds/leds-gpio.c 


//LED相关结构体

//驱动开发者使用gpio_led对LED进行赋值，包括LED名称、GPIO引脚号、灯亮是哪个电平，还有默认状态。
//gpio_led结构体定义如下：

struct gpio_led {
    const char *name; // 名称，会生成/sys/.../leds/name目录
    const char *default_trigger; // 默认触发器，可写可不写，在命令行可以重新赋值
    unsigned     gpio; // GPIO引脚号
    unsigned    active_low : 1; // 为1表示低电平LED点亮
    unsigned    retain_state_suspended : 1;
    unsigned    default_state : 2; // 默认状态
    /* default_state should be one of LEDS_GPIO_DEFSTATE_(ON|OFF|KEEP) */
};


//另外还要填写gpio_led_platform_data结构体，其定义如下：

struct gpio_led_platform_data {
    int         num_leds; // 一共有多少个LED灯
    const struct gpio_led *leds; // 上面的结构体指针

#define GPIO_LED_NO_BLINK_LOW    0    /* No blink GPIO state low */
#define GPIO_LED_NO_BLINK_HIGH    1    /* No blink GPIO state high */
#define GPIO_LED_BLINK        2    /* Please, blink */
    int        (*gpio_blink_set)(unsigned gpio, int state,
                    unsigned long *delay_on,
                    unsigned long *delay_off); // LED闪烁回调函数，可置为NULL
};


//leds-gpio.c源文件里设备树相关的主要内容:
static struct platform_driver gpio_led_driver = {
	.probe		= gpio_led_probe,
	.shutdown	= gpio_led_shutdown,
	.driver		= {
		.name	= "leds-gpio",
		.of_match_table = of_gpio_leds_match,
	},
};
static const struct of_device_id of_gpio_leds_match[] = {
	{ .compatible = "gpio-leds", },
	{},
};
MODULE_DEVICE_TABLE(of, of_gpio_leds_match);
//这两部分内容可得知，此设备驱动是一个platform_driver的对象，可通过platform_device或
//设备树里的设备节点来匹配。设备节点里的compatible属性值应是"gpio-leds".//
//匹配上后， 设备驱动里gpio_led_probe函数就会被触发调用. 在probe函数里就会取出设备树里提供的硬件资源.
static int gpio_led_probe(struct platform_device *pdev)
{
	struct gpio_led_platform_data *pdata = dev_get_platdata(&pdev->dev); //使用设备树的方式, pdata应为NULL;
	struct gpio_leds_priv *priv;
	int i, ret = 0;

	if (pdata && pdata->num_leds) {//使用platform_device方式的获取硬件资源的处理代码
		priv = devm_kzalloc(&pdev->dev,
				sizeof_gpio_leds_priv(pdata->num_leds),
					GFP_KERNEL);
		if (!priv)
			return -ENOMEM;

		priv->num_leds = pdata->num_leds;
		for (i = 0; i < priv->num_leds; i++) {
			ret = create_gpio_led(&pdata->leds[i], &priv->leds[i],
					      &pdev->dev, NULL,
					      pdata->gpio_blink_set);
			if (ret < 0)
				return ret;
		}
	} else {
		priv = gpio_leds_create(pdev);//使用设备树时的处理代码
		if (IS_ERR(priv))
			return PTR_ERR(priv);
	}

	platform_set_drvdata(pdev, priv);

	return 0;
}

static struct gpio_leds_priv *gpio_leds_create(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fwnode_handle *child;
	struct gpio_leds_priv *priv;
	int count, ret;

	/*获取子节点的个数, 意味着设备节点里是需要包含子节点的，
	并不是使用设备节点的属性来的提供资源.*/
	count = device_get_child_node_count(dev);
	if (!count)
		return ERR_PTR(-ENODEV);

	priv = devm_kzalloc(dev, sizeof_gpio_leds_priv(count), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	device_for_each_child_node(dev, child) {//遍历设备节点里的每一个子节点
		struct gpio_led_data *led_dat = &priv->leds[priv->num_leds];
		struct gpio_led led = {};
		const char *state = NULL;
		struct device_node *np = to_of_node(child);

	/*获取子节点的label属性值. 意味着每个子节点应有一个label属性，属性值应为字符串.*/
		ret = fwnode_property_read_string(child, "label", &led.name);
		if (ret && IS_ENABLED(CONFIG_OF) && np)
			led.name = np->name;
		if (!led.name) {
			fwnode_handle_put(child);
			return ERR_PTR(-EINVAL);
		}

	/*获取子节点里的gpio口信息. con_id为NULL，意味着子节点应是使用gpios属性来
	提供led所连接的io口信息. 而且这里仅是获取一个io口信息，并不是多个io口，
	意味着每个子节点表示一个led灯的资源*/
		led.gpiod = devm_fwnode_get_gpiod_from_child(dev, NULL, child,
							     GPIOD_ASIS,
							     led.name);
		if (IS_ERR(led.gpiod)) {//获取io口失败则返回错误码
			fwnode_handle_put(child);
			return ERR_CAST(led.gpiod);
		}

	//下面的属性应是可选设置的，因并没有不设值而返回错误码.
		fwnode_property_read_string(child, "linux,default-trigger",
					    &led.default_trigger);

		if (!fwnode_property_read_string(child, "default-state",
						 &state)) {
			if (!strcmp(state, "keep"))
				led.default_state = LEDS_GPIO_DEFSTATE_KEEP;
			else if (!strcmp(state, "on"))
				led.default_state = LEDS_GPIO_DEFSTATE_ON;
			else
				led.default_state = LEDS_GPIO_DEFSTATE_OFF;
		}

		if (fwnode_property_present(child, "retain-state-suspended"))
			led.retain_state_suspended = 1;
		if (fwnode_property_present(child, "retain-state-shutdown"))
			led.retain_state_shutdown = 1;
		if (fwnode_property_present(child, "panic-indicator"))
			led.panic_indicator = 1;

		ret = create_gpio_led(&led, led_dat, dev, np, NULL);
		if (ret < 0) {
			fwnode_handle_put(child);
			return ERR_PTR(ret);
		}
		led_dat->cdev.dev->of_node = np;
		priv->num_leds++;
	}

	return priv;
}
//leds-gpio设备驱动的设备树:
leds {
		compatible = "gpio-leds";

		led2 {
			label = "red:system";
			gpios = <&gpx1 0 GPIO_ACTIVE_HIGH>;
			default-state = "off";
			linux,default-trigger = "heartbeat";
		};

		led3 {
			label = "red:user";
			gpios = <&gpk1 1 GPIO_ACTIVE_HIGH>;
			default-state = "off";
		};
};

static int create_gpio_led(const struct gpio_led *template,
	struct gpio_led_data *led_dat, struct device *parent,
	struct device_node *np, gpio_blink_set_t blink_set)
{
	int ret, state;

	/*
	接下来主要填充gpio_led_data结构体
	struct gpio_led_data {
	struct led_classdev cdev;
	struct gpio_desc *gpiod;
	u8 can_sleep;
	u8 blinking;
	gpio_blink_set_t platform_gpio_blink_set;
	
	};
		
	比较重要的常用的设置项是下面几个： 
		led->cdev.name = pdata->name; 
		led->cdev.brightness = 0; 
		led->cdev.brightness_set = s5pv210_led_set; 
		led->pdata = pdata; 
		
	led类设备的操作对象主要有三个：brightness、max_brightness、trigger。
	分别代表”led的亮灭状态“、”led最高亮度值“、”led闪烁状态“。
	其中max_brightness仅仅表示最大亮度值，只可读不可写。其他两个参数都有读写权限。
	应用层可以通过这几个参数获取到当前led的工作状态，也可以操作这些文件来间接设置硬件led。 
	*/
	led_dat->gpiod = template->gpiod;
	if (!led_dat->gpiod) {
		/*
		 * This is the legacy code path for platform code that
		 * still uses GPIO numbers. Ultimately we would like to get
		 * rid of this block completely.
		 */
		unsigned long flags = GPIOF_OUT_INIT_LOW;

		/* skip leds that aren't available */
		if (!gpio_is_valid(template->gpio)) {
			dev_info(parent, "Skipping unavailable LED gpio %d (%s)\n",
					template->gpio, template->name);
			return 0;
		}

		if (template->active_low)
			flags |= GPIOF_ACTIVE_LOW;

		ret = devm_gpio_request_one(parent, template->gpio, flags,
					    template->name);
		if (ret < 0)
			return ret;

		led_dat->gpiod = gpio_to_desc(template->gpio);
		if (!led_dat->gpiod)
			return -EINVAL;
	}

	led_dat->cdev.name = template->name;
	led_dat->cdev.default_trigger = template->default_trigger;
	led_dat->can_sleep = gpiod_cansleep(led_dat->gpiod);
	if (!led_dat->can_sleep)
		/*调用devm_led_classdev_register想kernel的注册led驱动，kernel最终会调用
		cdev.brightness_set本例中就是gpio_led_set来点亮led*/
		led_dat->cdev.brightness_set = gpio_led_set;
	else
		led_dat->cdev.brightness_set_blocking = gpio_led_set_blocking;
	led_dat->blinking = 0;
	if (blink_set) {
		led_dat->platform_gpio_blink_set = blink_set;
		led_dat->cdev.blink_set = gpio_blink_set;
	}
	if (template->default_state == LEDS_GPIO_DEFSTATE_KEEP) {
		state = gpiod_get_value_cansleep(led_dat->gpiod);
		if (state < 0)
			return state;
	} else {
		state = (template->default_state == LEDS_GPIO_DEFSTATE_ON);
	}
	led_dat->cdev.brightness = state ? LED_FULL : LED_OFF;
	if (!template->retain_state_suspended)
		led_dat->cdev.flags |= LED_CORE_SUSPENDRESUME;
	if (template->panic_indicator)
		led_dat->cdev.flags |= LED_PANIC_INDICATOR;
	if (template->retain_state_shutdown)
		led_dat->cdev.flags |= LED_RETAIN_AT_SHUTDOWN;

	ret = gpiod_direction_output(led_dat->gpiod, state);
	if (ret < 0)
		return ret;

	//向内核注册led设备 
	return devm_of_led_classdev_register(parent, np, &led_dat->cdev);
}	
	


//启动系统， 然后就可以查看到myleds设备节点产生platform_device信息:
# ls /sys/bus/platform/devices/myleds/
driver/          leds/            of_node/         subsystem/
driver_override  modalias         power/           uevent

//看到有driver目录即表示已与platform_driver匹配上了.

# ls /sys/bus/platform/devices/myleds/leds/led
red:system/  red:user/    
//表示在led子系统里会创建出/sys/class/leds/led0   /sys/class/leds/led1目录

//控制led:
echo 1 > /sys/class/leds/red:user/brightness   // label为led0的led灯亮
echo 0 > /sys/class/leds/red:user/brightness   // label为led0的led灯灭


/*触发器*/

//直接查看trigger文件，即可知道当前系统支持的触发器
/*设置触发器很简单，使用ecoh将需要的触发器名称写入trigger文件即可。
注意，写入的字符串一定是trigger文件已经存在的，否则会提示参数非法。写入心跳触发器示例：
*/
echo heartbeat > /sys/bus/platform/devices/leds-gpio/leds/red/trigger//(非本驱动测试，仅供参考)
//此时板子上红灯应会闪烁。











