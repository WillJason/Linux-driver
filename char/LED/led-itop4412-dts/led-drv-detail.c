/*
Ӳ��ƽ̨��itop4412
ϵͳ��linux-4.14.2
linux�ں˵�leds-gpio��ʹ��GPIO����LED��������ֻҪ��������LED�ƶԽӵ�GPIO���źŽ�����
�������ã�����ʹ����������ˣ�ʮ�ַ��㡣
leds-gpio��װ��ʮ�ֺã�ֻ��Ҫ�ṩ������ʹ�õ�GPIO���ɡ����⻹�߱����������ܣ���ʵ����
����LED������(��Ƶ��)������default-on�ǵ���LED�ƵĴ�������û��ȡ��ǰһֱ���š�
heartbeat��������������������ʵ�����˴������ǿ�����˸2�Σ�Ȼ����������ʱ�������ʱ�䳤��
timerΪ��ʱ����������1HZ����������������ideӲ�̡�mmc��CPU���������Ͳ�һһ�����ˡ�

leds����λ��drivers/ledsĿ¼��leds-gpio��������Ϊ��leds-gpio���������ļ�Ϊdrivers/leds/leds-gpio.c��

����������λ��drivers/leds/triggerĿ¼��
*/
//�ں����leds-gpio�豸��������ѡ��:
make menuconfig ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
  Device Drivers  --->
    -*- LED Support  --->
      <*>   LED Support for GPIO connected LEDs
//ͨ������ѡ��İ�����Ϣ���Բ����Ӧ���豸����Դ�ļ�: drivers/leds/leds-gpio.c 


//LED��ؽṹ��

//����������ʹ��gpio_led��LED���и�ֵ������LED���ơ�GPIO���źš��������ĸ���ƽ������Ĭ��״̬��
//gpio_led�ṹ�嶨�����£�

struct gpio_led {
    const char *name; // ���ƣ�������/sys/.../leds/nameĿ¼
    const char *default_trigger; // Ĭ�ϴ���������д�ɲ�д���������п������¸�ֵ
    unsigned     gpio; // GPIO���ź�
    unsigned    active_low : 1; // Ϊ1��ʾ�͵�ƽLED����
    unsigned    retain_state_suspended : 1;
    unsigned    default_state : 2; // Ĭ��״̬
    /* default_state should be one of LEDS_GPIO_DEFSTATE_(ON|OFF|KEEP) */
};


//���⻹Ҫ��дgpio_led_platform_data�ṹ�壬�䶨�����£�

struct gpio_led_platform_data {
    int         num_leds; // һ���ж��ٸ�LED��
    const struct gpio_led *leds; // ����Ľṹ��ָ��

#define GPIO_LED_NO_BLINK_LOW    0    /* No blink GPIO state low */
#define GPIO_LED_NO_BLINK_HIGH    1    /* No blink GPIO state high */
#define GPIO_LED_BLINK        2    /* Please, blink */
    int        (*gpio_blink_set)(unsigned gpio, int state,
                    unsigned long *delay_on,
                    unsigned long *delay_off); // LED��˸�ص�����������ΪNULL
};


//leds-gpio.cԴ�ļ����豸����ص���Ҫ����:
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
//�����������ݿɵ�֪�����豸������һ��platform_driver�Ķ��󣬿�ͨ��platform_device��
//�豸������豸�ڵ���ƥ�䡣�豸�ڵ����compatible����ֵӦ��"gpio-leds".//
//ƥ���Ϻ� �豸������gpio_led_probe�����ͻᱻ��������. ��probe������ͻ�ȡ���豸�����ṩ��Ӳ����Դ.
static int gpio_led_probe(struct platform_device *pdev)
{
	struct gpio_led_platform_data *pdata = dev_get_platdata(&pdev->dev); //ʹ���豸���ķ�ʽ, pdataӦΪNULL;
	struct gpio_leds_priv *priv;
	int i, ret = 0;

	if (pdata && pdata->num_leds) {//ʹ��platform_device��ʽ�Ļ�ȡӲ����Դ�Ĵ������
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
		priv = gpio_leds_create(pdev);//ʹ���豸��ʱ�Ĵ������
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

	/*��ȡ�ӽڵ�ĸ���, ��ζ���豸�ڵ�������Ҫ�����ӽڵ�ģ�
	������ʹ���豸�ڵ�����������ṩ��Դ.*/
	count = device_get_child_node_count(dev);
	if (!count)
		return ERR_PTR(-ENODEV);

	priv = devm_kzalloc(dev, sizeof_gpio_leds_priv(count), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	device_for_each_child_node(dev, child) {//�����豸�ڵ����ÿһ���ӽڵ�
		struct gpio_led_data *led_dat = &priv->leds[priv->num_leds];
		struct gpio_led led = {};
		const char *state = NULL;
		struct device_node *np = to_of_node(child);

	/*��ȡ�ӽڵ��label����ֵ. ��ζ��ÿ���ӽڵ�Ӧ��һ��label���ԣ�����ֵӦΪ�ַ���.*/
		ret = fwnode_property_read_string(child, "label", &led.name);
		if (ret && IS_ENABLED(CONFIG_OF) && np)
			led.name = np->name;
		if (!led.name) {
			fwnode_handle_put(child);
			return ERR_PTR(-EINVAL);
		}

	/*��ȡ�ӽڵ����gpio����Ϣ. con_idΪNULL����ζ���ӽڵ�Ӧ��ʹ��gpios������
	�ṩled�����ӵ�io����Ϣ. ����������ǻ�ȡһ��io����Ϣ�������Ƕ��io�ڣ�
	��ζ��ÿ���ӽڵ��ʾһ��led�Ƶ���Դ*/
		led.gpiod = devm_fwnode_get_gpiod_from_child(dev, NULL, child,
							     GPIOD_ASIS,
							     led.name);
		if (IS_ERR(led.gpiod)) {//��ȡio��ʧ���򷵻ش�����
			fwnode_handle_put(child);
			return ERR_CAST(led.gpiod);
		}

	//���������Ӧ�ǿ�ѡ���õģ���û�в���ֵ�����ش�����.
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
//leds-gpio�豸�������豸��:
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
	��������Ҫ���gpio_led_data�ṹ��
	struct gpio_led_data {
	struct led_classdev cdev;
	struct gpio_desc *gpiod;
	u8 can_sleep;
	u8 blinking;
	gpio_blink_set_t platform_gpio_blink_set;
	
	};
		
	�Ƚ���Ҫ�ĳ��õ������������漸���� 
		led->cdev.name = pdata->name; 
		led->cdev.brightness = 0; 
		led->cdev.brightness_set = s5pv210_led_set; 
		led->pdata = pdata; 
		
	led���豸�Ĳ���������Ҫ��������brightness��max_brightness��trigger��
	�ֱ����led������״̬������led�������ֵ������led��˸״̬����
	����max_brightness������ʾ�������ֵ��ֻ�ɶ�����д�����������������ж�дȨ�ޡ�
	Ӧ�ò����ͨ���⼸��������ȡ����ǰled�Ĺ���״̬��Ҳ���Բ�����Щ�ļ����������Ӳ��led�� 
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
		/*����devm_led_classdev_register��kernel��ע��led������kernel���ջ����
		cdev.brightness_set�����о���gpio_led_set������led*/
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

	//���ں�ע��led�豸 
	return devm_of_led_classdev_register(parent, np, &led_dat->cdev);
}	
	


//����ϵͳ�� Ȼ��Ϳ��Բ鿴��myleds�豸�ڵ����platform_device��Ϣ:
# ls /sys/bus/platform/devices/myleds/
driver/          leds/            of_node/         subsystem/
driver_override  modalias         power/           uevent

//������driverĿ¼����ʾ����platform_driverƥ������.

# ls /sys/bus/platform/devices/myleds/leds/led
red:system/  red:user/    
//��ʾ��led��ϵͳ��ᴴ����/sys/class/leds/led0   /sys/class/leds/led1Ŀ¼

//����led:
echo 1 > /sys/class/leds/red:user/brightness   // labelΪled0��led����
echo 0 > /sys/class/leds/red:user/brightness   // labelΪled0��led����


/*������*/

//ֱ�Ӳ鿴trigger�ļ�������֪����ǰϵͳ֧�ֵĴ�����
/*���ô������ܼ򵥣�ʹ��ecoh����Ҫ�Ĵ���������д��trigger�ļ����ɡ�
ע�⣬д����ַ���һ����trigger�ļ��Ѿ����ڵģ��������ʾ�����Ƿ���д������������ʾ����
*/
echo heartbeat > /sys/bus/platform/devices/leds-gpio/leds/red/trigger//(�Ǳ��������ԣ������ο�)
//��ʱ�����Ϻ��Ӧ����˸��











