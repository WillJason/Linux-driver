/*
硬件平台：itop4412
系统：linux-4.14.2

Exynos4412一共可以输出4路PWM（timer0、timer1、timer2、timer3产生的PWM有对应的输出引脚，
timer4没有对应的引脚），但是在tiny4412上只有两路PWM引出供片外外设使用，分别来自timer0和timer1，
其中timer0输出的PWM0用于控制底板上的无源蜂鸣器，timer1产生的PWM1用于控制LCD的背光亮度。

*/
//对应的设备驱动源文件: drivers/pwm/pwm-samsung.c
//pwm-samsung.c源文件里设备树相关的主要内容:
static struct platform_driver pwm_samsung_driver = {
	.driver		= {
		.name	= "samsung-pwm",
		.pm	= &pwm_samsung_pm_ops,
		.of_match_table = of_match_ptr(samsung_pwm_matches),
	},
	.probe		= pwm_samsung_probe,
	.remove		= pwm_samsung_remove,
};
static const struct of_device_id samsung_pwm_matches[] = {
	{ .compatible = "samsung,s3c2410-pwm", .data = &s3c24xx_variant },
	{ .compatible = "samsung,s3c6400-pwm", .data = &s3c64xx_variant },
	{ .compatible = "samsung,exynos4210-pwm", .data = &s5p64x0_variant },
	{},
};
MODULE_DEVICE_TABLE(of, samsung_pwm_matches);
//匹配上后， 设备驱动里pwm_samsung_probe函数就会被触发调用. 在probe函数里就会取出设备树里提供的硬件资源.
static int pwm_samsung_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct samsung_pwm_chip *chip;
	struct resource *res;
	unsigned int chan;
	int ret;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	chip->chip.dev = &pdev->dev;
	chip->chip.ops = &pwm_samsung_ops;//设备操作函数
	chip->chip.base = -1;
	chip->chip.npwm = SAMSUNG_PWM_NUM;
	chip->inverter_mask = BIT(SAMSUNG_PWM_NUM) - 1;

	if (IS_ENABLED(CONFIG_OF) && pdev->dev.of_node) {
		
		////获取设备ID表结构以及平台设备资源
		ret = pwm_samsung_parse_dt(chip);//因为我们使用的是设备树，所以使用这个分支，调用该函数
		if (ret)
			return ret;

		chip->chip.of_xlate = of_pwm_xlate_with_flags;
		chip->chip.of_pwm_n_cells = 3;
	} else {
		if (!pdev->dev.platform_data) {
			dev_err(&pdev->dev, "no platform data specified\n");
			return -EINVAL;
		}

		memcpy(&chip->variant, pdev->dev.platform_data,
							sizeof(chip->variant));
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	chip->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(chip->base))
		return PTR_ERR(chip->base);

	chip->base_clk = devm_clk_get(&pdev->dev, "timers");
	if (IS_ERR(chip->base_clk)) {
		dev_err(dev, "failed to get timer base clk\n");
		return PTR_ERR(chip->base_clk);
	}

	ret = clk_prepare_enable(chip->base_clk);
	if (ret < 0) {
		dev_err(dev, "failed to enable base clock\n");
		return ret;
	}

	for (chan = 0; chan < SAMSUNG_PWM_NUM; ++chan)
		if (chip->variant.output_mask & BIT(chan))
			pwm_samsung_set_invert(chip, chan, true);

	/* Following clocks are optional. */
	chip->tclk0 = devm_clk_get(&pdev->dev, "pwm-tclk0");
	chip->tclk1 = devm_clk_get(&pdev->dev, "pwm-tclk1");

	/* 设置私有数据 */
	platform_set_drvdata(pdev, chip);

	//初始化完成后的pwm chip可以通过pwmchip_add接口注册到kernel中，之后的事情，pwm driver就不用操心了。
	ret = pwmchip_add(&chip->chip);
	if (ret < 0) {
		dev_err(dev, "failed to register PWM chip\n");
		clk_disable_unprepare(chip->base_clk);
		return ret;
	}

	dev_dbg(dev, "base_clk at %lu, tclk0 at %lu, tclk1 at %lu\n",
		clk_get_rate(chip->base_clk),
		!IS_ERR(chip->tclk0) ? clk_get_rate(chip->tclk0) : 0,
		!IS_ERR(chip->tclk1) ? clk_get_rate(chip->tclk1) : 0);

	return 0;
}
/*
PWM framework使用struct pwm_chip抽象PWM控制器。通常情况下，在一个SOC中，可以同时支持多路
PWM输出（如6路），以便同时控制多个PWM设备。这样每一路PWM输出，可以看做一个PWM设备（由上面
struct pwm_device抽象），没有意外的话，这些PWM设备的控制方式应该类似。PWM framework会统一管理
这些PWM设备，将它们归类为一个PWM chip。（我们这里在pwm_chip的上层又封装了一个结构体samsung_pwm_chip）
samsung_pwm_chip的定义如下：
		struct samsung_pwm_chip {
			struct pwm_chip chip;
			struct samsung_pwm_variant variant;
			u8 inverter_mask;
			u8 disabled_mask;
		
			void __iomem *base;
			struct clk *base_clk;
			struct clk *tclk0;
			struct clk *tclk1;
		};
其中struct pwm_chip的定义如下：
		struct pwm_chip {
			struct device *dev;
			struct list_head list;
			const struct pwm_ops *ops;
			int base;
			unsigned int npwm;
		
			struct pwm_device *pwms;
		
			struct pwm_device * (*of_xlate)(struct pwm_chip *pc,
							const struct of_phandle_args *args);
			unsigned int of_pwm_n_cells;
		};
dev，该pwm chip对应的设备，一般由pwm driver对应的platform驱动指定。必须提供！
ops，操作PWM设备的回调函数，后面会详细介绍。必须提供！
npwm，该pwm chip可以支持的pwm channel（也可以称作pwm device由struct pwm_device表示）个数，kernel会根据该number，分配相应个数的struct pwm_device结构，保存在pwms指针中。必须提供！
pwms，保存所有pwm device的数组，kernel会自行分配，不需要driver关心。
base，在将该chip下所有pwm device组成radix tree时使用，只有旧的pwm_request接口会使用，因此忽略它吧，编写pwm driver不需要关心。
of_pwm_n_cells，该PWM chip所提供的DTS node的cell，一般是2或者3，例如：为3时，consumer需要在DTS指定pwm number、pwm period和pwm flag三种信息（如2.1中的介绍）；为2时，没有flag信息。
of_xlate，用于解析consumer中指定的、pwm信息的DTS node的回调函数（如2.1中介绍的，pwms = <&pwm 0 5000000 PWM_POLARITY_INVERTED>）。
注2：一般情况下，of_pwm_n_cells取值为3，或者2（不关心极性），of_xlate则可以使用kernel提供的of_pwm_xlate_with_flags（解析of_pwm_n_cells为3的chip）或者of_pwm_simple_xlate（解析of_pwm_n_cells为2的情况）。具体的driver可以根据实际情况修改上述规则，但不到万不得已的时候，不要做这种非标准的、掏力不讨好的事情！（有关of_xlate的流程，会在下一篇流程分析的文章中介绍。）
can_sleep，如果ops回调函数中，.config()，.enable()或者.disable()操作会sleep，则要设置该变量。
*/


//通过将pwm的操作绑定在一个结构体上，交给应用层调用
/*
这些回调函数的操作对象是具体的pwm device（由struct pwm_device类型的指针表示），包括：
config，配置pwm device的频率、占空比。必须提供！
enable/disable，使能/禁止pwm信号输出。必须提供！
request/free，不再使用。
set_polarity，设置pwm信号的极性。可选，具体需要参考of_pwm_n_cells的定义。
*/
static const struct pwm_ops pwm_samsung_ops = {
	.request	= pwm_samsung_request,
	.free		= pwm_samsung_free,
	.enable		= pwm_samsung_enable,
	.disable	= pwm_samsung_disable,
	.config		= pwm_samsung_config,
	.set_polarity	= pwm_samsung_set_polarity,
	.owner		= THIS_MODULE,
};
//我们向export文件写入0时，就会调用这里的pwm_samsung_request函数
/*
接着上面的操作，继续执行命令： 
cd pwm0 
里面有七个文件： 
capture enable polarity uevent duty_cycle period power 
其中， 
enable：写入1使能pwm，写入0关闭pwm； 
polarity：有normal或inversed两个参数选择，表示TOUT_n输出引脚电平翻转； 
duty_cycle：在normal模式下，表示一个周期内高电平持续的时间（单位：纳秒），在reversed模式下，表示一个周期中低电平持续的时间（单位：纳秒）； 
period：表示pwm波的周期(单位：纳秒)； 
往里面写入，就是会调用到pwm_samsung_ops结构体里的成员函数。

所以可以这样使用pwm： 

echo 1000000000 > period 
echo 500000000 > duty_cycle 
echo normal > polarity 
echo 1 > enable 

这样就是占空比为500000000/1000000000的pwm出现。 
*/
//设备树里的设备节点:
pwm: pwm@139D0000 {
		compatible = "samsung,exynos4210-pwm";
		reg = <0x139D0000 0x1000>;
		interrupts = <GIC_SPI 37 IRQ_TYPE_LEVEL_HIGH>,
			     <GIC_SPI 38 IRQ_TYPE_LEVEL_HIGH>,
			     <GIC_SPI 39 IRQ_TYPE_LEVEL_HIGH>,
			     <GIC_SPI 40 IRQ_TYPE_LEVEL_HIGH>,
			     <GIC_SPI 41 IRQ_TYPE_LEVEL_HIGH>;
		clocks = <&clock CLK_PWM>;
		clock-names = "timers";
		#pwm-cells = <3>;
		status = "disabled";
};
&pwm {
	status = "okay";
	pinctrl-0 = <&pwm0_out>;
	pinctrl-names = "default";
	samsung,pwm-outputs = <0>;
};

//重启系统后，可以查看到:
~ # cd /sys/class/pwm/pwmchip0/                                                 
/sys/devices/platform/139d0000.pwm/pwm/pwmchip0 # ls
device     export     npwm       power      subsystem  uevent     unexport
/sys/devices/platform/139d0000.pwm/pwm/pwmchip0 # echo 0 > export               
/sys/devices/platform/139d0000.pwm/pwm/pwmchip0 # ls
device     npwm       pwm0       uevent
export     power      subsystem  unexport
/sys/devices/platform/139d0000.pwm/pwm/pwmchip0 # cd pwm0/                      
/sys/devices/platform/139d0000.pwm/pwm/pwmchip0/pwm0 # ls
capture     duty_cycle  enable      period      polarity    power       uevent
/sys/devices/platform/139d0000.pwm/pwm/pwmchip0/pwm0 # echo normal > polarity   
sh: write error: Invalid argument
/sys/devices/platform/139d0000.pwm/pwm/pwmchip0/pwm0 # echo 1000000000 > period                                                                           
/sys/devices/platform/139d0000.pwm/pwm/pwmchip0/pwm0 # echo 100000000 > duty_cycle                                                                              
/sys/devices/platform/139d0000.pwm/pwm/pwmchip0/pwm0 # echo 1 > enable          
/sys/devices/platform/139d0000.pwm/pwm/pwmchip0/pwm0 # echo 0 > enable 

/*
npwm的意思是Exynos4412支持的pwm通道个数（在驱动里将这个值设置为了5，
即将timer4也包含在内了）。蜂鸣器对应的的PWM0，所以我们向export中写入0，
然后就会在当前目录下产生了一个名为pwm0的新目录，其中是设置PWM0参数的配置文件。

period：表示pwm波的周期(单位：纳秒)；

duty_cycle：在normal模式下，表示一个周期内高电平持续的时间（单位：纳秒），
所以duty_cycle <= period；在reversed模式下，表示一个周期中低电平持续的时间（单位：纳秒）；

enable：向其中写入1表示启动pwm，写入0，表示关闭pwm；
实现一个频率为1Hz， 占空比为1:9的例子：
这里：1秒 = 1 000 000 000 纳秒
向unexport中写入0，pwm0目录会被自动删除。
*/

















