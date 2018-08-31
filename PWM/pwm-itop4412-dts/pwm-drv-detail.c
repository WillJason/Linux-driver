/*
Ӳ��ƽ̨��itop4412
ϵͳ��linux-4.14.2

Exynos4412һ���������4·PWM��timer0��timer1��timer2��timer3������PWM�ж�Ӧ��������ţ�
timer4û�ж�Ӧ�����ţ���������tiny4412��ֻ����·PWM������Ƭ������ʹ�ã��ֱ�����timer0��timer1��
����timer0�����PWM0���ڿ��Ƶװ��ϵ���Դ��������timer1������PWM1���ڿ���LCD�ı������ȡ�

*/
//��Ӧ���豸����Դ�ļ�: drivers/pwm/pwm-samsung.c
//pwm-samsung.cԴ�ļ����豸����ص���Ҫ����:
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
//ƥ���Ϻ� �豸������pwm_samsung_probe�����ͻᱻ��������. ��probe������ͻ�ȡ���豸�����ṩ��Ӳ����Դ.
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
	chip->chip.ops = &pwm_samsung_ops;//�豸��������
	chip->chip.base = -1;
	chip->chip.npwm = SAMSUNG_PWM_NUM;
	chip->inverter_mask = BIT(SAMSUNG_PWM_NUM) - 1;

	if (IS_ENABLED(CONFIG_OF) && pdev->dev.of_node) {
		
		////��ȡ�豸ID��ṹ�Լ�ƽ̨�豸��Դ
		ret = pwm_samsung_parse_dt(chip);//��Ϊ����ʹ�õ����豸��������ʹ�������֧�����øú���
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

	/* ����˽������ */
	platform_set_drvdata(pdev, chip);

	//��ʼ����ɺ��pwm chip����ͨ��pwmchip_add�ӿ�ע�ᵽkernel�У�֮������飬pwm driver�Ͳ��ò����ˡ�
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
PWM frameworkʹ��struct pwm_chip����PWM��������ͨ������£���һ��SOC�У�����ͬʱ֧�ֶ�·
PWM�������6·�����Ա�ͬʱ���ƶ��PWM�豸������ÿһ·PWM��������Կ���һ��PWM�豸��������
struct pwm_device���󣩣�û������Ļ�����ЩPWM�豸�Ŀ��Ʒ�ʽӦ�����ơ�PWM framework��ͳһ����
��ЩPWM�豸�������ǹ���Ϊһ��PWM chip��������������pwm_chip���ϲ��ַ�װ��һ���ṹ��samsung_pwm_chip��
samsung_pwm_chip�Ķ������£�
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
����struct pwm_chip�Ķ������£�
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
dev����pwm chip��Ӧ���豸��һ����pwm driver��Ӧ��platform����ָ���������ṩ��
ops������PWM�豸�Ļص��������������ϸ���ܡ������ṩ��
npwm����pwm chip����֧�ֵ�pwm channel��Ҳ���Գ���pwm device��struct pwm_device��ʾ��������kernel����ݸ�number��������Ӧ������struct pwm_device�ṹ��������pwmsָ���С������ṩ��
pwms����������pwm device�����飬kernel�����з��䣬����Ҫdriver���ġ�
base���ڽ���chip������pwm device���radix treeʱʹ�ã�ֻ�оɵ�pwm_request�ӿڻ�ʹ�ã���˺������ɣ���дpwm driver����Ҫ���ġ�
of_pwm_n_cells����PWM chip���ṩ��DTS node��cell��һ����2����3�����磺Ϊ3ʱ��consumer��Ҫ��DTSָ��pwm number��pwm period��pwm flag������Ϣ����2.1�еĽ��ܣ���Ϊ2ʱ��û��flag��Ϣ��
of_xlate�����ڽ���consumer��ָ���ġ�pwm��Ϣ��DTS node�Ļص���������2.1�н��ܵģ�pwms = <&pwm 0 5000000 PWM_POLARITY_INVERTED>����
ע2��һ������£�of_pwm_n_cellsȡֵΪ3������2�������ļ��ԣ���of_xlate�����ʹ��kernel�ṩ��of_pwm_xlate_with_flags������of_pwm_n_cellsΪ3��chip������of_pwm_simple_xlate������of_pwm_n_cellsΪ2��������������driver���Ը���ʵ������޸��������򣬵������򲻵��ѵ�ʱ�򣬲�Ҫ�����ַǱ�׼�ġ��������ֺõ����飡���й�of_xlate�����̣�������һƪ���̷����������н��ܡ���
can_sleep�����ops�ص������У�.config()��.enable()����.disable()������sleep����Ҫ���øñ�����
*/


//ͨ����pwm�Ĳ�������һ���ṹ���ϣ�����Ӧ�ò����
/*
��Щ�ص������Ĳ��������Ǿ����pwm device����struct pwm_device���͵�ָ���ʾ����������
config������pwm device��Ƶ�ʡ�ռ�ձȡ������ṩ��
enable/disable��ʹ��/��ֹpwm�ź�����������ṩ��
request/free������ʹ�á�
set_polarity������pwm�źŵļ��ԡ���ѡ��������Ҫ�ο�of_pwm_n_cells�Ķ��塣
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
//������export�ļ�д��0ʱ���ͻ���������pwm_samsung_request����
/*
��������Ĳ���������ִ����� 
cd pwm0 
�������߸��ļ��� 
capture enable polarity uevent duty_cycle period power 
���У� 
enable��д��1ʹ��pwm��д��0�ر�pwm�� 
polarity����normal��inversed��������ѡ�񣬱�ʾTOUT_n������ŵ�ƽ��ת�� 
duty_cycle����normalģʽ�£���ʾһ�������ڸߵ�ƽ������ʱ�䣨��λ�����룩����reversedģʽ�£���ʾһ�������е͵�ƽ������ʱ�䣨��λ�����룩�� 
period����ʾpwm��������(��λ������)�� 
������д�룬���ǻ���õ�pwm_samsung_ops�ṹ����ĳ�Ա������

���Կ�������ʹ��pwm�� 

echo 1000000000 > period 
echo 500000000 > duty_cycle 
echo normal > polarity 
echo 1 > enable 

��������ռ�ձ�Ϊ500000000/1000000000��pwm���֡� 
*/
//�豸������豸�ڵ�:
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

//����ϵͳ�󣬿��Բ鿴��:
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
npwm����˼��Exynos4412֧�ֵ�pwmͨ���������������ｫ���ֵ����Ϊ��5��
����timer4Ҳ���������ˣ�����������Ӧ�ĵ�PWM0������������export��д��0��
Ȼ��ͻ��ڵ�ǰĿ¼�²�����һ����Ϊpwm0����Ŀ¼������������PWM0�����������ļ���

period����ʾpwm��������(��λ������)��

duty_cycle����normalģʽ�£���ʾһ�������ڸߵ�ƽ������ʱ�䣨��λ�����룩��
����duty_cycle <= period����reversedģʽ�£���ʾһ�������е͵�ƽ������ʱ�䣨��λ�����룩��

enable��������д��1��ʾ����pwm��д��0����ʾ�ر�pwm��
ʵ��һ��Ƶ��Ϊ1Hz�� ռ�ձ�Ϊ1:9�����ӣ�
���1�� = 1 000 000 000 ����
��unexport��д��0��pwm0Ŀ¼�ᱻ�Զ�ɾ����
*/

















