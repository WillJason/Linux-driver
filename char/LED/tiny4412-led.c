/*
项目：设备树之LED点灯
硬件平台：tiny4412
系统：linux-4.4
文件系统：busybox-1.25
编译器： arm-none-linux-gnueabi-gcc（gcc version 4.8.3 20140320）
uboot：友善自带uboot
devicetree:
*/

/*设备树文件*/
/* 
 * FriendlyARM's Exynos4412 based TINY4412 board device tree source 
 * 
 * Copyright (c) 2013 Alex Ling <kasimling@gmail.com> 
 * 
 * Device tree source file for FriendlyARM's TINY4412 board which is based on 
 * Samsung's Exynos4412 SoC. 
 * 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation. 
*/  
  
/dts-v1/;  
#include "exynos4412.dtsi"  
#include <dt-bindings/gpio/gpio.h>  
#include <dt-bindings/usb4640/usb4640.h>  
  
/ {  
    model = "FriendlyARM TINY4412 board based on Exynos4412";  
    compatible = "friendlyarm,tiny4412", "samsung,exynos4412", "samsung,exynos4";  
  
    chosen {  
        stdout-path = &serial_0;  
        bootargs = "root=/dev/ram0 rw rootfstype=ext4 console=ttySAC0,115200 ethmac=1C:6F:65:34:51:7E init=/linuxrc";  
    };  
  
    memory {  
        reg = <0x40000000 0x40000000>;  
    };  
  
    leds {  
        compatible = "gpio-leds";  
    status = "disabled";  
        led1 {  
            label = "led1";  
            gpios = <&gpm4 0 GPIO_ACTIVE_LOW>;  
            default-state = "off";  
            linux,default-trigger = "heartbeat";  
        };  
  
        led2 {  
            label = "led2";  
            gpios = <&gpm4 1 GPIO_ACTIVE_LOW>;  
            default-state = "off";  
        };  
  
        led3 {  
            label = "led3";  
            gpios = <&gpm4 2 GPIO_ACTIVE_LOW>;  
            default-state = "off";  
        };  
  
        led4 {  
            label = "led4";  
            gpios = <&gpm4 3 GPIO_ACTIVE_LOW>;  
            default-state = "off";  
            linux,default-trigger = "mmc0";  
        };  
    };  
  
    fixed-rate-clocks {  
        xxti {  
            compatible = "samsung,clock-xxti";  
            clock-frequency = <0>;  
        };  
  
        xusbxti {  
            compatible = "samsung,clock-xusbxti";  
            clock-frequency = <24000000>;  
        };  
    };  
        usb-hub {  
        compatible = "smsc,usb4640";  
        reset-gpios = <&gpm2 4 GPIO_ACTIVE_LOW>;  
        initial-mode = <USB4640_MODE_HUB>;  
    };  
      
    interrupt_demo: interrupt_demo {  
            compatible         = "tiny4412,interrupt_demo";  
            tiny4412,int_gpio1 = <&gpx3 2 GPIO_ACTIVE_HIGH>;  
            tiny4412,int_gpio2 = <&gpx3 3 GPIO_ACTIVE_HIGH>;  
            tiny4412,int_gpio3 = <&gpx3 4 GPIO_ACTIVE_HIGH>;  
            tiny4412,int_gpio4 = <&gpx3 5 GPIO_ACTIVE_HIGH>;  
    };  
    led_pin {  
    compatible         = "tiny4412,led_demo";  
    pinctrl-names = "led_demo";  
    pinctrl-0 = <&led_demo>;  
    tiny4412,int_gpio1 = <&gpm4 0 GPIO_ACTIVE_HIGH>;  
    tiny4412,int_gpio2 = <&gpm4 1 GPIO_ACTIVE_HIGH>;  
    tiny4412,int_gpio3 = <&gpm4 2 GPIO_ACTIVE_HIGH>;  
    tiny4412,int_gpio4 = <&gpm4 3 GPIO_ACTIVE_HIGH>;  
        };  
  
  
};  
&pinctrl_1 {  
    led_demo: led{  
            samsung,pins = "gpm4-0", "gpm4-1" ,"gpm4-2", "gpm4-3";  
            samsung,pin-function = <0x1>;   //1为输出  
            samsung,pin-pud = <0x0>;        //没有上拉  
            samsung,pin-drv = <0x0>;        //驱动强度？  
    };  
};   
  
&rtc {  
    status = "okay";  
};  
  
&sdhci_2 {  
    bus-width = <4>;  
    pinctrl-0 = <&sd2_clk &sd2_cmd &sd2_cd &sd2_bus4>;  
    pinctrl-names = "default";  
    #status = "okay";     
    status = "disabled";  
};  
  
&serial_0 {  
    status = "okay";  
};  
  
&serial_1 {  
    status = "okay";  
};  
  
&serial_2 {  
    status = "okay";  
};  
  
&serial_3 {  
    status = "okay";  
};  
  
&exynos_usbphy {  
    status = "okay";  
};  
  
&ehci {  
    status = "okay";  
    port@0 {  
        status = "okay";  
    };  
    port@1 {  
        status = "okay";  
    };  
    port@2 {  
        status = "okay";  
    };  
};  
  
&ohci {  
    status = "okay";  
    port@0 {  
        status = "okay";  
    };  
};  
  
&hsotg {  
    status = "okay";  
};  


/*驱动*/
[cpp] view plain copy
#include <linux/module.h>  
#include <linux/kernel.h>  
#include <linux/cdev.h>  
#include <linux/device.h>  
#include <linux/platform_device.h>  
#include <linux/gpio.h>  
#include <linux/of.h>  
#include <linux/of_gpio.h>  
#include <linux/fs.h>  
#include <asm/uaccess.h>  
  
#define LED_CNT   4  
  
static int  major;  
static struct cdev  led_cdev;   //内核中用cdev描述一个字符设备  
static struct class *cls;  
static int led1,led2,led3,led4;  
  
static ssize_t led_write(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)  
{  
    char buf;  
    int minor = iminor(file->f_inode);  
  
    printk("minor is %d\n",minor);  
    printk("%s\n",__func__);  
    if(count != 1){  
        printk("count != 1\n");   
        return 1;  
    }  
    if (copy_from_user(&buf, user_buf, count))  
        return -EFAULT;  
  
    printk("rcv %d\n",buf);  
    if(buf == 0x01)  
    {  
        switch(minor){  
        case 0:  
            gpio_set_value(led1, 0);  
            break;  
        case 1:  
            gpio_set_value(led2, 0);  
            break;  
        case 2:  
            gpio_set_value(led3, 0);  
            break;  
        case 3:  
            gpio_set_value(led4, 0);  
            break;  
        default:  
            printk("%s rcv minor error\n",__func__);  
        }                         
    }  
    else if(buf == 0x0)  
    {  
        switch(minor){  
        case 0:  
            gpio_set_value(led1, 1);  
            break;  
        case 1:  
            gpio_set_value(led2, 1);  
            break;  
        case 2:  
            gpio_set_value(led3, 1);  
            break;  
        case 3:  
            gpio_set_value(led4, 1);  
            break;  
        default:  
            printk("%s rcv minor error\n",__func__);  
        }         
    }  
    return 1;  
}  
static int led_open(struct inode *inode, struct file *file)  
{  
    printk("led_open\n");  
    return 0;  
}  
  
static struct file_operations led_fops = {  
    .owner = THIS_MODULE,  
    .open  = led_open,  
    .write = led_write,  
};  
  
static int led_probe(struct platform_device *pdev) {  
  
    struct device *dev = &pdev->dev;  
    dev_t devid;  
    struct pinctrl *pctrl;  
    struct pinctrl_state *pstate;  
    /*获取一个pinctrl句柄，参数是dev是包含这个pin的device结构体即xxx这个设备的device
		获取设备操作句柄（设备模型中的struct device）的pin control state holder（struct pinctrl）*/
    pctrl = devm_pinctrl_get(dev);  
    if(pctrl == NULL)  
    {  
        printk("devm_pinctrl_get error\n");  
    }  
    /*获取这个pin对应pin_state（引脚状态-turnon_tes/turnoff_tes）s*/
    pstate = pinctrl_lookup_state(pctrl, "led_demo");  
    if(pstate == NULL)  
    {  
        printk("pinctrl_lookup_state error\n");  
    }  
    /*设置引脚为为某个stata -- turnon_tes/turnoff_tes*/
    pinctrl_select_state(pctrl, pstate);//设置为输出模式   
    printk("enter %s\n",__func__);  
    /*得到GPIO的编号*/
    led1 = of_get_named_gpio(dev->of_node, "tiny4412,int_gpio1", 0);;
    /*of_get_named_gpio：此函数是解析设备树的函数，我们通过这个函数去解析设备树，
    tiny4412,int_gpio1 = <&gpm4 0 GPIO_ACTIVE_HIGH>; 跟踪下去会发现这个函数掉用
    了list = of_get_property(np, "tiny4412,int_gpio2", &size);设备树解析是创界了
    设备节点，现在通过这个函数去获取属性。*/  
    led2 = of_get_named_gpio(dev->of_node, "tiny4412,int_gpio2", 0);;  
    led3 = of_get_named_gpio(dev->of_node, "tiny4412,int_gpio3", 0);;  
    led4 = of_get_named_gpio(dev->of_node, "tiny4412,int_gpio4", 0);;  
    if(led1 <= 0)  
    {  
        printk("%s error\n",__func__);  
        return -EINVAL;  
    }  
    else  
    {  
        printk("led1 %d\n",led1);  
        printk("led2 %d\n",led2);  
        printk("led3 %d\n",led3);  
        printk("led4 %d\n",led4);  
        /*获取一个GPIO并初始化属性*/
        devm_gpio_request_one(dev, led1, GPIOF_OUT_INIT_HIGH, "LED1");  
        devm_gpio_request_one(dev, led2, GPIOF_OUT_INIT_HIGH, "LED2");  
        devm_gpio_request_one(dev, led3, GPIOF_OUT_INIT_HIGH, "LED3");  
        devm_gpio_request_one(dev, led4, GPIOF_OUT_INIT_HIGH, "LED4");  
    }  
  
    if(alloc_chrdev_region(&devid, 0, LED_CNT, "led") < 0)/* (major,0~1) 对应 hello_fops, (major, 2~255)都不对应hello_fops */  
    {  
        printk("%s ERROR\n",__func__);  
        goto error;  
    }  
    major = MAJOR(devid);                       
  
    cdev_init(&led_cdev, &led_fops);        //绑定文件操作函数  
    cdev_add(&led_cdev, devid, LED_CNT);    //注册到内核  
  
    cls = class_create(THIS_MODULE, "led"); //创建led类,向类中添加设备,mdev会帮我们创建设备节点  
    device_create(cls, NULL, MKDEV(major, 0), NULL, "led0");   
    device_create(cls, NULL, MKDEV(major, 1), NULL, "led1");   
    device_create(cls, NULL, MKDEV(major, 2), NULL, "led2");   
    device_create(cls, NULL, MKDEV(major, 3), NULL, "led3");   
    //return 0;  
error:  
    unregister_chrdev_region(MKDEV(major, 0), LED_CNT);  
    return 0;  
}  
  
static int led_remove(struct platform_device *pdev) {  
  
    printk("enter %s\n",__func__);  
    device_destroy(cls, MKDEV(major, 0));  
    device_destroy(cls, MKDEV(major, 1));  
    device_destroy(cls, MKDEV(major, 2));  
    device_destroy(cls, MKDEV(major, 3));  
    class_destroy(cls);  
  
    cdev_del(&led_cdev);  
    unregister_chrdev_region(MKDEV(major, 0), LED_CNT);  
  
    printk("%s enter.\n", __func__);  
    return 0;  
}  
  
static const struct of_device_id led_dt_ids[] = {  
    { .compatible = "tiny4412,led_demo", },  
    {},  
};  
  
MODULE_DEVICE_TABLE(of, led_dt_ids);  
  
static struct platform_driver led_driver = {  
    .driver        = {  
        .name      = "led_demo",  
        .of_match_table    = of_match_ptr(led_dt_ids),  
    },  
    .probe         = led_probe,  
    .remove        = led_remove,  
};  
  
static int led_init(void){  
    int ret;  
    printk("enter %s\n",__func__);  
    ret = platform_driver_register(&led_driver);  
    if (ret)  
        printk(KERN_ERR "led demo: probe failed: %d\n", ret);  
  
    return ret;   
}  
  
static void led_exit(void)  
{  
    printk("enter %s\n",__func__);  
    platform_driver_unregister(&led_driver);  
}  
  
module_init(led_init);  
module_exit(led_exit);  
  
MODULE_LICENSE("GPL"); 
/*设备树思路是：uboot启动时将设备树地址传给内核，内核解析设备树，并创建设备，初始化相关属性
，驱动中通过of_get_XXX函数去获取设备树加载时创建的设备。想要知道of函数做了什么，就去追踪这个
函数最后调用了什么，同时也就知道了内核解析设备树的时候为我们创建了什么。*/

/*测试应用程序*/
#include <sys/types.h>  
#include <sys/stat.h>  
#include <fcntl.h>  
#include <stdio.h>  
#include <errno.h>   
/* firstdrvtest on 
  * firstdrvtest off 
  */  
int main(int argc, char **argv)  
{  
    int fd;  
    int val = 1;  
    fd = open("/dev/led1", O_RDWR);  
    if (fd < 0)  
    {  
        printf("can't open!\n");  
        printf("strerror:%s\n", strerror(errno));    
    }  
    if (argc != 2)  
    {  
        printf("Usage :\n");  
        printf("%s <on|off>\n", argv[0]);  
        return 0;  
    }  
  
    if (strcmp(argv[1], "on") == 0)  
    {  
        val  = 1;  
    }  
    else  
    {  
        val = 0;  
    }  
      
    write(fd, &val, 1);  
    return 0;  
}  

/*烧写测试*/
#u-boot：
setenv bootargs  'root=/dev/nfs  rw  nfsroot=192.168.1.123:/work/nfs/rootfs_for_tiny4412/rootfs ethmac=1C:6F:65:34:51:7E  ip=192.168.1.125:192.168.1.123:192.168.1.1:255.255.255.0::eth0:off console=ttySAC0,115200  init=/linuxrc'
#u-boot：save
#u-boot：dnw 0x40600000
dnw arch/arm/boot/uImage
#u-boot：dnw 0x42000000
dnw  arch/arm/boot/dts/exynos4412-tiny4412.dtb
bootm 0x40600000 - 0x42000000









