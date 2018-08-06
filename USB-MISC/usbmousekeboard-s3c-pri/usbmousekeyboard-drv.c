/*
1. USB是主从结构的
   所有的USB传输，都是从USB主机这方发起；USB设备没有"主动"通知USB主机的能力。
   例子：USB鼠标滑动一下立刻产生数据，但是它没有能力通知PC机来读数据，只能被动地等得PC机来读。

2. USB的传输类型:
a. 控制传输：可靠，时间有保证，比如：USB设备的识别过程
b. 批量传输: 可靠, 时间没有保证, 比如：U盘
c. 中断传输：可靠，实时，比如：USB鼠标
d. 实时传输：不可靠，实时，比如：USB摄像头

3. USB传输的对象：端点(endpoint)
   我们说"读U盘"、"写U盘"，可以细化为：把数据写到U盘的端点1，从U盘的端点2里读出数据
   除了端点0外，每一个端点只支持一个方向的数据传输
   端点0用于控制传输，既能输出也能输入
   
4. 每一个端点都有传输类型，传输方向

5. 术语里、程序里说的输入(IN)、输出(OUT) "都是" 基于USB主机的立场说的。
   比如鼠标的数据是从鼠标传到PC机, 对应的端点称为"输入端点"据，只能被动地等得PC机来读。

2. USB的传输类型:
a. 控制传输：可靠，时间有保证，比如：USB设备的识别过程
b. 批量传输: 可靠, 时间没有保证, 比如：U盘
c. 中断传输：可靠，实时，比如：USB鼠标
d. 实时传输：不可靠，实时，比如：USB摄像头

3. USB传输的对象：端点(endpoint)
   我们说"读U盘"、"写U盘"，可以细化为：把数据写到U盘的端点1，从U盘的端点2里读出数据
   除了端点0外，每一个端点只支持一个方向的数据传输
   端点0用于控制传输，既能输出也能输入
   
4. 每一个端点都有传输类型，传输方向

5. 术语里、程序里说的输入(IN)、输出(OUT) "都是" 基于USB主机的立场说的。
   比如鼠标的数据是从鼠标传到PC机, 对应的端点称为"输入端点"



APP:
--------------------------------------
设备驱动------------它知道数据含义
--------------------------------------
								|1、识别设备
								|
								|
USB总线驱动程序 |2、找到并安装对应的设备驱动
								|
								|
								|3、提供USB读写函数，不了解数据含义
--------------------------------------
硬件：USB主机控制器
					||
					||
				USB设备
				
				---------usb_bus_type-------
				|														|
				|usb_new_device							|usb_register
				|														|
		usb_interface										usb_driver
																			.id_table
																			.probe
																			.disconnects

怎么写USB设备驱动程序？
1. 分配/设置usb_driver结构体
        .id_table
        .probe
        .disconnect
2. 注册				

1. make menuconfig去掉原来的USB鼠标驱动
-> Device Drivers 
  -> HID Devices
  <> USB Human Interface Device (full HID) support 
  
*/

/*
 * drivers\hid\usbhid\usbmouse.c
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb/input.h>
#include <linux/hid.h>

static struct input_dev *uk_dev;
static char *usb_buf;
static dma_addr_t usb_buf_phys;
static int len;
static struct urb *uk_urb;

static struct usb_device_id usbmouse_as_key_id_table [] = {
	{ USB_INTERFACE_INFO(USB_INTERFACE_CLASS_HID, USB_INTERFACE_SUBCLASS_BOOT,
		USB_INTERFACE_PROTOCOL_MOUSE) },
	//{USB_DEVICE(0x1234,0x5678)},
	{ }	/* Terminating entry */
};
/*回调函数*/
static void usbmouse_as_key_irq(struct urb *urb)
{
	static unsigned char pre_val;
	
	#if 0	
	int i;
	static int cnt = 0;
	printk("data cnt %d: ", ++cnt);
	for (i = 0; i < len; i++)
	{
		printk("%02x ", usb_buf[i]);
	}
	printk("\n");
#endif
	
	
	/* USB鼠标数据含义
	 * data[0]: bit0-左键, 1-按下, 0-松开
	 *          bit1-右键, 1-按下, 0-松开
	 *          bit2-中键, 1-按下, 0-松开 
	 *
     */
	if ((pre_val & (1<<0)) != (usb_buf[0] & (1<<0)))
	{
		/* 左键发生了变化 */
		input_event(uk_dev, EV_KEY, KEY_L, (usb_buf[0] & (1<<0)) ? 1 : 0);
		input_sync(uk_dev);
	}

	if ((pre_val & (1<<1)) != (usb_buf[0] & (1<<1)))
	{
		/* 右键发生了变化 */
		input_event(uk_dev, EV_KEY, KEY_S, (usb_buf[0] & (1<<1)) ? 1 : 0);
		input_sync(uk_dev);
	}

	if ((pre_val & (1<<2)) != (usb_buf[0] & (1<<2)))
	{
		/* 中键发生了变化 */
		input_event(uk_dev, EV_KEY, KEY_ENTER, (usb_buf[0] & (1<<2)) ? 1 : 0);
		input_sync(uk_dev);
	}
	
	pre_val = usb_buf[0];

	/* 重新提交urb */
	usb_submit_urb(uk_urb, GFP_KERNEL);
	
}


static int usbmouse_as_key_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *dev = interface_to_usbdev(intf);//根据usb_interface指针intf获取usb_device的地址。
	struct usb_host_interface *interface;
	struct usb_endpoint_descriptor *endpoint;//端口描述符
	
	interface = intf->cur_altsetting;
	endpoint = &interface->endpoint[0].desc;
	
	/* a. 分配一个input_dev */
	uk_dev = input_allocate_device();
	
	/* b. 设置 */
	/* b.1 能产生哪类事件 */
	set_bit(EV_KEY, uk_dev->evbit);
	set_bit(EV_REP, uk_dev->evbit);
	
	/* b.2 能产生哪些事件 */
	set_bit(KEY_L, uk_dev->keybit);
	set_bit(KEY_S, uk_dev->keybit);
	set_bit(KEY_ENTER, uk_dev->keybit);
	
	/* c. 注册 */
	input_register_device(uk_dev);
	
	
	/* d. 硬件相关操作 */
	/* 数据传输3要素: 源,目的,长度 */
	/* 源: USB设备的某个端点 */
	pipe = usb_rcvintpipe(dev, endpoint->bEndpointAddress);//建立中断输入端点

	/* 长度: */
	len = endpoint->wMaxPacketSize;

	/* 目的: */
	usb_buf = usb_buffer_alloc(dev, len, GFP_ATOMIC, &usb_buf_phys);

	/* 使用"3要素" */
	/* 分配usb request block */
	uk_urb = usb_alloc_urb(0, GFP_KERNEL);
	/* 使用"3要素设置urb" */
	//对于中断urb，使用usb_fill_int_urb函数来初始化
	/*
	static inline void usb_fill_int_urb (struct urb *urb,要初始化的urb指针。
                     struct usb_device *dev,所要访问的设备
                     unsigned int      pipe,要访问的端点所对应的管道，使用usb_sndintpipe()或usb_rcvintpipe()创建
                     void              *transfer_buffer,要传输的数据缓冲区
                     int               buffer_length,缓冲区长度
                     usb_complete_t    complete_fn,当完成该urb所请求的操作时，要调用的回调函数
                     void              *context,complet_fn函数所需的上下文，通常取值为dev
                     int               interval)urb被调度的时间间隔
	*/
	usb_fill_int_urb(uk_urb, dev, pipe, usb_buf, len, usbmouse_as_key_irq, NULL, endpoint->bInterval);
	uk_urb->transfer_dma = usb_buf_phys;
	uk_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	/* 使用URB */
	usb_submit_urb(uk_urb, GFP_KERNEL);
	
	return 0;
	
}

static void usbmouse_as_key_disconnect(struct usb_interface *intf)
{
	struct usb_device *dev = interface_to_usbdev(intf);

	//printk("disconnect usbmouse!\n");
	usb_kill_urb(uk_urb);
	usb_free_urb(uk_urb);

	usb_buffer_free(dev, len, usb_buf, usb_buf_phys);
	input_unregister_device(uk_dev);
	input_free_device(uk_dev);
}


/* 1. 分配/设置usb_driver */
static struct usb_driver usbmouse_as_key_driver = {
	.name		= "usbmouse_as_key_",
	.probe		= usbmouse_as_key_probe,
	.disconnect	= usbmouse_as_key_disconnect,
	.id_table	= usbmouse_as_key_id_table,
};


static int usbmouse_as_key_init(void)
{
	/* 2. 注册 */
	usb_register(&usbmouse_as_key_driver);
	return 0;
}

static void usbmouse_as_key_exit(void)
{
	usb_deregister(&usbmouse_as_key_driver);	
}

module_init(usbmouse_as_key_init);
module_exit(usbmouse_as_key_exit);

MODULE_LICENSE("GPL");


























