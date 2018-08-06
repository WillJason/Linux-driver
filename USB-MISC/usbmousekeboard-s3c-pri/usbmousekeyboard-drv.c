/*
1. USB�����ӽṹ��
   ���е�USB���䣬���Ǵ�USB�����ⷽ����USB�豸û��"����"֪ͨUSB������������
   ���ӣ�USB��껬��һ�����̲������ݣ�������û������֪ͨPC���������ݣ�ֻ�ܱ����صȵ�PC��������

2. USB�Ĵ�������:
a. ���ƴ��䣺�ɿ���ʱ���б�֤�����磺USB�豸��ʶ�����
b. ��������: �ɿ�, ʱ��û�б�֤, ���磺U��
c. �жϴ��䣺�ɿ���ʵʱ�����磺USB���
d. ʵʱ���䣺���ɿ���ʵʱ�����磺USB����ͷ

3. USB����Ķ��󣺶˵�(endpoint)
   ����˵"��U��"��"дU��"������ϸ��Ϊ��������д��U�̵Ķ˵�1����U�̵Ķ˵�2���������
   ���˶˵�0�⣬ÿһ���˵�ֻ֧��һ����������ݴ���
   �˵�0���ڿ��ƴ��䣬�������Ҳ������
   
4. ÿһ���˵㶼�д������ͣ����䷽��

5. �����������˵������(IN)�����(OUT) "����" ����USB����������˵�ġ�
   �������������Ǵ���괫��PC��, ��Ӧ�Ķ˵��Ϊ"����˵�"�ݣ�ֻ�ܱ����صȵ�PC��������

2. USB�Ĵ�������:
a. ���ƴ��䣺�ɿ���ʱ���б�֤�����磺USB�豸��ʶ�����
b. ��������: �ɿ�, ʱ��û�б�֤, ���磺U��
c. �жϴ��䣺�ɿ���ʵʱ�����磺USB���
d. ʵʱ���䣺���ɿ���ʵʱ�����磺USB����ͷ

3. USB����Ķ��󣺶˵�(endpoint)
   ����˵"��U��"��"дU��"������ϸ��Ϊ��������д��U�̵Ķ˵�1����U�̵Ķ˵�2���������
   ���˶˵�0�⣬ÿһ���˵�ֻ֧��һ����������ݴ���
   �˵�0���ڿ��ƴ��䣬�������Ҳ������
   
4. ÿһ���˵㶼�д������ͣ����䷽��

5. �����������˵������(IN)�����(OUT) "����" ����USB����������˵�ġ�
   �������������Ǵ���괫��PC��, ��Ӧ�Ķ˵��Ϊ"����˵�"



APP:
--------------------------------------
�豸����------------��֪�����ݺ���
--------------------------------------
								|1��ʶ���豸
								|
								|
USB������������ |2���ҵ�����װ��Ӧ���豸����
								|
								|
								|3���ṩUSB��д���������˽����ݺ���
--------------------------------------
Ӳ����USB����������
					||
					||
				USB�豸
				
				---------usb_bus_type-------
				|														|
				|usb_new_device							|usb_register
				|														|
		usb_interface										usb_driver
																			.id_table
																			.probe
																			.disconnects

��ôдUSB�豸��������
1. ����/����usb_driver�ṹ��
        .id_table
        .probe
        .disconnect
2. ע��				

1. make menuconfigȥ��ԭ����USB�������
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
/*�ص�����*/
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
	
	
	/* USB������ݺ���
	 * data[0]: bit0-���, 1-����, 0-�ɿ�
	 *          bit1-�Ҽ�, 1-����, 0-�ɿ�
	 *          bit2-�м�, 1-����, 0-�ɿ� 
	 *
     */
	if ((pre_val & (1<<0)) != (usb_buf[0] & (1<<0)))
	{
		/* ��������˱仯 */
		input_event(uk_dev, EV_KEY, KEY_L, (usb_buf[0] & (1<<0)) ? 1 : 0);
		input_sync(uk_dev);
	}

	if ((pre_val & (1<<1)) != (usb_buf[0] & (1<<1)))
	{
		/* �Ҽ������˱仯 */
		input_event(uk_dev, EV_KEY, KEY_S, (usb_buf[0] & (1<<1)) ? 1 : 0);
		input_sync(uk_dev);
	}

	if ((pre_val & (1<<2)) != (usb_buf[0] & (1<<2)))
	{
		/* �м������˱仯 */
		input_event(uk_dev, EV_KEY, KEY_ENTER, (usb_buf[0] & (1<<2)) ? 1 : 0);
		input_sync(uk_dev);
	}
	
	pre_val = usb_buf[0];

	/* �����ύurb */
	usb_submit_urb(uk_urb, GFP_KERNEL);
	
}


static int usbmouse_as_key_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *dev = interface_to_usbdev(intf);//����usb_interfaceָ��intf��ȡusb_device�ĵ�ַ��
	struct usb_host_interface *interface;
	struct usb_endpoint_descriptor *endpoint;//�˿�������
	
	interface = intf->cur_altsetting;
	endpoint = &interface->endpoint[0].desc;
	
	/* a. ����һ��input_dev */
	uk_dev = input_allocate_device();
	
	/* b. ���� */
	/* b.1 �ܲ��������¼� */
	set_bit(EV_KEY, uk_dev->evbit);
	set_bit(EV_REP, uk_dev->evbit);
	
	/* b.2 �ܲ�����Щ�¼� */
	set_bit(KEY_L, uk_dev->keybit);
	set_bit(KEY_S, uk_dev->keybit);
	set_bit(KEY_ENTER, uk_dev->keybit);
	
	/* c. ע�� */
	input_register_device(uk_dev);
	
	
	/* d. Ӳ����ز��� */
	/* ���ݴ���3Ҫ��: Դ,Ŀ��,���� */
	/* Դ: USB�豸��ĳ���˵� */
	pipe = usb_rcvintpipe(dev, endpoint->bEndpointAddress);//�����ж�����˵�

	/* ����: */
	len = endpoint->wMaxPacketSize;

	/* Ŀ��: */
	usb_buf = usb_buffer_alloc(dev, len, GFP_ATOMIC, &usb_buf_phys);

	/* ʹ��"3Ҫ��" */
	/* ����usb request block */
	uk_urb = usb_alloc_urb(0, GFP_KERNEL);
	/* ʹ��"3Ҫ������urb" */
	//�����ж�urb��ʹ��usb_fill_int_urb��������ʼ��
	/*
	static inline void usb_fill_int_urb (struct urb *urb,Ҫ��ʼ����urbָ�롣
                     struct usb_device *dev,��Ҫ���ʵ��豸
                     unsigned int      pipe,Ҫ���ʵĶ˵�����Ӧ�Ĺܵ���ʹ��usb_sndintpipe()��usb_rcvintpipe()����
                     void              *transfer_buffer,Ҫ��������ݻ�����
                     int               buffer_length,����������
                     usb_complete_t    complete_fn,����ɸ�urb������Ĳ���ʱ��Ҫ���õĻص�����
                     void              *context,complet_fn��������������ģ�ͨ��ȡֵΪdev
                     int               interval)urb�����ȵ�ʱ����
	*/
	usb_fill_int_urb(uk_urb, dev, pipe, usb_buf, len, usbmouse_as_key_irq, NULL, endpoint->bInterval);
	uk_urb->transfer_dma = usb_buf_phys;
	uk_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	/* ʹ��URB */
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


/* 1. ����/����usb_driver */
static struct usb_driver usbmouse_as_key_driver = {
	.name		= "usbmouse_as_key_",
	.probe		= usbmouse_as_key_probe,
	.disconnect	= usbmouse_as_key_disconnect,
	.id_table	= usbmouse_as_key_id_table,
};


static int usbmouse_as_key_init(void)
{
	/* 2. ע�� */
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


























