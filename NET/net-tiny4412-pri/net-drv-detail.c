/*
��ôд������������
1. ����һ��net_device�ṹ��
2. ����:
2.1 ��������: hard_start_xmit
2.2 �յ�����ʱ(���жϴ�������)��netif_rx�ϱ�����
2.3 ��������
3. ע��: register_netdevice

tiny4412��Ҫ����������dm9620.c�����Ǿʹ���������ļ�������������������

ģ���ʼ��ʱע��/����usb�豸����������usb�豸���������󣬻����usb�豸
��probe����usbnet_probe����usbnet_probe�д��������豸��������net�豸��usb
�豸֮��Ĺ�������ʼ����������ɡ�
*/

/*��������ں����ͳ��ں���*/
static int __init dm9620_init(void)
{
	return usb_register(&dm9620_driver);
}

static void __exit dm9620_exit(void)
{
	usb_deregister(&dm9620_driver);
}

module_init(dm9620_init);
module_exit(dm9620_exit);
/*����usb_driver�ṹ��*/
static struct usb_driver dm9620_driver = {
	.name = "dm9620",
	.id_table = products,
	.probe = usbnet_probe,
	.disconnect = usbnet_disconnect,
	.suspend = usbnet_suspend,
	.resume = usbnet_resume,
};
static const struct usb_device_id products[] = {
	{
   USB_DEVICE(0x0a46, 0x9620),     /* Davicom 9620 */
   .driver_info = (unsigned long)&dm9620_info,
   },
   //1F.1010 0100 0110 (0A46)
     // .Original Default.
   //+
   //1268, 1200
   {},			// END
}
static const struct driver_info dm9620_info = {
	.description	= "Davicom DM9620 USB Ethernet",
	.flags		= FLAG_ETHER,
	.bind		= dm9620_bind,
	.rx_fixup	= dm9620_rx_fixup,
	.tx_fixup	= dm9620_tx_fixup,
	.status		= dm9620_status,
	.link_reset	= dm9620_link_reset,
	.reset		= dm9620_link_reset,
	.unbind     = dm9620_unbind,
};

/*probe��������*/
int usbnet_probe (struct usb_interface *udev, const struct usb_device_id *prod)
{
	struct usbnet			*dev;/* ˽�����ݽṹ����Ϊnet�豸��usb�豸֮��Ĺ�����*/
	struct net_device		*net;/* �����豸 */
	struct usb_host_interface	*interface;/* usb interface */
	struct driver_info		*info;
	struct usb_device		*xdev; /* usb�豸*/
	int				status;
	/* usb�豸��net�����豸�ǱȽ���Ҫ */
	const char			*name;
	struct usb_driver 	*driver = to_usb_driver(udev->dev.driver);

	/* usbnet already took usb runtime pm, so have to enable the feature
	 * for usb interface, otherwise usb_autopm_get_interface may return
	 * failure if USB_SUSPEND(RUNTIME_PM) is enabled.
	 */
	if (!driver->supports_autosuspend) {
		driver->supports_autosuspend = 1;
		pm_runtime_enable(&udev->dev);
	}

	name = udev->dev.driver->name;
	info = (struct driver_info *) prod->driver_info;//dm9620.c�ж����dm9620_info
	if (!info) {
		dev_dbg (&udev->dev, "blacklisted by %s\n", name);
		return -ENODEV;
	}
	xdev = interface_to_usbdev (udev);//����usb_interfaceָ��intf��ȡusb_device�ĵ�ַ��
	interface = udev->cur_altsetting;

	usb_get_dev (xdev);/*ͨ���豸�Ľӿڣ��������ӿ�����Ӧ���豸���������豸�����ô���*/

	status = -ENOMEM;

	// set up our own records
	/* ���������豸�������豸��˽���������������usbnet��Ϣ��ΪʲôҪ��ô����
	Ŀ����*�ڷ�������ʱ�ܹ��ҵ���ص�usb�豸��*/
	net = alloc_etherdev(sizeof(*dev));
	if (!net)
		goto out;

	/* netdev_printk() needs this so do it as early as possible */
	SET_NETDEV_DEV(net, &udev->dev);

	 /* ���������豸���� */
	dev = netdev_priv(net);
	dev->udev = xdev;
	dev->intf = udev;
	dev->driver_info = info;
	dev->driver_name = name;
	dev->msg_enable = netif_msg_init (msg_level, NETIF_MSG_DRV
				| NETIF_MSG_PROBE | NETIF_MSG_LINK);
	skb_queue_head_init (&dev->rxq);
	skb_queue_head_init (&dev->txq);
	skb_queue_head_init (&dev->done);
	skb_queue_head_init(&dev->rxq_pause);
	dev->bh.func = usbnet_bh;
	dev->bh.data = (unsigned long) dev;
	INIT_WORK (&dev->kevent, kevent);
	init_usb_anchor(&dev->deferred);
	dev->delay.function = usbnet_bh;
	dev->delay.data = (unsigned long) dev;
	init_timer (&dev->delay);
	mutex_init (&dev->phy_mutex);

	dev->net = net;/* usbnet�б����������豸 */
	strcpy (net->name, "usb%d"); /* �����豸���ƴ�ethxxx�ı�Ϊusbxxx*/
	memcpy (net->dev_addr, node_id, sizeof node_id);/* ��������mac��ַ */

	/* rx and tx sides can use different message sizes;
	 * bind() should set rx_urb_size in that case.
	 * ���պͷ���������Ϣ��С�����ǲ�ͬ�ġ�
   * bind() ʱӦ������ rx_urb_size
   *
	*/
	dev->hard_mtu = net->mtu + net->hard_header_len;
#if 0
// dma_supported() is deeply broken on almost all architectures
	// possible with some EHCI controllers
	if (dma_supported (&udev->dev, DMA_BIT_MASK(64)))
		net->features |= NETIF_F_HIGHDMA;
#endif

	net->netdev_ops = &usbnet_netdev_ops;/*�漰��dm9000�ײ�Ĳ�������*/
	net->watchdog_timeo = TX_TIMEOUT_JIFFIES;
	net->ethtool_ops = &usbnet_ethtool_ops;/* ethtoolʹ�õĽӿ� */

	// allow device-specific bind/init procedures
	// NOTE net->name still not usable ...
	/*/* ����usbnet������net��usb�豸����ϵ�ӿ� */
	if (info->bind) {
		// ���ò�ͬUSB��������ע���bind��������Ҫ�����ǣ���ȡin, out, status
		//(���գ����ͣ�״̬��ѯ)��endpoint��ע���ض��豸���е�netdev_ops��������������phy
		status = info->bind (dev, udev);
		if (status < 0)
			goto out1;

		// heuristic:  "usb%d" for links we know are two-host,
		// else "eth%d" when there's reasonable doubt.  userspace
		// can rename the link if it knows better.
		if ((dev->driver_info->flags & FLAG_ETHER) != 0 &&
		    ((dev->driver_info->flags & FLAG_POINTTOPOINT) == 0 ||
		     (net->dev_addr [0] & 0x02) == 0))
			strcpy (net->name, "eth%d"); /* �����豸���ƴ�usbxxx�ı�Ϊethxxx*/
		/* WLAN devices should always be named "wlan%d" */
		if ((dev->driver_info->flags & FLAG_WLAN) != 0)
			strcpy(net->name, "wlan%d");
		/* WWAN devices should always be named "wwan%d" */
		if ((dev->driver_info->flags & FLAG_WWAN) != 0)
			strcpy(net->name, "wwan%d");

		/* maybe the remote can't receive an Ethernet MTU */
		if (net->mtu > (dev->hard_mtu - net->hard_header_len))
			net->mtu = dev->hard_mtu - net->hard_header_len;
	} else if (!info->in || !info->out)
		status = usbnet_get_endpoints (dev, udev);
	else {
		/* ����usbnet�ӿ� */
		dev->in = usb_rcvbulkpipe (xdev, info->in);
		dev->out = usb_sndbulkpipe (xdev, info->out);
		if (!(info->flags & FLAG_NO_SETINT))
			status = usb_set_interface (xdev,
				interface->desc.bInterfaceNumber,
				interface->desc.bAlternateSetting);
		else
			status = 0;

	}
	if (status >= 0 && dev->status)
		status = init_status (dev, udev);
	if (status < 0)
		goto out3;

	if (!dev->rx_urb_size)
		dev->rx_urb_size = dev->hard_mtu;
	dev->maxpacket = usb_maxpacket (dev->udev, dev->out, 1);

	if ((dev->driver_info->flags & FLAG_WLAN) != 0)
		SET_NETDEV_DEVTYPE(net, &wlan_type);
	if ((dev->driver_info->flags & FLAG_WWAN) != 0)
		SET_NETDEV_DEVTYPE(net, &wwan_type);

	/* ע�������豸 */
	status = register_netdev (net);
	if (status)
		goto out4;
	netif_info(dev, probe, dev->net,
		   "register '%s' at usb-%s-%s, %s, %pM\n",
		   udev->dev.driver->name,
		   xdev->bus->bus_name, xdev->devpath,
		   dev->driver_info->description,
		   net->dev_addr);

	// ok, it's ready to go.
 	/* ����usb->usbnet����ϵ
  * ������Կ�����net->usbnet<-usb��usbnet����usb�豸��net�豸֮���Ŧ��
  */
	usb_set_intfdata (udev, dev);

	/* ���������豸 ���ϲ�ע����������豸��Э���Ϳ���ͨ������豸��tx rx�ӿڷ��ͽ������ݡ�*/
	netif_device_attach (net);

	if (dev->driver_info->flags & FLAG_LINK_INTR)
		netif_carrier_off(net);

	return 0;

out4:
	usb_free_urb(dev->interrupt);
out3:
	if (info->unbind)
		info->unbind (dev, udev);
out1:
	free_netdev(net);
out:
	usb_put_dev(xdev);
	return status;
}
//���Ͽ���֪����usb�豸�����󣬻����probe�ӿڡ�probe�ӿڴ���net�豸�����ṩ���û��ɼ���ʹ�á�

//#define alloc_etherdev(sizeof_priv) alloc_etherdev_mq(sizeof_priv, 1)
struct net_device *alloc_netdev_mq(int sizeof_priv, const char *name,
        void (*setup)(struct net_device *), unsigned int queue_count)
{
    struct netdev_queue *tx;
    struct net_device *dev;
    size_t alloc_size;
    void *p;

    BUG_ON(strlen(name) >= sizeof(dev->name)); /*���name�ֶγ���*/

    alloc_size = sizeof(struct net_device); /*��ȡ�ṹ���С*/

     /*�Ƿ����˽��������*/
    if (sizeof_priv) {
        /* ensure 32-byte alignment of private area */
        /*ȷ��˽����������32λ�����*/
        alloc_size = (alloc_size + NETDEV_ALIGN_CONST) & ~NETDEV_ALIGN_CONST;/*NETDEV_ALIGN_CONST = 31*/
        alloc_size += sizeof_priv;
    }
    /* ensure 32-byte alignment of whole construct */
    /*ȷ�������ṹ���ֽڶ����*/
    alloc_size += NETDEV_ALIGN_CONST;

     /*�����ڴ�ռ�*/
    p = kzalloc(alloc_size, GFP_KERNEL);
    if (!p) {
        printk(KERN_ERR "alloc_netdev: Unable to allocate device.\n");
        return NULL;
    }
       /*����queue_count�ĸ��������ڴ�ռ�*/
    tx = kcalloc(queue_count, sizeof(struct netdev_queue), GFP_KERNEL);
    if (!tx) {
        printk(KERN_ERR "alloc_netdev: Unable to allocate "
               "tx qdiscs.\n");
        kfree(p);
        return NULL;
    }
    /*devָ����32λ�����!!!*/
    dev = (struct net_device *)
        (((long)p + NETDEV_ALIGN_CONST) & ~NETDEV_ALIGN_CONST);
    dev->padded = (char *)dev - (char *)p;  /*�����������Ĵ�С*/
    dev_net_set(dev, &init_net);  /*�ú�����Ҫ�ں�����namespace��������*/

    dev->_tx = tx;                              /*���淢�Ͷ���*/
    dev->num_tx_queues = queue_count;           /*���淢�Ͷ�����*/
    dev->real_num_tx_queues = queue_count;     /*�����Լ���Ͷ�����*/

    dev->gso_max_size = GSO_MAX_SIZE; /*65536*/

    netdev_init_queues(dev);      /*��ʼ������*/

    INIT_LIST_HEAD(&dev->napi_list); /*��ʼ������ͷ*/
    setup(dev);                 /*���ú���ָ��setup����*/
    strcpy(dev->name, name); /*����name*/
    return dev;
}

		/*register_netdevice��ʼ�豸ע�Ṥ����������net_set_todo����net_set_todo����
		�����netdev_run_todo���ע�ᡣ
    register_netdevice�Ĺ�����Ҫ�������²��֣�
    ��ʼ��net_device�Ĳ����ֶ�
    ����ں�֧��Divert���ܣ�����alloc_divert_blk����ù�����������ݿռ�飬��������dev->divert
    ����豸�����Ѿ���dev->init���г�ʼ������ִ�д˺�����
    ��dev_new_index������豸һ��ʶ���롣
    ��net_device���뵽ȫ�ֱ�dev_base���Լ����Ź�ϣ��dev_name_head��dev_index_head��
    ��鹦�ܱ�ʶ�Ƿ�����Ч����ϡ�
    ����dev->state�е�__LINK_STATE_PRESENT��ʶ��ʹ���豸��Ϊ�ں����á�
    ��dev_init_scheduler��ʼ���豸���й����Ա�������������ʵ��Qos��
    ͨ��netdev_chain֪ͨ����֪ͨ���жԱ��豸ע�����Ȥ����ϵͳ��
    ��netdev_run_todo���������ע��ʱ����ֻ����dev->reg_state�������豸ע�����sysfs��
		*/
int register_netdev(struct net_device *dev)
{
	int err;

	rtnl_lock();
	err = register_netdevice(dev);//----->
	rtnl_unlock();
	return err;
}
int register_netdevice(struct net_device *dev)
{
	int ret;
	struct net *net = dev_net(dev);

	BUG_ON(dev_boot_phase);
	ASSERT_RTNL();

	might_sleep();

	/* When net_device's are persistent, this will be fatal. */
	BUG_ON(dev->reg_state != NETREG_UNINITIALIZED);
	BUG_ON(!net);

	spin_lock_init(&dev->addr_list_lock);
	netdev_set_addr_lockdep_class(dev);

	dev->iflink = -1;

	ret = dev_get_valid_name(dev, dev->name);
	if (ret < 0)
		goto out;

	/* Init, if this function is available */
	if (dev->netdev_ops->ndo_init) {
		ret = dev->netdev_ops->ndo_init(dev);
		if (ret) {
			if (ret > 0)
				ret = -EIO;
			goto out;
		}
	}

	dev->ifindex = dev_new_index(net);
	if (dev->iflink == -1)
		dev->iflink = dev->ifindex;

	/* Transfer changeable features to wanted_features and enable
	 * software offloads (GSO and GRO).
	 */
	dev->hw_features |= NETIF_F_SOFT_FEATURES;
	dev->features |= NETIF_F_SOFT_FEATURES;
	dev->wanted_features = dev->features & dev->hw_features;

	/* Turn on no cache copy if HW is doing checksum */
	if (!(dev->flags & IFF_LOOPBACK)) {
		dev->hw_features |= NETIF_F_NOCACHE_COPY;
		if (dev->features & NETIF_F_ALL_CSUM) {
			dev->wanted_features |= NETIF_F_NOCACHE_COPY;
			dev->features |= NETIF_F_NOCACHE_COPY;
		}
	}

	/* Make NETIF_F_HIGHDMA inheritable to VLAN devices.
	 */
	dev->vlan_features |= NETIF_F_HIGHDMA;

	ret = call_netdevice_notifiers(NETDEV_POST_INIT, dev);
	ret = notifier_to_errno(ret);
	if (ret)
		goto err_uninit;

	ret = netdev_register_kobject(dev);
	if (ret)
		goto err_uninit;
	dev->reg_state = NETREG_REGISTERED;

	__netdev_update_features(dev);

	/*
	 *	Default initial state at registry is that the
	 *	device is present.
	 */

	set_bit(__LINK_STATE_PRESENT, &dev->state);

	dev_init_scheduler(dev);
	dev_hold(dev);
	list_netdevice(dev);

	/* Notify protocols, that a new device appeared. */
	ret = call_netdevice_notifiers(NETDEV_REGISTER, dev);
	ret = notifier_to_errno(ret);
	if (ret) {
		rollback_registered(dev);
		dev->reg_state = NETREG_UNREGISTERED;
	}
	/*
	 *	Prevent userspace races by waiting until the network
	 *	device is fully setup before sending notifications.
	 */
	if (!dev->rtnl_link_ops ||
	    dev->rtnl_link_state == RTNL_LINK_INITIALIZED)
		rtmsg_ifinfo(RTM_NEWLINK, dev, ~0U);

out:
	return ret;

err_uninit:
	if (dev->netdev_ops->ndo_uninit)
		dev->netdev_ops->ndo_uninit(dev);
	goto out;
}


























