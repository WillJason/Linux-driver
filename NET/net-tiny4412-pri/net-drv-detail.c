/*
怎么写网卡驱动程序？
1. 分配一个net_device结构体
2. 设置:
2.1 发包函数: hard_start_xmit
2.2 收到数据时(在中断处理函数里)用netif_rx上报数据
2.3 其他设置
3. 注册: register_netdevice

tiny4412主要驱动函数是dm9620.c，我们就从这个驱动文件来分析网卡驱动程序

模块初始化时注册/创建usb设备。当发现有usb设备插入计算机后，会调用usb设备
的probe，即usbnet_probe。在usbnet_probe中创建网络设备，并建立net设备与usb
设备之间的关联。初始化就这样完成。
*/

/*先来看入口函数和出口函数*/
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
/*分配usb_driver结构体*/
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

/*probe函数分析*/
int usbnet_probe (struct usb_interface *udev, const struct usb_device_id *prod)
{
	struct usbnet			*dev;/* 私有数据结构，作为net设备与usb设备之间的关联表*/
	struct net_device		*net;/* 网络设备 */
	struct usb_host_interface	*interface;/* usb interface */
	struct driver_info		*info;
	struct usb_device		*xdev; /* usb设备*/
	int				status;
	/* usb设备和net网络设备是比较重要 */
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
	info = (struct driver_info *) prod->driver_info;//dm9620.c中定义的dm9620_info
	if (!info) {
		dev_dbg (&udev->dev, "blacklisted by %s\n", name);
		return -ENODEV;
	}
	xdev = interface_to_usbdev (udev);//根据usb_interface指针intf获取usb_device的地址。
	interface = udev->cur_altsetting;

	usb_get_dev (xdev);/*通过设备的接口，获得这个接口所对应的设备，并增加设备的引用次数*/

	status = -ENOMEM;

	// set up our own records
	/* 创建网络设备，网络设备的私有数据区保存的是usbnet信息。为什么要这么做？
	目的是*在发送数据时能够找到相关的usb设备。*/
	net = alloc_etherdev(sizeof(*dev));
	if (!net)
		goto out;

	/* netdev_printk() needs this so do it as early as possible */
	SET_NETDEV_DEV(net, &udev->dev);

	 /* 设置网络设备参数 */
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

	dev->net = net;/* usbnet中保存了网络设备 */
	strcpy (net->name, "usb%d"); /* 网络设备名称从ethxxx改变为usbxxx*/
	memcpy (net->dev_addr, node_id, sizeof node_id);/* 设置网络mac地址 */

	/* rx and tx sides can use different message sizes;
	 * bind() should set rx_urb_size in that case.
	 * 接收和发送数据消息大小可以是不同的。
   * bind() 时应该设置 rx_urb_size
   *
	*/
	dev->hard_mtu = net->mtu + net->hard_header_len;
#if 0
// dma_supported() is deeply broken on almost all architectures
	// possible with some EHCI controllers
	if (dma_supported (&udev->dev, DMA_BIT_MASK(64)))
		net->features |= NETIF_F_HIGHDMA;
#endif

	net->netdev_ops = &usbnet_netdev_ops;/*涉及到dm9000底层的操作函数*/
	net->watchdog_timeo = TX_TIMEOUT_JIFFIES;
	net->ethtool_ops = &usbnet_ethtool_ops;/* ethtool使用的接口 */

	// allow device-specific bind/init procedures
	// NOTE net->name still not usable ...
	/*/* 设置usbnet，配置net和usb设备的联系接口 */
	if (info->bind) {
		// 调用不同USB网卡各自注册的bind函数，主要作用是，获取in, out, status
		//(接收，发送，状态查询)的endpoint，注册特定设备特有的netdev_ops，启动配置网卡phy
		status = info->bind (dev, udev);
		if (status < 0)
			goto out1;

		// heuristic:  "usb%d" for links we know are two-host,
		// else "eth%d" when there's reasonable doubt.  userspace
		// can rename the link if it knows better.
		if ((dev->driver_info->flags & FLAG_ETHER) != 0 &&
		    ((dev->driver_info->flags & FLAG_POINTTOPOINT) == 0 ||
		     (net->dev_addr [0] & 0x02) == 0))
			strcpy (net->name, "eth%d"); /* 网络设备名称从usbxxx改变为ethxxx*/
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
		/* 设置usbnet接口 */
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

	/* 注册网络设备 */
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
 	/* 建立usb->usbnet的联系
  * 这里可以看出，net->usbnet<-usb，usbnet就是usb设备和net设备之间的纽带
  */
	usb_set_intfdata (udev, dev);

	/* 启动网络设备 向上层注册这个网卡设备，协议层就可以通过这个设备的tx rx接口发送接收数据。*/
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
//由上可以知道，usb设备创建后，会调用probe接口。probe接口创建net设备，以提供给用户可见和使用。

//#define alloc_etherdev(sizeof_priv) alloc_etherdev_mq(sizeof_priv, 1)
struct net_device *alloc_netdev_mq(int sizeof_priv, const char *name,
        void (*setup)(struct net_device *), unsigned int queue_count)
{
    struct netdev_queue *tx;
    struct net_device *dev;
    size_t alloc_size;
    void *p;

    BUG_ON(strlen(name) >= sizeof(dev->name)); /*检查name字段长度*/

    alloc_size = sizeof(struct net_device); /*获取结构体大小*/

     /*是否分配私有数据区*/
    if (sizeof_priv) {
        /* ensure 32-byte alignment of private area */
        /*确保私有数据区是32位对齐的*/
        alloc_size = (alloc_size + NETDEV_ALIGN_CONST) & ~NETDEV_ALIGN_CONST;/*NETDEV_ALIGN_CONST = 31*/
        alloc_size += sizeof_priv;
    }
    /* ensure 32-byte alignment of whole construct */
    /*确保整个结构是字节对齐的*/
    alloc_size += NETDEV_ALIGN_CONST;

     /*分配内存空间*/
    p = kzalloc(alloc_size, GFP_KERNEL);
    if (!p) {
        printk(KERN_ERR "alloc_netdev: Unable to allocate device.\n");
        return NULL;
    }
       /*根据queue_count的个数分配内存空间*/
    tx = kcalloc(queue_count, sizeof(struct netdev_queue), GFP_KERNEL);
    if (!tx) {
        printk(KERN_ERR "alloc_netdev: Unable to allocate "
               "tx qdiscs.\n");
        kfree(p);
        return NULL;
    }
    /*dev指针是32位对齐的!!!*/
    dev = (struct net_device *)
        (((long)p + NETDEV_ALIGN_CONST) & ~NETDEV_ALIGN_CONST);
    dev->padded = (char *)dev - (char *)p;  /*计算填充区域的大小*/
    dev_net_set(dev, &init_net);  /*该函数需要内核配置namespace，先无视*/

    dev->_tx = tx;                              /*保存发送队列*/
    dev->num_tx_queues = queue_count;           /*保存发送队列数*/
    dev->real_num_tx_queues = queue_count;     /*保存以激活发送队列数*/

    dev->gso_max_size = GSO_MAX_SIZE; /*65536*/

    netdev_init_queues(dev);      /*初始化队列*/

    INIT_LIST_HEAD(&dev->napi_list); /*初始化链表头*/
    setup(dev);                 /*调用函数指针setup函数*/
    strcpy(dev->name, name); /*拷贝name*/
    return dev;
}

		/*register_netdevice开始设备注册工作，并调用net_set_todo，而net_set_todo最终
		会调用netdev_run_todo完成注册。
    register_netdevice的工作主要包括以下部分：
    初始化net_device的部分字段
    如果内核支持Divert功能，则用alloc_divert_blk分配该功能所需的数据空间块，并连接至dev->divert
    如果设备驱动已经对dev->init进行初始化，则执行此函数。
    由dev_new_index分配给设备一个识别码。
    把net_device插入到全局表dev_base，以及两张哈希表dev_name_head，dev_index_head。
    检查功能标识是否有无效的组合。
    设置dev->state中的__LINK_STATE_PRESENT标识，使得设备能为内核所用。
    用dev_init_scheduler初始化设备队列规则，以便流量控制用于实现Qos。
    通过netdev_chain通知表链通知所有对本设备注册感兴趣的子系统。
    当netdev_run_todo被调用完成注册时，它只更新dev->reg_state，并将设备注册进入sysfs。
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


























