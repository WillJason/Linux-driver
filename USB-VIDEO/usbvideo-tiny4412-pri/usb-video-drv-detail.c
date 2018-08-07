/*
虚拟视频驱动vivi.c分析:
1．分配video_device
2．设置
3．注册：video_register_device

vivi_init
    vivi_create_instance
        v4l2_device_register   // 不是主要, 只是用于初始化一些东西，比如自旋锁、引用计数
        video_device_alloc
        // 设置
          1. vfd:
            .fops           = &vivi_fops,
            .ioctl_ops 	= &vivi_ioctl_ops,
            .release	= video_device_release,
          2.
            vfd->v4l2_dev = &dev->v4l2_dev;
          3. 设置"ctrl属性"(用于APP的ioctl)：
            	v4l2_ctrl_handler_init(hdl, 11);
            	dev->volume = v4l2_ctrl_new_std(hdl, &vivi_ctrl_ops,
            			V4L2_CID_AUDIO_VOLUME, 0, 255, 1, 200);
            	dev->brightness = v4l2_ctrl_new_std(hdl, &vivi_ctrl_ops,
            			V4L2_CID_BRIGHTNESS, 0, 255, 1, 127);
            	dev->contrast = v4l2_ctrl_new_std(hdl, &vivi_ctrl_ops,
            			V4L2_CID_CONTRAST, 0, 255, 1, 16);                        
        video_register_device(video_device, type:VFL_TYPE_GRABBER, nr)
            __video_register_device
                vdev->cdev = cdev_alloc();
                vdev->cdev->ops = &v4l2_fops;
                cdev_add
                
                video_device[vdev->minor] = vdev;

        		if (vdev->ctrl_handler == NULL)
        			vdev->ctrl_handler = vdev->v4l2_dev->ctrl_handler;

USB摄像头驱动程序

1.构造一个usb_driver
2.设置
   probe:
        2.1. 分配video_device:video_device_alloc
        2.2. 设置
           .fops
           .ioctl_ops (里面需要设置11项)
           如果要用内核提供的缓冲区操作函数，还需要构造一个videobuf_queue_ops
        2.3. 注册: video_register_device      
  id_table: 表示支持哪些USB设备      
3.注册： usb_register
 
UVC: USB Video Class
UVC驱动：drivers\media\video\uvc\ 这个目录下的所有文件都是UVC驱动程序

分析一个驱动程序就是跟踪应用程序对它的调用过程
先了解一下USB摄像头内部硬件的框架
下载Uvc的规格书
1. UVC设备有2个interface: VideoControl Interface, VideoStreaming Interface
2. VideoControl Interface用于控制，比如设置亮度。它内部有多个Unit/Terminal(在程序里Unit/Terminal都称为entity)
   可以通过类似的函数来访问：
                			ret = uvc_query_ctrl(dev  / 哪一个USB设备 /, SET_CUR, ctrl->entity->id  / 哪一个unit/terminal /,
                				dev->intfnum / 哪一个接口: VC interface /, ctrl->info->selector,
                				uvc_ctrl_data(ctrl, UVC_CTRL_DATA_CURRENT),
                				ctrl->info->size);
3. VideoStreaming Interface用于获得视频数据 ，也可以用来选择fromat/frame(VS可能有多种format, 一个format支持多种frame， frame用来表示分辨率等信息)
   可以通过类似的函数来访问：
                    	ret = __uvc_query_ctrl(video->dev / 哪一个USB设备 /, SET_CUR, 0,
                    		video->streaming->intfnum  / 哪一个接口: VS /,
                    		probe ? VS_PROBE_CONTROL : VS_COMMIT_CONTROL, data, size,
                    		uvc_timeout_param);
4. 我们在设置FORMAT时只是简单的使用video->streaming->format[fmt->index]等数据，
   这些数据哪来的？
   应是设备被枚举时设置的，也就是分析它的描述符时设置的。

5. UVC驱动的重点在于：
   描述符的分析
   属性的控制: 通过VideoControl Interface来设置
   格式的选择：通过VideoStreaming Interface来设置
   数据的获得：通过VideoStreaming Interface的URB来获得
        			    			
*/

/*先来看入口函数和出口函数*/
static int __init uvc_init(void)
{
	int ret;

	uvc_debugfs_init();

	ret = usb_register(&uvc_driver.driver);
	if (ret < 0) {
		uvc_debugfs_cleanup();
		return ret;
	}

	printk(KERN_INFO DRIVER_DESC " (" DRIVER_VERSION ")\n");
	return 0;
}

static void __exit uvc_cleanup(void)
{
	usb_deregister(&uvc_driver.driver);
	uvc_debugfs_cleanup();
}

module_init(uvc_init);
module_exit(uvc_cleanup);
/*分配usb_driver结构体*/
struct uvc_driver uvc_driver = {
	.driver = {
		.name		= "uvcvideo",
		.probe		= uvc_probe,
		.disconnect	= uvc_disconnect,
		.suspend	= uvc_suspend,
		.resume		= uvc_resume,
		.reset_resume	= uvc_reset_resume,
		.id_table	= uvc_ids,
		.supports_autosuspend = 1,
	},
};
static struct usb_device_id uvc_ids[] = {
	/*iPassion USB Web Camera */
    { .match_flags = USB_DEVICE_ID_MATCH_DEVICE
                      | USB_DEVICE_ID_MATCH_INT_INFO,
    .idVendor = 0x1B3B,//百问网自制摄像头
    .idProduct = 0x2970,/*If you use iP2977, then type "0x2977" */                  
    .bInterfaceClass = USB_CLASS_VIDEO,
    .bInterfaceSubClass = 1,
    .bInterfaceProtocol = 0,
    .driver_info = UVC_QUIRK_PROBE_MINMAX
                     | UVC_QUIRK_IGNORE_SELECTOR_UNIT},
	/* LogiLink Wireless Webcam */
	{ .match_flags		= USB_DEVICE_ID_MATCH_DEVICE
				| USB_DEVICE_ID_MATCH_INT_INFO,
	  .idVendor		= 0x0416,
	  .idProduct		= 0xa91a,
	  .bInterfaceClass	= USB_CLASS_VIDEO,
	  .bInterfaceSubClass	= 1,
	  .bInterfaceProtocol	= 0,
	  .driver_info		= UVC_QUIRK_PROBE_MINMAX },
	  .......
};

/*probe函数分析*/
static int uvc_probe(struct usb_interface *intf,
		     const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);//根据usb_interface指针intf获取usb_device的地址。
	struct uvc_device *dev;									//声明uvc设备
	int ret;

	if (id->idVendor && id->idProduct)				//有厂商id和商品id(知名设备
		uvc_trace(UVC_TRACE_PROBE, "Probing known UVC device %s "
				"(%04x:%04x)\n", udev->devpath, id->idVendor,
				id->idProduct);
	else																			//通用uvc设备
		uvc_trace(UVC_TRACE_PROBE, "Probing generic UVC device %s\n",
				udev->devpath);

	/* Allocate memory for the device and initialize it. */
	if ((dev = kzalloc(sizeof *dev, GFP_KERNEL)) == NULL)//分配uvc设备内存  
		return -ENOMEM;

	INIT_LIST_HEAD(&dev->entities);						//初始化entities(实体)链表 Terminal或Unit  
	INIT_LIST_HEAD(&dev->chains);							//初始化chains(链)链表  
	INIT_LIST_HEAD(&dev->streams);						//初始化streams(视频流)链表  
	atomic_set(&dev->nstreams, 0);						//设置原子变量的值。
	atomic_set(&dev->users, 0);
	atomic_set(&dev->nmappings, 0);

	dev->udev = usb_get_dev(udev);						//捆绑usb设备,并增加其引用计数 
	dev->intf = usb_get_intf(intf);						//捆绑usb接口,并增加其引用计数
	dev->intfnum = intf->cur_altsetting->desc.bInterfaceNumber;//获取usb接口描述符接口数
	dev->quirks = (uvc_quirks_param == -1)
		    ? id->driver_info : uvc_quirks_param;

	if (udev->product != NULL)								//存在产品名
		strlcpy(dev->name, udev->product, sizeof dev->name);//设置uvc设备名字为其产品名
	else																			//通用的uvc设备名
		snprintf(dev->name, sizeof dev->name,
			"UVC Camera (%04x:%04x)",
			le16_to_cpu(udev->descriptor.idVendor),
			le16_to_cpu(udev->descriptor.idProduct));

	/* Parse the Video Class control descriptor. */
	if (uvc_parse_control(dev) < 0) {					//[@5]uvc解析usb视频类控制描述符,分析设备的控制描述符
		uvc_trace(UVC_TRACE_PROBE, "Unable to parse UVC "
			"descriptors.\n");
		goto error;
	}

	uvc_printk(KERN_INFO, "Found UVC %u.%02x device %s (%04x:%04x)\n",
		dev->uvc_version >> 8, dev->uvc_version & 0xff,
		udev->product ? udev->product : "<unnamed>",
		le16_to_cpu(udev->descriptor.idVendor),
		le16_to_cpu(udev->descriptor.idProduct));

	if (dev->quirks != id->driver_info) {
		uvc_printk(KERN_INFO, "Forcing device quirks to 0x%x by module "
			"parameter for testing purpose.\n", dev->quirks);
		uvc_printk(KERN_INFO, "Please report required quirks to the "
			"linux-uvc-devel mailing list.\n");
	}

	/* Register the media and V4L2 devices. */
#ifdef CONFIG_MEDIA_CONTROLLER
	dev->mdev.dev = &intf->dev;
	strlcpy(dev->mdev.model, dev->name, sizeof(dev->mdev.model));
	if (udev->serial)
		strlcpy(dev->mdev.serial, udev->serial,
			sizeof(dev->mdev.serial));
	strcpy(dev->mdev.bus_info, udev->devpath);
	dev->mdev.hw_revision = le16_to_cpu(udev->descriptor.bcdDevice);
	dev->mdev.driver_version = LINUX_VERSION_CODE;
	if (media_device_register(&dev->mdev) < 0)
		goto error;

	dev->vdev.mdev = &dev->mdev;
#endif
	if (v4l2_device_register(&intf->dev, &dev->vdev) < 0)//调用v4l2_device_register函数初始化v4l2_dev ，比如自旋锁、引用计数\\初始化v4l2_dev->subdevs子设备实例的链表，然后设置名字和设置dev->driver_data
		goto error;

	/* Initialize controls. */
	if (uvc_ctrl_init_device(dev) < 0)					//[@8]uvc初始化uvc控制设备 
		goto error;

	/* Scan the device for video chains. */
	if (uvc_scan_device(dev) < 0)								//[@10]uvc扫描视频链
		goto error;

	/* Register video device nodes. */
	if (uvc_register_chains(dev) < 0)						//[@11]uvc注册视频设备
		goto error;

	/* Save our data pointer in the interface data. */
	usb_set_intfdata(intf, dev);								//设置uvc设备为usb接口的数据

	/* Initialize the interrupt URB. */
	if ((ret = uvc_status_init(dev)) < 0) {			//[@12]uvc设备状态初始化
		uvc_printk(KERN_INFO, "Unable to initialize the status "
			"endpoint (%d), status interrupt will not be "
			"supported.\n", ret);
	}

	uvc_trace(UVC_TRACE_PROBE, "UVC device initialized.\n");
	usb_enable_autosuspend(udev);								//使能自动挂起
	return 0;

error:
	uvc_unregister_video(dev);
	return -ENODEV;
}


/*
[@5]调用uvc_parse_control函数 
看下调用关系：

uvc_parse_control(dev)
    uvc_parse_standard_control(dev, buffer, buflen)
        uvc_parse_streaming(dev, intf)      
跟踪下uvc_parse_streaming函数：
*/
static int uvc_parse_streaming(struct uvc_device *dev,
    struct usb_interface *intf)
{
    /*以下大部分内容省略，只显示重要的*/
    struct uvc_streaming *streaming = NULL;
    struct uvc_format *format;
    struct uvc_frame *frame;

    streaming = kzalloc(sizeof *streaming, GFP_KERNEL);
    size = nformats * sizeof *format + nframes * sizeof *frame
         + nintervals * sizeof *interval;

    format = kzalloc(size, GFP_KERNEL);//申请format数组存放格式
    streaming->format = format;//设置格式
    streaming->nformats = nformats;//最多支持nformats种格式
    ret = uvc_parse_format(dev, streaming, format,
                &interval, buffer, buflen);//分析格式
    list_add_tail(&streaming->list, &dev->streams);
    return 0;
}
/*这里面申请了streaming和format内存 
streaming是uvc_streaming 结构体，视频流，很重要，大部分参数都是存在里面。这函数里申请了之后进行了很多设置，不过现在我省略了写。 
format内存存放的是视频的格式，frame存放的是如分辨率 
这里面都把他设置到了streaming里面（streaming->format = format;streaming->nformats = nformats;） 
最后调用uvc_parse_format函数分析格式：
*/
static int uvc_parse_format()
{
    fmtdesc = uvc_format_by_guid(&buffer[5]);//通过GUID找到格式format
    /*里面还会对frame进行各种分析和设置，
     *如设置format->nframes得出最多有多少种分辨率选择
     *暂时忽略*/
}
//里面uvc_format_by_guid函数会从uvc_fmts数组中通过匹配guid找到格式：

static struct uvc_format_desc uvc_fmts[] = {
    {
        .name       = "YUV 4:2:2 (YUYV)",
        .guid       = UVC_GUID_FORMAT_YUY2,
        .fcc        = V4L2_PIX_FMT_YUYV,
    },
    {
        .name       = "YUV 4:2:2 (YUYV)",
        .guid       = UVC_GUID_FORMAT_YUY2_ISIGHT,
        .fcc        = V4L2_PIX_FMT_YUYV,
    },
    {
        .name       = "YUV 4:2:0 (NV12)",
        .guid       = UVC_GUID_FORMAT_NV12,
        .fcc        = V4L2_PIX_FMT_NV12,
    },
    {
        .name       = "MJPEG",
        .guid       = UVC_GUID_FORMAT_MJPEG,
        .fcc        = V4L2_PIX_FMT_MJPEG,
    },
    /*后面省略......*/
}


/*
[@8]调用uvc_ctrl_init_device
*/
int uvc_ctrl_init_device(struct uvc_device *dev)
{
    /*省略了部分内容*/
    list_for_each_entry(entity, &dev->entities, list) {
        bmControls = entity->extension.bmControls;//控制位图
        bControlSize = entity->extension.bControlSize;//控制位域大小
        entity->controls = kcalloc(ncontrols, sizeof(*ctrl),
                       GFP_KERNEL);//分配ncontrols个uvc控制内存
        if (entity->controls == NULL)
            return -ENOMEM;
        entity->ncontrols = ncontrols;//设置uvc控制个数

        /* Initialize all supported controls */
        ctrl = entity->controls;//指向uvc控制数组
        for (i = 0; i < bControlSize * 8; ++i) {
            if (uvc_test_bit(bmControls, i) == 0)//跳过控制位域设置0的
                continue;
            ctrl->entity = entity;
            ctrl->index = i;//设置控制位域索引
            uvc_ctrl_init_ctrl(dev, ctrl);//初始化uvc控件
            ctrl++;//uvc控制 指向下一个uvc控制数组项
        }
    }
}
/*uvc_ctrl_init_device主要就是初始化控制参数，里面就会遍历uvc设备实体entities链表，然后设置位图和位域大小 
最后还会调用uvc_ctrl_init_ctrl函数设置背光，色温等等*/


/*
[@10]
*/


/*
[@11]调用uvc_register_chains函数: 
调用关系：
uvc_register_chains
    uvc_register_terms(dev, chain)
        uvc_stream_by_id
        uvc_register_video
    uvc_mc_register_entities(chain)
uvc_stream_by_id函数会通过函数传入的id和dev->streams链表的header.bTerminalLink匹配，寻找到stream 
这不是重点，我们的重点是uvc_register_video函数，找到stream会就要注册：*/

static int uvc_register_video(struct uvc_device *dev,
        struct uvc_streaming *stream)
{
    /*部分内容省略......*/
    struct video_device *vdev = &stream->vdev;

    ret = uvc_queue_init(&stream->queue, stream->type, !uvc_no_drop_param);//初始化队列
    ret = uvc_video_init(stream);//初始化

    uvc_debugfs_init_stream(stream);
    vdev->v4l2_dev = &dev->vdev;
    vdev->fops = &uvc_fops;//v4l2操作函数集
    vdev->ioctl_ops = &uvc_ioctl_ops;//设置真正的ioctl操作集
    vdev->release = uvc_release;//释放方法
    vdev->prio = &stream->chain->prio;

    strlcpy(vdev->name, dev->name, sizeof vdev->name);
    video_set_drvdata(vdev, stream);//将uvc视频流作为v4l2设备的驱动数据

    ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);//注册
    return 0;
}

const struct v4l2_file_operations uvc_fops = {
	.owner		= THIS_MODULE,
	.open		= uvc_v4l2_open,
	.release	= uvc_v4l2_release,
	.unlocked_ioctl	= uvc_v4l2_ioctl,
	#ifdef CONFIG_COMPAT
	.compat_ioctl32	= uvc_v4l2_compat_ioctl32,
	#endif
	.read		= uvc_v4l2_read,
	.mmap		= uvc_v4l2_mmap,
	.poll		= uvc_v4l2_poll,
	#ifndef CONFIG_MMU
	.get_unmapped_area = uvc_v4l2_get_unmapped_area,
	#endif
};

/*这是非常重要的函数，我们来一点一点分析： 
看下uvc_queue_init函数，队列初始化，队列这东西，我们视频传输时会调用到，在ioctl里操作：*/

static struct vb2_ops uvc_queue_qops = {
    .queue_setup = uvc_queue_setup,
    .buf_prepare = uvc_buffer_prepare,
    .buf_queue = uvc_buffer_queue,
    .buf_finish = uvc_buffer_finish,
    .wait_prepare = vb2_ops_wait_prepare,
    .wait_finish = vb2_ops_wait_finish,
    .start_streaming = uvc_start_streaming,
    .stop_streaming = uvc_stop_streaming,
};
int uvc_queue_init(struct uvc_video_queue *queue, enum v4l2_buf_type type,
            int drop_corrupted)
{
    queue->queue.type = type;
    queue->queue.io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
    queue->queue.drv_priv = queue;
    queue->queue.buf_struct_size = sizeof(struct uvc_buffer);
    queue->queue.ops = &uvc_queue_qops;//stream->queue->queue.ops
    queue->queue.mem_ops = &vb2_vmalloc_memops;
    queue->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC
        | V4L2_BUF_FLAG_TSTAMP_SRC_SOE;
    queue->queue.lock = &queue->mutex;
    ret = vb2_queue_init(&queue->queue);//初始化queue

    mutex_init(&queue->mutex);
    spin_lock_init(&queue->irqlock);
    INIT_LIST_HEAD(&queue->irqqueue);//初始化stream->queue->irqqueue
    queue->flags = drop_corrupted ? UVC_QUEUE_DROP_CORRUPTED : 0;

    return 0;
}
/*里面先对队列进行初始化设置，如设置type和ops。 
这里queue->queue.ops = &uvc_queue_qops非常重要，之后我们调用vidioc_streamon回调函数时就是调用到这里的uvc_queue_qops结构体里的.start_streaming函数 
这函数里对各种队列进行了初始化：*/

vb2_queue_init(&queue->queue)
    q->buf_ops = &v4l2_buf_ops;
    vb2_core_queue_init(struct vb2_queue *q)
        INIT_LIST_HEAD(&q->queued_list);//stream->queue->queue->queued_list
        INIT_LIST_HEAD(&q->done_list);//stream->queue->done_list
INIT_LIST_HEAD(&queue->irqqueue);//初始化stream->queue->irqqueue

//我们继续看回uvc_register_video函数，里面接着调用了uvc_video_init函数初始化UVC视频设备：

int uvc_video_init(struct uvc_streaming *stream)
{
    /*省略部分内容*/
    struct uvc_streaming_control *probe = &stream->ctrl;//获取uvc数据流的uvs数据流控制对象

    if (uvc_get_video_ctrl(stream, probe, 1, UVC_GET_DEF) == 0)//先得到定义的控制参数
        uvc_set_video_ctrl(stream, probe, 1);//再设置uvc视频控制
    ret = uvc_get_video_ctrl(stream, probe, 1, UVC_GET_CUR);//最后在get一次
    for (i = stream->nformats; i > 0; --i) {
        format = &stream->format[i-1];//获取对应的uvc格式
        if (format->index == probe->bFormatIndex)
            break;
    }
    probe->bFormatIndex = format->index;//设置uvc视频流控制的格式索引为uvc格式的索引
    probe->bFrameIndex = frame->bFrameIndex;//设置uvc视频流控制的分辨率索引为uvc分辨率的索引

    stream->def_format = format;
    stream->cur_format = format;//设置uvc格式为uvc数据流的cur_format成员
    stream->cur_frame = frame;//设置uvc帧为uvc数据流的cur_frame成员

    if (stream->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {//视频采集
        if (stream->dev->quirks & UVC_QUIRK_BUILTIN_ISIGHT)
            stream->decode = uvc_video_decode_isight;
        else if (stream->intf->num_altsetting > 1)
            stream->decode = uvc_video_decode_isoc;//同步方式
        else
            stream->decode = uvc_video_decode_bulk;//bluk方式
    } 
    return 0;
}
/*这里面内容就比较多了，先得到，然后设置uvc的控制参数，里面会操作urb发出usb数据。 
然后通过probe->bFormatIndex索引找到使用的format格式和通过probe->bFrameIndex找到对应的frame分辨率，然后设置到stream里。 
最后选择解码方式，如同步方式或者bluk方式，解码方式会在数据完成时被回调函数complete里调用。

再次回到uvc_register_video函数，没办法，这个函数太重要了： 
里面继续：

    vdev->fops = &uvc_fops;//v4l2操作函数集
    vdev->ioctl_ops = &uvc_ioctl_ops;//设置真正的ioctl操作集
    vdev->release = uvc_release;//释放方法
    ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);

里面就是vdev->v4l2_dev = &dev->vdev;这样v4l2_device就与video_device关联起来，也就是我们文章一开始那个图看到的。 
然后设置fops操作函数vdev->fops = &uvc_fops，虽然这不是给用户空间使用的open、read、write函数，但是最后vdev->cdev->ops还是最调用到这个uvc_fops的，所以用户空间实际上的pen、read、write函数还是会在这调用。 然后ioctl操作函数最终是会调用到vdev->ioctl_ops = &uvc_ioctl_ops。可以说，V4L2最重要的就是各种形式的ioctl了，这里先不讲，下一节在分析看看。 
然后最终就是我们的注册函数了：video_register_device里调用到__video_register_device函数：*/
int __video_register_device(struct video_device *vdev, int type, int nr,
        int warn_if_nr_in_use, struct module *owner)
{
    /*省略部分函数*/
    vdev->minor = -1;//-1表明这个video device从未被注册过
    switch (type) {//根据type选择设备名称
    case VFL_TYPE_GRABBER:
        name_base = "video";
        break;
    case VFL_TYPE_VBI:
        name_base = "vbi";
        break;
    case VFL_TYPE_RADIO:
        name_base = "radio";
        break;
    case VFL_TYPE_SUBDEV:
        name_base = "v4l-subdev";
        break;
    case VFL_TYPE_SDR:
        name_base = "swradio";
        break;
    default:
        printk(KERN_ERR "%s called with unknown type: %d\n", __func__, type);
        return -EINVAL;
    }
    switch (type) {//选择得到次设备号偏移值
    case VFL_TYPE_GRABBER://用于视频输入/输出设备的 videoX
        minor_offset = 0;
        minor_cnt = 64;
        break;
    case VFL_TYPE_RADIO://用于广播调谐器的 radioX
        minor_offset = 64;
        minor_cnt = 64;
        break;
    case VFL_TYPE_VBI://用于垂直消隐数据的 vbiX (例如，隐藏式字幕，图文电视)
        minor_offset = 224;
        minor_cnt = 32;
        break;
    default:
        minor_offset = 128;
        minor_cnt = 64;
        break;
    }
    nr = devnode_find(vdev, 0, minor_cnt);//获取一个没有被使用的设备节点序号
    for (i = 0; i < VIDEO_NUM_DEVICES; i++)
        if (video_device[i] == NULL)//从video_device[]数组中选择一个空缺项，这个空缺项的索引值放到i中
            break;
    vdev->minor = i + minor_offset;//设备的次设备号
    video_device[vdev->minor] = vdev;//注意：将设置好的video_device放入到video_device[]
    vdev->cdev->ops = &v4l2_fops;//操作用户空间操作函数集
    ret = cdev_add(vdev->cdev, MKDEV(VIDEO_MAJOR, vdev->minor), 1);//添加字符设备到系统
    ret = device_register(&vdev->dev);//设备注册
    set_bit(V4L2_FL_REGISTERED, &vdev->flags);//将flags第0为设置为1，表示这个video_device是注册过的了

    return 0;

}
/*我们梳理一下里面做的事情： 
1.确定设备名称，也就是我们在/dev/下生成的video啊，radio之类的 
2.得到次设备的偏移值 
3.找到一个空的video_device数组，把vdev存进去 
4.设置vdev->cdev，这里就设置了vdev->cdev->ops = &v4l2_fops;里面就是真正的用户空间操作集合,后面详细介绍内容 
5.注册video_device设备 
6.就是标志此video_device以注册

最后【11】调用uvc_register_chains函数里还会调用一个uvc_mc_register_entities函数，里面继续调用
uvc_mc_init_entity函数，这就是v4l2_device_register_subdev函数，进行注册v4l2_subdev，同时初始化
然后连接到v4l2_dev->subdevs管理。
*/


/*
[@12]调用uvc_status_init函数
*/
int uvc_status_init(struct uvc_device *dev)
{
    /*省略部分函数*/
    struct usb_host_endpoint *ep = dev->int_ep;//获取usb_host_endpoint

    uvc_input_init(dev);//初始化uvc输入设备,里面注册input设备
    dev->status = kzalloc(UVC_MAX_STATUS_SIZE, GFP_KERNEL);//分配urb设备状态内存
    dev->int_urb = usb_alloc_urb(0, GFP_KERNEL);//分配urb
    pipe = usb_rcvintpipe(dev->udev, ep->desc.bEndpointAddress);//中断输入端点
    usb_fill_int_urb(dev->int_urb, dev->udev, pipe,
        dev->status, UVC_MAX_STATUS_SIZE, uvc_status_complete,
        dev, interval);//填充中断urb

    return 0;
}
/*里面就是关于urb的一些东西了，看看就好。

最后，我们用户空间怎么才操作的？ 
看看__video_register_device函数里的：vdev->cdev->ops = &v4l2_fops;
*/
static const struct file_operations v4l2_fops = {
    .owner = THIS_MODULE,
    .read = v4l2_read,
    .write = v4l2_write,
    .open = v4l2_open,
    .get_unmapped_area = v4l2_get_unmapped_area,
    .mmap = v4l2_mmap,
    .unlocked_ioctl = v4l2_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = v4l2_compat_ioctl32,
#endif
    .release = v4l2_release,
    .poll = v4l2_poll,
    .llseek = no_llseek,
};
static int v4l2_open(struct inode *inode, struct file *filp)
{
    /*省略部分函数*/
    struct video_device *vdev;
    vdev = video_devdata(filp);//根据次设备号从video_devices[]数组中得到video_device
    if (vdev->fops->open) {
        if (video_is_registered(vdev))
            ret = vdev->fops->open(filp);//实际就是vdev->fops
        else
            ret = -ENODEV;
    }
}
/*记得我们之前把video_device放入到video_device[]吗？就是这里取了出来 
然后调用vdev->fops->open(filp) 
vdev->fops就是我们在uvc_register_video函数里设置的： 
vdev->fops = &uvc_fops
*/
const struct v4l2_file_operations uvc_fops = {//实际的用户操作
    .owner      = THIS_MODULE,
    .open       = uvc_v4l2_open,
    .release    = uvc_v4l2_release,
    .unlocked_ioctl = video_ioctl2,
#ifdef CONFIG_COMPAT
    .compat_ioctl32 = uvc_v4l2_compat_ioctl32,
#endif
    .read       = uvc_v4l2_read,
    .mmap       = uvc_v4l2_mmap,
    .poll       = uvc_v4l2_poll,
#ifndef CONFIG_MMU
    .get_unmapped_area = uvc_v4l2_get_unmapped_area,
#endif
};







