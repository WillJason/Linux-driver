/*
������Ƶ����vivi.c����:
1������video_device
2������
3��ע�᣺video_register_device

vivi_init
    vivi_create_instance
        v4l2_device_register   // ������Ҫ, ֻ�����ڳ�ʼ��һЩ���������������������ü���
        video_device_alloc
        // ����
          1. vfd:
            .fops           = &vivi_fops,
            .ioctl_ops 	= &vivi_ioctl_ops,
            .release	= video_device_release,
          2.
            vfd->v4l2_dev = &dev->v4l2_dev;
          3. ����"ctrl����"(����APP��ioctl)��
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

USB����ͷ��������

1.����һ��usb_driver
2.����
   probe:
        2.1. ����video_device:video_device_alloc
        2.2. ����
           .fops
           .ioctl_ops (������Ҫ����11��)
           ���Ҫ���ں��ṩ�Ļ�������������������Ҫ����һ��videobuf_queue_ops
        2.3. ע��: video_register_device      
  id_table: ��ʾ֧����ЩUSB�豸      
3.ע�᣺ usb_register
 
UVC: USB Video Class
UVC������drivers\media\video\uvc\ ���Ŀ¼�µ������ļ�����UVC��������

����һ������������Ǹ���Ӧ�ó�������ĵ��ù���
���˽�һ��USB����ͷ�ڲ�Ӳ���Ŀ��
����Uvc�Ĺ����
1. UVC�豸��2��interface: VideoControl Interface, VideoStreaming Interface
2. VideoControl Interface���ڿ��ƣ������������ȡ����ڲ��ж��Unit/Terminal(�ڳ�����Unit/Terminal����Ϊentity)
   ����ͨ�����Ƶĺ��������ʣ�
                			ret = uvc_query_ctrl(dev  / ��һ��USB�豸 /, SET_CUR, ctrl->entity->id  / ��һ��unit/terminal /,
                				dev->intfnum / ��һ���ӿ�: VC interface /, ctrl->info->selector,
                				uvc_ctrl_data(ctrl, UVC_CTRL_DATA_CURRENT),
                				ctrl->info->size);
3. VideoStreaming Interface���ڻ����Ƶ���� ��Ҳ��������ѡ��fromat/frame(VS�����ж���format, һ��format֧�ֶ���frame�� frame������ʾ�ֱ��ʵ���Ϣ)
   ����ͨ�����Ƶĺ��������ʣ�
                    	ret = __uvc_query_ctrl(video->dev / ��һ��USB�豸 /, SET_CUR, 0,
                    		video->streaming->intfnum  / ��һ���ӿ�: VS /,
                    		probe ? VS_PROBE_CONTROL : VS_COMMIT_CONTROL, data, size,
                    		uvc_timeout_param);
4. ����������FORMATʱֻ�Ǽ򵥵�ʹ��video->streaming->format[fmt->index]�����ݣ�
   ��Щ���������ģ�
   Ӧ���豸��ö��ʱ���õģ�Ҳ���Ƿ�������������ʱ���õġ�

5. UVC�������ص����ڣ�
   �������ķ���
   ���ԵĿ���: ͨ��VideoControl Interface������
   ��ʽ��ѡ��ͨ��VideoStreaming Interface������
   ���ݵĻ�ã�ͨ��VideoStreaming Interface��URB�����
        			    			
*/

/*��������ں����ͳ��ں���*/
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
/*����usb_driver�ṹ��*/
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
    .idVendor = 0x1B3B,//��������������ͷ
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

/*probe��������*/
static int uvc_probe(struct usb_interface *intf,
		     const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);//����usb_interfaceָ��intf��ȡusb_device�ĵ�ַ��
	struct uvc_device *dev;									//����uvc�豸
	int ret;

	if (id->idVendor && id->idProduct)				//�г���id����Ʒid(֪���豸
		uvc_trace(UVC_TRACE_PROBE, "Probing known UVC device %s "
				"(%04x:%04x)\n", udev->devpath, id->idVendor,
				id->idProduct);
	else																			//ͨ��uvc�豸
		uvc_trace(UVC_TRACE_PROBE, "Probing generic UVC device %s\n",
				udev->devpath);

	/* Allocate memory for the device and initialize it. */
	if ((dev = kzalloc(sizeof *dev, GFP_KERNEL)) == NULL)//����uvc�豸�ڴ�  
		return -ENOMEM;

	INIT_LIST_HEAD(&dev->entities);						//��ʼ��entities(ʵ��)���� Terminal��Unit  
	INIT_LIST_HEAD(&dev->chains);							//��ʼ��chains(��)����  
	INIT_LIST_HEAD(&dev->streams);						//��ʼ��streams(��Ƶ��)����  
	atomic_set(&dev->nstreams, 0);						//����ԭ�ӱ�����ֵ��
	atomic_set(&dev->users, 0);
	atomic_set(&dev->nmappings, 0);

	dev->udev = usb_get_dev(udev);						//����usb�豸,�����������ü��� 
	dev->intf = usb_get_intf(intf);						//����usb�ӿ�,�����������ü���
	dev->intfnum = intf->cur_altsetting->desc.bInterfaceNumber;//��ȡusb�ӿ��������ӿ���
	dev->quirks = (uvc_quirks_param == -1)
		    ? id->driver_info : uvc_quirks_param;

	if (udev->product != NULL)								//���ڲ�Ʒ��
		strlcpy(dev->name, udev->product, sizeof dev->name);//����uvc�豸����Ϊ���Ʒ��
	else																			//ͨ�õ�uvc�豸��
		snprintf(dev->name, sizeof dev->name,
			"UVC Camera (%04x:%04x)",
			le16_to_cpu(udev->descriptor.idVendor),
			le16_to_cpu(udev->descriptor.idProduct));

	/* Parse the Video Class control descriptor. */
	if (uvc_parse_control(dev) < 0) {					//[@5]uvc����usb��Ƶ�����������,�����豸�Ŀ���������
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
	if (v4l2_device_register(&intf->dev, &dev->vdev) < 0)//����v4l2_device_register������ʼ��v4l2_dev �����������������ü���\\��ʼ��v4l2_dev->subdevs���豸ʵ��������Ȼ���������ֺ�����dev->driver_data
		goto error;

	/* Initialize controls. */
	if (uvc_ctrl_init_device(dev) < 0)					//[@8]uvc��ʼ��uvc�����豸 
		goto error;

	/* Scan the device for video chains. */
	if (uvc_scan_device(dev) < 0)								//[@10]uvcɨ����Ƶ��
		goto error;

	/* Register video device nodes. */
	if (uvc_register_chains(dev) < 0)						//[@11]uvcע����Ƶ�豸
		goto error;

	/* Save our data pointer in the interface data. */
	usb_set_intfdata(intf, dev);								//����uvc�豸Ϊusb�ӿڵ�����

	/* Initialize the interrupt URB. */
	if ((ret = uvc_status_init(dev)) < 0) {			//[@12]uvc�豸״̬��ʼ��
		uvc_printk(KERN_INFO, "Unable to initialize the status "
			"endpoint (%d), status interrupt will not be "
			"supported.\n", ret);
	}

	uvc_trace(UVC_TRACE_PROBE, "UVC device initialized.\n");
	usb_enable_autosuspend(udev);								//ʹ���Զ�����
	return 0;

error:
	uvc_unregister_video(dev);
	return -ENODEV;
}


/*
[@5]����uvc_parse_control���� 
���µ��ù�ϵ��

uvc_parse_control(dev)
    uvc_parse_standard_control(dev, buffer, buflen)
        uvc_parse_streaming(dev, intf)      
������uvc_parse_streaming������
*/
static int uvc_parse_streaming(struct uvc_device *dev,
    struct usb_interface *intf)
{
    /*���´󲿷�����ʡ�ԣ�ֻ��ʾ��Ҫ��*/
    struct uvc_streaming *streaming = NULL;
    struct uvc_format *format;
    struct uvc_frame *frame;

    streaming = kzalloc(sizeof *streaming, GFP_KERNEL);
    size = nformats * sizeof *format + nframes * sizeof *frame
         + nintervals * sizeof *interval;

    format = kzalloc(size, GFP_KERNEL);//����format�����Ÿ�ʽ
    streaming->format = format;//���ø�ʽ
    streaming->nformats = nformats;//���֧��nformats�ָ�ʽ
    ret = uvc_parse_format(dev, streaming, format,
                &interval, buffer, buflen);//������ʽ
    list_add_tail(&streaming->list, &dev->streams);
    return 0;
}
/*������������streaming��format�ڴ� 
streaming��uvc_streaming �ṹ�壬��Ƶ��������Ҫ���󲿷ֲ������Ǵ������档�⺯����������֮������˺ܶ����ã�����������ʡ����д�� 
format�ڴ��ŵ�����Ƶ�ĸ�ʽ��frame��ŵ�����ֱ��� 
�����涼�������õ���streaming���棨streaming->format = format;streaming->nformats = nformats;�� 
������uvc_parse_format����������ʽ��
*/
static int uvc_parse_format()
{
    fmtdesc = uvc_format_by_guid(&buffer[5]);//ͨ��GUID�ҵ���ʽformat
    /*���滹���frame���и��ַ��������ã�
     *������format->nframes�ó�����ж����ֱַ���ѡ��
     *��ʱ����*/
}
//����uvc_format_by_guid�������uvc_fmts������ͨ��ƥ��guid�ҵ���ʽ��

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
    /*����ʡ��......*/
}


/*
[@8]����uvc_ctrl_init_device
*/
int uvc_ctrl_init_device(struct uvc_device *dev)
{
    /*ʡ���˲�������*/
    list_for_each_entry(entity, &dev->entities, list) {
        bmControls = entity->extension.bmControls;//����λͼ
        bControlSize = entity->extension.bControlSize;//����λ���С
        entity->controls = kcalloc(ncontrols, sizeof(*ctrl),
                       GFP_KERNEL);//����ncontrols��uvc�����ڴ�
        if (entity->controls == NULL)
            return -ENOMEM;
        entity->ncontrols = ncontrols;//����uvc���Ƹ���

        /* Initialize all supported controls */
        ctrl = entity->controls;//ָ��uvc��������
        for (i = 0; i < bControlSize * 8; ++i) {
            if (uvc_test_bit(bmControls, i) == 0)//��������λ������0��
                continue;
            ctrl->entity = entity;
            ctrl->index = i;//���ÿ���λ������
            uvc_ctrl_init_ctrl(dev, ctrl);//��ʼ��uvc�ؼ�
            ctrl++;//uvc���� ָ����һ��uvc����������
        }
    }
}
/*uvc_ctrl_init_device��Ҫ���ǳ�ʼ�����Ʋ���������ͻ����uvc�豸ʵ��entities����Ȼ������λͼ��λ���С 
��󻹻����uvc_ctrl_init_ctrl�������ñ��⣬ɫ�µȵ�*/


/*
[@10]
*/


/*
[@11]����uvc_register_chains����: 
���ù�ϵ��
uvc_register_chains
    uvc_register_terms(dev, chain)
        uvc_stream_by_id
        uvc_register_video
    uvc_mc_register_entities(chain)
uvc_stream_by_id������ͨ�����������id��dev->streams�����header.bTerminalLinkƥ�䣬Ѱ�ҵ�stream 
�ⲻ���ص㣬���ǵ��ص���uvc_register_video�������ҵ�stream���Ҫע�᣺*/

static int uvc_register_video(struct uvc_device *dev,
        struct uvc_streaming *stream)
{
    /*��������ʡ��......*/
    struct video_device *vdev = &stream->vdev;

    ret = uvc_queue_init(&stream->queue, stream->type, !uvc_no_drop_param);//��ʼ������
    ret = uvc_video_init(stream);//��ʼ��

    uvc_debugfs_init_stream(stream);
    vdev->v4l2_dev = &dev->vdev;
    vdev->fops = &uvc_fops;//v4l2����������
    vdev->ioctl_ops = &uvc_ioctl_ops;//����������ioctl������
    vdev->release = uvc_release;//�ͷŷ���
    vdev->prio = &stream->chain->prio;

    strlcpy(vdev->name, dev->name, sizeof vdev->name);
    video_set_drvdata(vdev, stream);//��uvc��Ƶ����Ϊv4l2�豸����������

    ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);//ע��
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

/*���Ƿǳ���Ҫ�ĺ�����������һ��һ������� 
����uvc_queue_init���������г�ʼ���������ⶫ����������Ƶ����ʱ����õ�����ioctl�������*/

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
    ret = vb2_queue_init(&queue->queue);//��ʼ��queue

    mutex_init(&queue->mutex);
    spin_lock_init(&queue->irqlock);
    INIT_LIST_HEAD(&queue->irqqueue);//��ʼ��stream->queue->irqqueue
    queue->flags = drop_corrupted ? UVC_QUEUE_DROP_CORRUPTED : 0;

    return 0;
}
/*�����ȶԶ��н��г�ʼ�����ã�������type��ops�� 
����queue->queue.ops = &uvc_queue_qops�ǳ���Ҫ��֮�����ǵ���vidioc_streamon�ص�����ʱ���ǵ��õ������uvc_queue_qops�ṹ�����.start_streaming���� 
�⺯����Ը��ֶ��н����˳�ʼ����*/

vb2_queue_init(&queue->queue)
    q->buf_ops = &v4l2_buf_ops;
    vb2_core_queue_init(struct vb2_queue *q)
        INIT_LIST_HEAD(&q->queued_list);//stream->queue->queue->queued_list
        INIT_LIST_HEAD(&q->done_list);//stream->queue->done_list
INIT_LIST_HEAD(&queue->irqqueue);//��ʼ��stream->queue->irqqueue

//���Ǽ�������uvc_register_video������������ŵ�����uvc_video_init������ʼ��UVC��Ƶ�豸��

int uvc_video_init(struct uvc_streaming *stream)
{
    /*ʡ�Բ�������*/
    struct uvc_streaming_control *probe = &stream->ctrl;//��ȡuvc��������uvs���������ƶ���

    if (uvc_get_video_ctrl(stream, probe, 1, UVC_GET_DEF) == 0)//�ȵõ�����Ŀ��Ʋ���
        uvc_set_video_ctrl(stream, probe, 1);//������uvc��Ƶ����
    ret = uvc_get_video_ctrl(stream, probe, 1, UVC_GET_CUR);//�����getһ��
    for (i = stream->nformats; i > 0; --i) {
        format = &stream->format[i-1];//��ȡ��Ӧ��uvc��ʽ
        if (format->index == probe->bFormatIndex)
            break;
    }
    probe->bFormatIndex = format->index;//����uvc��Ƶ�����Ƶĸ�ʽ����Ϊuvc��ʽ������
    probe->bFrameIndex = frame->bFrameIndex;//����uvc��Ƶ�����Ƶķֱ�������Ϊuvc�ֱ��ʵ�����

    stream->def_format = format;
    stream->cur_format = format;//����uvc��ʽΪuvc��������cur_format��Ա
    stream->cur_frame = frame;//����uvc֡Ϊuvc��������cur_frame��Ա

    if (stream->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {//��Ƶ�ɼ�
        if (stream->dev->quirks & UVC_QUIRK_BUILTIN_ISIGHT)
            stream->decode = uvc_video_decode_isight;
        else if (stream->intf->num_altsetting > 1)
            stream->decode = uvc_video_decode_isoc;//ͬ����ʽ
        else
            stream->decode = uvc_video_decode_bulk;//bluk��ʽ
    } 
    return 0;
}
/*���������ݾͱȽ϶��ˣ��ȵõ���Ȼ������uvc�Ŀ��Ʋ�������������urb����usb���ݡ� 
Ȼ��ͨ��probe->bFormatIndex�����ҵ�ʹ�õ�format��ʽ��ͨ��probe->bFrameIndex�ҵ���Ӧ��frame�ֱ��ʣ�Ȼ�����õ�stream� 
���ѡ����뷽ʽ����ͬ����ʽ����bluk��ʽ�����뷽ʽ�����������ʱ���ص�����complete����á�

�ٴλص�uvc_register_video������û�취���������̫��Ҫ�ˣ� 
���������

    vdev->fops = &uvc_fops;//v4l2����������
    vdev->ioctl_ops = &uvc_ioctl_ops;//����������ioctl������
    vdev->release = uvc_release;//�ͷŷ���
    ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);

�������vdev->v4l2_dev = &dev->vdev;����v4l2_device����video_device����������Ҳ������������һ��ʼ�Ǹ�ͼ�����ġ� 
Ȼ������fops��������vdev->fops = &uvc_fops����Ȼ�ⲻ�Ǹ��û��ռ�ʹ�õ�open��read��write�������������vdev->cdev->ops��������õ����uvc_fops�ģ������û��ռ�ʵ���ϵ�pen��read��write�������ǻ�������á� Ȼ��ioctl�������������ǻ���õ�vdev->ioctl_ops = &uvc_ioctl_ops������˵��V4L2����Ҫ�ľ��Ǹ�����ʽ��ioctl�ˣ������Ȳ�������һ���ڷ��������� 
Ȼ�����վ������ǵ�ע�ắ���ˣ�video_register_device����õ�__video_register_device������*/
int __video_register_device(struct video_device *vdev, int type, int nr,
        int warn_if_nr_in_use, struct module *owner)
{
    /*ʡ�Բ��ֺ���*/
    vdev->minor = -1;//-1�������video device��δ��ע���
    switch (type) {//����typeѡ���豸����
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
    switch (type) {//ѡ��õ����豸��ƫ��ֵ
    case VFL_TYPE_GRABBER://������Ƶ����/����豸�� videoX
        minor_offset = 0;
        minor_cnt = 64;
        break;
    case VFL_TYPE_RADIO://���ڹ㲥��г���� radioX
        minor_offset = 64;
        minor_cnt = 64;
        break;
    case VFL_TYPE_VBI://���ڴ�ֱ�������ݵ� vbiX (���磬����ʽ��Ļ��ͼ�ĵ���)
        minor_offset = 224;
        minor_cnt = 32;
        break;
    default:
        minor_offset = 128;
        minor_cnt = 64;
        break;
    }
    nr = devnode_find(vdev, 0, minor_cnt);//��ȡһ��û�б�ʹ�õ��豸�ڵ����
    for (i = 0; i < VIDEO_NUM_DEVICES; i++)
        if (video_device[i] == NULL)//��video_device[]������ѡ��һ����ȱ������ȱ�������ֵ�ŵ�i��
            break;
    vdev->minor = i + minor_offset;//�豸�Ĵ��豸��
    video_device[vdev->minor] = vdev;//ע�⣺�����úõ�video_device���뵽video_device[]
    vdev->cdev->ops = &v4l2_fops;//�����û��ռ����������
    ret = cdev_add(vdev->cdev, MKDEV(VIDEO_MAJOR, vdev->minor), 1);//����ַ��豸��ϵͳ
    ret = device_register(&vdev->dev);//�豸ע��
    set_bit(V4L2_FL_REGISTERED, &vdev->flags);//��flags��0Ϊ����Ϊ1����ʾ���video_device��ע�������

    return 0;

}
/*��������һ�������������飺 
1.ȷ���豸���ƣ�Ҳ����������/dev/�����ɵ�video����radio֮��� 
2.�õ����豸��ƫ��ֵ 
3.�ҵ�һ���յ�video_device���飬��vdev���ȥ 
4.����vdev->cdev�������������vdev->cdev->ops = &v4l2_fops;��������������û��ռ��������,������ϸ�������� 
5.ע��video_device�豸 
6.���Ǳ�־��video_device��ע��

���11������uvc_register_chains�����ﻹ�����һ��uvc_mc_register_entities�����������������
uvc_mc_init_entity�����������v4l2_device_register_subdev����������ע��v4l2_subdev��ͬʱ��ʼ��
Ȼ�����ӵ�v4l2_dev->subdevs����
*/


/*
[@12]����uvc_status_init����
*/
int uvc_status_init(struct uvc_device *dev)
{
    /*ʡ�Բ��ֺ���*/
    struct usb_host_endpoint *ep = dev->int_ep;//��ȡusb_host_endpoint

    uvc_input_init(dev);//��ʼ��uvc�����豸,����ע��input�豸
    dev->status = kzalloc(UVC_MAX_STATUS_SIZE, GFP_KERNEL);//����urb�豸״̬�ڴ�
    dev->int_urb = usb_alloc_urb(0, GFP_KERNEL);//����urb
    pipe = usb_rcvintpipe(dev->udev, ep->desc.bEndpointAddress);//�ж�����˵�
    usb_fill_int_urb(dev->int_urb, dev->udev, pipe,
        dev->status, UVC_MAX_STATUS_SIZE, uvc_status_complete,
        dev, interval);//����ж�urb

    return 0;
}
/*������ǹ���urb��һЩ�����ˣ������ͺá�

��������û��ռ���ô�Ų����ģ� 
����__video_register_device������ģ�vdev->cdev->ops = &v4l2_fops;
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
    /*ʡ�Բ��ֺ���*/
    struct video_device *vdev;
    vdev = video_devdata(filp);//���ݴ��豸�Ŵ�video_devices[]�����еõ�video_device
    if (vdev->fops->open) {
        if (video_is_registered(vdev))
            ret = vdev->fops->open(filp);//ʵ�ʾ���vdev->fops
        else
            ret = -ENODEV;
    }
}
/*�ǵ�����֮ǰ��video_device���뵽video_device[]�𣿾�������ȡ�˳��� 
Ȼ�����vdev->fops->open(filp) 
vdev->fops����������uvc_register_video���������õģ� 
vdev->fops = &uvc_fops
*/
const struct v4l2_file_operations uvc_fops = {//ʵ�ʵ��û�����
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







