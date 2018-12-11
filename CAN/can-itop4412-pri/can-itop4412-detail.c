/*
精英板的itop4412的CAN 驱动需要 SPI 总线支持。
驱动的源码是“net/can/af_can.c”.

CAN总线是一种在汽车上广泛采用的总线协议，被设计作为汽车环境中的微控制器通讯。
LZ理论知识有限，网上抄一句介绍的吧。如下：CAN(Controller Area Network)总线，即控制器局域网总线，
是一种有效支持分布式控制或实时控制的串行通信网络。由于其高性能、高可靠性、及独特的设计和适宜的
价格而广泛应用于工业现场控制、智能楼宇、医疗器械、交通工具以及传感器等领域，并已被公认为几种最
有前途的现场总线之一。CAN总线规范已经被国际标准化组织制订为国际标准ISO11898，
并得到了众多半导体器件厂商的支持。

电路相关的资料不在这里介绍了，可以查看博客等资料。

Socket CAN的设计克服了将CAN设备驱动实现为字符设备驱动带来的局限性，因为字符设备驱动直接操作控制器
硬件进行帧的收发、帧的排队以及一些高层传输协议只能在应用层实现。另外，字符设备驱动只能单进程进行
访问，不支持多进程同时操作。
SocketCAN的设计基于新的协议族PF_CAN.协议族PF_CAN一方面向应用程序提供Socket接口，另一方面它构建在Linux
网络体系结构中的网络层之上，从而可以更有效利用Linux网络子系统中的各种排队策略。CAN控制器被注册成一个
网络设备，这样的控制器收到的帧就可以传送给网络层，进而传送到CAN协议族部分（帧发送的过程的传递方向
与此相反）。同时，协议族模块提供传输层协议动态加载和卸载接口函数，更好地支持使用各种CAN传输层协议
（目前协议族中只包括两种CAN协议：CAN_RAW和CAN_BCM）。此外协议族还支持各种CAN帧的订阅，支持多个进程
同时进行SOCKET CAN通信，每个进程可以同时使用不同协议进行各种帧的收发。

CAN子系统：
can子系统实现协议族PF_CAN，主要包括三个C文件：af_can.c、raw.c和bcm.c。其中af_can.c是整个子系统的
核心管理文件，raw.c和bcm.c分别是raw和bcm协议的实现文件。CAN子系统与其他模块的关系如图：

						BSD Socket Layer
								|
						CAN子系统
			CAN_RAW        CAN_BCM
								|
				  Network	Layer		
				  			|
				  	CAN设备驱动
				  	
		af_can.c是Socket CAN的核心管理模块。can_creat()创建CAN通信所需的socket。当应用程序调用socket（）
创建socket时，就会简介调用此函数：can_proto_register(struct can_proto *)和can_proto_unregister(
struct can_proto *)可被用来动态加载和卸载CAN传输协议，传输协议在此用struct can_proto表示。
CAN_RAW和CAN_BCM协议分别定义如下：
	static const struct can_proto raw_can_proto = {
	.type       = SOCK_RAW,
	.protocol   = CAN_RAW,
	.ops        = &raw_ops,
	.prot       = &raw_proto,
};

static const struct can_proto bcm_can_proto = {
	.type       = SOCK_DGRAM,
	.protocol   = CAN_BCM,
	.ops        = &bcm_ops,
	.prot       = &bcm_proto,
};

can_rcv()是网络层用来接收包的操作函数，与包对应的操作函数通过定义struct packet_type来指明：
 *
 * af_can module init/exit functions
 *

static struct packet_type can_packet __read_mostly = {
	.type = cpu_to_be16(ETH_P_CAN),
	.dev  = NULL,
	.func = can_rcv,
};
	can_rcv()中调用can_rcv_filter()对收到的CAN帧进行过滤处理，只接收用户通过can_rx_register()订阅的
CAN帧；与can_rx_register()对应的函数can_rx_unregister（）用来取消用户订阅的CAN帧。
	raw.c和bcm.c分别是协议族里用来实现raw.socket和bcm.socket通信所需的文件。利用CAN_RAW和CAN_BCM协议
均能实现帧ID的订阅，并且利用CAN_BCM协议还能进行帧内容的过滤。
	其中，raw_rcv()和bcm_send_to_user()是传输层用来接收CAN帧的操作函数，当can_rcv()接收到用户订阅的帧
时，就会调用这两函数将帧放到接收队列中，然后raw_recvmsg()或bcm_recvmsg()就会通过调用
skb_recv_datagram()来从接收队列中取出帧，从而把帧进一步传送到上层；
	

*/












