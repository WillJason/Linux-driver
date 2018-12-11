/*
��Ӣ���itop4412��CAN ������Ҫ SPI ����֧�֡�
������Դ���ǡ�net/can/af_can.c��.

CAN������һ���������Ϲ㷺���õ�����Э�飬�������Ϊ���������е�΢������ͨѶ��
LZ����֪ʶ���ޣ����ϳ�һ����ܵİɡ����£�CAN(Controller Area Network)���ߣ������������������ߣ�
��һ����Ч֧�ֲַ�ʽ���ƻ�ʵʱ���ƵĴ���ͨ�����硣����������ܡ��߿ɿ��ԡ������ص���ƺ����˵�
�۸���㷺Ӧ���ڹ�ҵ�ֳ����ơ�����¥�ҽ����е����ͨ�����Լ������������򣬲��ѱ�����Ϊ������
��ǰ;���ֳ�����֮һ��CAN���߹淶�Ѿ������ʱ�׼����֯�ƶ�Ϊ���ʱ�׼ISO11898��
���õ����ڶ�뵼���������̵�֧�֡�

��·��ص����ϲ�����������ˣ����Բ鿴���͵����ϡ�

Socket CAN����ƿ˷��˽�CAN�豸����ʵ��Ϊ�ַ��豸���������ľ����ԣ���Ϊ�ַ��豸����ֱ�Ӳ���������
Ӳ������֡���շ���֡���Ŷ��Լ�һЩ�߲㴫��Э��ֻ����Ӧ�ò�ʵ�֡����⣬�ַ��豸����ֻ�ܵ����̽���
���ʣ���֧�ֶ����ͬʱ������
SocketCAN����ƻ����µ�Э����PF_CAN.Э����PF_CANһ������Ӧ�ó����ṩSocket�ӿڣ���һ������������Linux
������ϵ�ṹ�е������֮�ϣ��Ӷ����Ը���Ч����Linux������ϵͳ�еĸ����ŶӲ��ԡ�CAN��������ע���һ��
�����豸�������Ŀ������յ���֡�Ϳ��Դ��͸�����㣬�������͵�CANЭ���岿�֣�֡���͵Ĺ��̵Ĵ��ݷ���
����෴����ͬʱ��Э����ģ���ṩ�����Э�鶯̬���غ�ж�ؽӿں��������õ�֧��ʹ�ø���CAN�����Э��
��ĿǰЭ������ֻ��������CANЭ�飺CAN_RAW��CAN_BCM��������Э���廹֧�ָ���CAN֡�Ķ��ģ�֧�ֶ������
ͬʱ����SOCKET CANͨ�ţ�ÿ�����̿���ͬʱʹ�ò�ͬЭ����и���֡���շ���

CAN��ϵͳ��
can��ϵͳʵ��Э����PF_CAN����Ҫ��������C�ļ���af_can.c��raw.c��bcm.c������af_can.c��������ϵͳ��
���Ĺ����ļ���raw.c��bcm.c�ֱ���raw��bcmЭ���ʵ���ļ���CAN��ϵͳ������ģ��Ĺ�ϵ��ͼ��

						BSD Socket Layer
								|
						CAN��ϵͳ
			CAN_RAW        CAN_BCM
								|
				  Network	Layer		
				  			|
				  	CAN�豸����
				  	
		af_can.c��Socket CAN�ĺ��Ĺ���ģ�顣can_creat()����CANͨ�������socket����Ӧ�ó������socket����
����socketʱ���ͻ�����ô˺�����can_proto_register(struct can_proto *)��can_proto_unregister(
struct can_proto *)�ɱ�������̬���غ�ж��CAN����Э�飬����Э���ڴ���struct can_proto��ʾ��
CAN_RAW��CAN_BCMЭ��ֱ������£�
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

can_rcv()��������������հ��Ĳ��������������Ӧ�Ĳ�������ͨ������struct packet_type��ָ����
 *
 * af_can module init/exit functions
 *

static struct packet_type can_packet __read_mostly = {
	.type = cpu_to_be16(ETH_P_CAN),
	.dev  = NULL,
	.func = can_rcv,
};
	can_rcv()�е���can_rcv_filter()���յ���CAN֡���й��˴���ֻ�����û�ͨ��can_rx_register()���ĵ�
CAN֡����can_rx_register()��Ӧ�ĺ���can_rx_unregister��������ȡ���û����ĵ�CAN֡��
	raw.c��bcm.c�ֱ���Э����������ʵ��raw.socket��bcm.socketͨ��������ļ�������CAN_RAW��CAN_BCMЭ��
����ʵ��֡ID�Ķ��ģ���������CAN_BCMЭ�黹�ܽ���֡���ݵĹ��ˡ�
	���У�raw_rcv()��bcm_send_to_user()�Ǵ������������CAN֡�Ĳ�����������can_rcv()���յ��û����ĵ�֡
ʱ���ͻ��������������֡�ŵ����ն����У�Ȼ��raw_recvmsg()��bcm_recvmsg()�ͻ�ͨ������
skb_recv_datagram()���ӽ��ն�����ȡ��֡���Ӷ���֡��һ�����͵��ϲ㣻
	

*/












