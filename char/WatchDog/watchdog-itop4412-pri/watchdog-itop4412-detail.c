/*
	������ó��Ĳ��Գ���topeet_watchdogtest�������������塣 ���Գ����һ������
Ϊ���Ź��豸�ڵ㣬�ڶ�������Ϊ���Ź�����ʱ�䡣
	���磺./topeet_watchdogtest /dev/watchdog 5
	����ִ��֮������ͼ��ʾ����������������Ź���Ȼ��ι�������ֹͣι������������
����

*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/watchdog.h>

int main(int argc,char *argv[])
{
	int fd,timeout,count;

	if(argc !=3){
		printf("usage:\n\
argv1:watchdog dev nodes\n\
argv2:timeout such as 5,6,7..max-timeout\n");
		return -1;
	}else{
		printf("parameters1 is %s,parameters2 is %s\n",argv[1],argv[2]);
	}
	
	fd = open(argv[1], O_WRONLY);
	if (fd == -1){
		perror("watchdog ");
		exit(EXIT_FAILURE);
	}
	
	timeout = atoi(argv[2]);
	if(timeout<=3){
		printf("argv2 is small,you can try 5\n");
		return -1;
	}
	
	ioctl(fd, WDIOC_SETTIMEOUT, &timeout);
	ioctl(fd, WDIOC_GETTIMEOUT, &timeout);
	printf("watchdog settime is %d\n",timeout);
	
	count = timeout;
	while(--count){
		sleep(1);
		printf("feel dog in %d second\n",count);	
	}
	ioctl(fd,WDIOC_KEEPALIVE,&timeout);
	printf("MY GOD! I feel dog %d second\n",timeout);
	
	count = timeout;
	while(1){
		printf("feel dog in %d second\r\n",count--);
		sleep(1);
	}
	close(fd);
	return 0;
}


//iTOP-4412 �������ں�Դ���еĿ��Ź�û������ʱ�ӣ���������Դ����Ҫ�޸ġ�
//���ں�Դ���У�ʹ�����vim arch/arm/mach-exynos/clock-exynos4.c����ʱ����
//���ļ����������С�static struct clk exynos4_init_clocks[]�����������ͼ��ʾ���롣
{
.name = "watchdog",
.parent = &exynos4_clk_pclk_acp,
.enable = exynos4_clk_ip_perir_ctrl,
.ctrlbit = (1 << 14),
}

