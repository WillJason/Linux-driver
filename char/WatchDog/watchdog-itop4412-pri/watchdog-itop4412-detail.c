/*
	将编译得出的测试程序“topeet_watchdogtest”拷贝到开发板。 测试程序第一个参数
为看门狗设备节点，第二个参数为看门狗重启时间。
	例如：./topeet_watchdogtest /dev/watchdog 5
	程序执行之后，如下图所示，程序会先启动看门狗，然后喂狗，最后停止喂狗，开发板重
启。

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


//iTOP-4412 开发板内核源码中的看门狗没有配置时钟，另外驱动源码需要修改。
//在内核源码中，使用命令“vim arch/arm/mach-exynos/clock-exynos4.c”打开时钟配
//置文件。在数组中“static struct clk exynos4_init_clocks[]”中添加如下图所示代码。
{
.name = "watchdog",
.parent = &exynos4_clk_pclk_acp,
.enable = exynos4_clk_ip_perir_ctrl,
.ctrlbit = (1 << 14),
}

