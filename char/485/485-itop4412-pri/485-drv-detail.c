/*
485设备驱动属于字符设备驱动。该驱动对应源码“drivers/char/max485_ctl.c“，对应设备节点
“/dev/max485_ctl_pin”。
驱动文件在： 在kernel/drivers/char/目录下。
max485_crtl.c ——-总线驱动 
*/
////需要添加485的平台设备信息
static void __init smdk4x12_machine_init(void)
{
	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));//注册平台设备
}

static struct platform_device *smdk4x12_devices[] __initdata = {

	#ifdef CONFIG_MAX485_CTL
	&s3c_device_max485_ctl ,
	#endif
}

#ifdef CONFIG_MAX485_CTL
struct platform_device s3c_device_max485_ctl = {
        .name   = "max485_ctl",
        .id             = -1,
};
#endif

//内核配置
Device Drivers --->
	Character devices --->
		Enable MAX485 pin config

/*-----------------------------------------------------------------------------------
samsung max485_crtl.c 分析
-----------------------------------------------------------------------------------*/
static struct platform_driver max485_ctl_driver = {
	.probe = max485_ctl_probe,
	.remove = max485_ctl_remove,
	.suspend = max485_ctl_suspend,
	.resume = max485_ctl_resume,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init max485_ctl_init(void)
{
	return platform_driver_register(&max485_ctl_driver);
}


static int max485_ctl_probe(struct platform_device *pdev)
{
	int err = 0;
	
	int ret;
	char *banner = "max485_ctl Initialize\n";

	printk(banner);

	/*
	一般gpio_request封装了mem_request(),起保护作用，最后要调用mem_free之类的。主要是告诉内核这地址被占用了。
	当其它地方调用同一地址的gpio_request就会报告错误，该地址已被申请。在/proc/mem应该会有地址占用表描述。
	这种用法的保护作用前提是大家都遵守先申请再访问，有一个地方没遵守这个规则，这功能就失效了。好比进程互斥，
	必需大家在访问临界资源的时候都得先获取锁一样，其中一个没遵守约定，代码就废了。
	gpio则为你要申请的哪一个管脚，label则是为其取一个名字。
	*/
	err = gpio_request(EXYNOS4_GPA0(7), "GPA0_7");
	if (err) {
		printk(KERN_ERR "failed to request GPA0_7 for "
			"max485_ctl control\n");
		return err;
	}
	/*在某个GPIO口写上某个值之后，还会把这个端口设置为输出模式*/
	gpio_direction_output(EXYNOS4_GPA0(7), 1);

	s3c_gpio_cfgpin(EXYNOS4_GPA0(7), S3C_GPIO_OUTPUT);//配置为输出模式
	gpio_free(EXYNOS4_GPA0(7));//释放GPIO port 的使用权,由gpio 指定具体 port

	ret = misc_register(&max485_ctl_dev);
	if(ret<0)
	{
		printk("max485_ctl:register device failed!\n");
		goto exit;
	}

	return 0;

exit:
	misc_deregister(&max485_ctl_dev);
	return ret;
}


static struct file_operations max485_ctl_ops = {
	.owner 	= THIS_MODULE,
	.open 	= max485_ctl_open,
	.release= max485_ctl_release,
	.unlocked_ioctl 	= max485_ctl_ioctl,
};

static struct miscdevice max485_ctl_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.fops	= &max485_ctl_ops,
	.name	= "max485_ctl_pin",
};

/*我们可以看到GPIO之前在probe函数被释放，ioctl函数中又重新注册申请*/
long max485_ctl_ioctl(struct file *filp,unsigned int cmd,unsigned long arg)
{
	printk("firecxx debug: max485_ctl_ioctl cmd is %d\n" , cmd);

	switch(cmd)
	{		
		case 1:
			if(gpio_request(EXYNOS4_GPA0(7) ,"GPA0_7"))
			{
				DPRINTK("max485_ctl GPIO err!\r\n");
			}
			else
			{
				gpio_direction_output(EXYNOS4_GPA0(7), 1);
				DPRINTK("max485_ctl Set High!\n");
				gpio_free(EXYNOS4_GPA0(7));

				mdelay(100);
			}
				
			break;
		case 0:
			if(gpio_request(EXYNOS4_GPA0(7) ,"GPA0_7"))
			{
				DPRINTK("max485_ctl GPIO err!\r\n");
			}
			else
			{			
				gpio_direction_output(EXYNOS4_GPA0(7),0);
				DPRINTK("max485_ctl Set Low!\n");
				gpio_free(EXYNOS4_GPA0(7));

				mdelay(100); 
			}
			
			break;
			
		default:
			DPRINTK("max485_ctl COMMAND ERROR!\n");
			return -ENOTTY;
	}
	return 0;
}
//测试程序：test_485.c

/*
	test_485 程序需要输入参数，第一个参数是选择测试的串口，第二个是设置该口是发送数据还是
接收数据。全能版连接控制 485 的串口是 ttySAC1，所以第一个参数为 ttySAC1。运行时命令分别
为“./test_485 /dev/ttySAC1 0”和“./test_485 dev/ttySAC1 0”。
*/
//#include <stdio.h>
#include <unistd.h>
//#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>

#define MAX485_CONTROL

//#include "uart.c"
int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio,oldtio;
	if  ( tcgetattr( fd,&oldtio)  !=  0) { 
		perror("SetupSerial 1");
		return -1;
	}
	bzero( &newtio, sizeof( newtio ) );
	newtio.c_cflag  |=  CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

	switch( nBits )
	{
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	}

	switch( nEvent )
	{
	case 'O':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	case 'E': 
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 'N':  
		newtio.c_cflag &= ~PARENB;
		break;
	}

	printf("Baund Rate: %d\n", nSpeed);

	switch( nSpeed )
	{
	case 2400:
		cfsetispeed(&newtio, B2400);
		cfsetospeed(&newtio, B2400);
		break;
	case 4800:
		cfsetispeed(&newtio, B4800);
		cfsetospeed(&newtio, B4800);
		break;
	case 9600:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	case 115200:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	case 460800:
		cfsetispeed(&newtio, B460800);
		cfsetospeed(&newtio, B460800);
		break;
	case 921600:
		printf("Rate:921600\n");
		cfsetispeed(&newtio, B921600);
                cfsetospeed(&newtio, B921600);
                break;
	default:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	}
	if( nStop == 1 )
		newtio.c_cflag &=  ~CSTOPB;
	else if ( nStop == 2 )
	newtio.c_cflag |=  CSTOPB;
	newtio.c_cc[VTIME]  = 0;
	newtio.c_cc[VMIN] = 0;
	tcflush(fd,TCIFLUSH);
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)
	{
		perror("com set error");
		return -1;
	}
//	printf("set done!\n\r");
	return 0;
}

int prepare_to_send(int fd)
{
	int ret;

	ret = ioctl(fd, 1, 0);
	if(ret<0)
	{
		printf("max485 set ctl to high failed!\r\n");

		return -1;
	}
	else
	{
		return 0;
	}
}

int prepare_to_recv(int fd)
{
	int ret;

	ret = ioctl(fd, 0, 0);
	if(ret<0)
	{
		printf("max485 set ctl to low failed!\r\n");
		
		return -1;
	}
	else
	{
		return 0;
	}
}

void main(int argc, char* argv[])
{
	unsigned char ucTmp;
	int fd1,fd2,nset1,nset2,nread;

	char buf[100];
	//char buf1[1];

	//char *buff = "Hello\n\r";

	int i = 0;

	char *max485_ctl = "/dev/max485_ctl_pin";

	if(3 != argc)
    	{
		printf("Usage:	test_485 [uart port] [type]\r\n");
		printf("		type: 0--recv, 1--send\r\n");
        	
		return;
	}

	fd1 = open(argv[1], O_RDWR);
	if (fd1 == -1)
	{
		printf("Open %s faild\n", argv[1]);
		exit(1);
	}

	nset1 = set_opt(fd1, 9600, 8, 'N', 1);
	if (nset2 == -1)
	{
		printf("Set uart faild\n");
		exit(1);
	}

#ifdef MAX485_CONTROL
	if((fd2=open(max485_ctl, O_RDWR|O_NOCTTY|O_NDELAY))<0)
	{
		printf("Open %s faild\n", max485_ctl);
		close(fd1);
		
		exit(1);
	}
#endif
	
	if(0 == atoi(argv[2]))	//recv
	{
#ifdef MAX485_CONTROL
		prepare_to_recv(fd2);
#endif
		while(1)
		{	
			
			nread = read(fd1, buf, 100);
			if (nread > 0)
			{
				for(i=0; i<nread; i++)
				{
					printf("%c", buf[i]);
			
					if(buf[i] == 'q')
						//break;
						goto exit;
				}
			}
			//if(nread)
			//{
			//	printf("\r\n");
			//}
			sleep(1);
		}
	}
	else	//send 
	{
		#ifdef MAX485_CONTROL
			prepare_to_send(fd2);
		#endif
		
		while(1)
		{
			printf("Send data, time:%d\r\n", i);
			sprintf(buf, "iTOP-4412: max485 test app(times:%d)\r\n", i++);
			//nread = write(fd1, "iTOP-4412: max485 test app\r\n", strlen("iTOP-4412: max485 test app\r\n"));
			nread = write(fd1, buf, strlen(buf));
			sleep(1);
			#if 0
				nread = read(fd1, buf, 100);
			 	if (nread > 0)
        {
           for(i=0; i<nread; i++)
           {
               printf("%c", buf[i]);
               if(buf[i] == 'q')
               //break;
               goto exit;
        	}
        }
        if(nread)
        {
           printf("\r\n");
        }
			#endif
		}
	}
	exit:
	close(fd1);

  return;
}









