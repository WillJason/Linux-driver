/*
485�豸���������ַ��豸��������������ӦԴ�롰drivers/char/max485_ctl.c������Ӧ�豸�ڵ�
��/dev/max485_ctl_pin����
�����ļ��ڣ� ��kernel/drivers/char/Ŀ¼�¡�
max485_crtl.c ����-�������� 
*/
////��Ҫ���485��ƽ̨�豸��Ϣ
static void __init smdk4x12_machine_init(void)
{
	platform_add_devices(smdk4x12_devices, ARRAY_SIZE(smdk4x12_devices));//ע��ƽ̨�豸
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

//�ں�����
Device Drivers --->
	Character devices --->
		Enable MAX485 pin config

/*-----------------------------------------------------------------------------------
samsung max485_crtl.c ����
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
	һ��gpio_request��װ��mem_request(),�𱣻����ã����Ҫ����mem_free֮��ġ���Ҫ�Ǹ����ں����ַ��ռ���ˡ�
	�������ط�����ͬһ��ַ��gpio_request�ͻᱨ����󣬸õ�ַ�ѱ����롣��/proc/memӦ�û��е�ַռ�ñ�������
	�����÷��ı�������ǰ���Ǵ�Ҷ������������ٷ��ʣ���һ���ط�û������������⹦�ܾ�ʧЧ�ˡ��ñȽ��̻��⣬
	�������ڷ����ٽ���Դ��ʱ�򶼵��Ȼ�ȡ��һ��������һ��û����Լ��������ͷ��ˡ�
	gpio��Ϊ��Ҫ�������һ���ܽţ�label����Ϊ��ȡһ�����֡�
	*/
	err = gpio_request(EXYNOS4_GPA0(7), "GPA0_7");
	if (err) {
		printk(KERN_ERR "failed to request GPA0_7 for "
			"max485_ctl control\n");
		return err;
	}
	/*��ĳ��GPIO��д��ĳ��ֵ֮�󣬻��������˿�����Ϊ���ģʽ*/
	gpio_direction_output(EXYNOS4_GPA0(7), 1);

	s3c_gpio_cfgpin(EXYNOS4_GPA0(7), S3C_GPIO_OUTPUT);//����Ϊ���ģʽ
	gpio_free(EXYNOS4_GPA0(7));//�ͷ�GPIO port ��ʹ��Ȩ,��gpio ָ������ port

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

/*���ǿ��Կ���GPIO֮ǰ��probe�������ͷţ�ioctl������������ע������*/
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
//���Գ���test_485.c

/*
	test_485 ������Ҫ�����������һ��������ѡ����ԵĴ��ڣ��ڶ��������øÿ��Ƿ������ݻ���
�������ݡ�ȫ�ܰ����ӿ��� 485 �Ĵ����� ttySAC1�����Ե�һ������Ϊ ttySAC1������ʱ����ֱ�
Ϊ��./test_485 /dev/ttySAC1 0���͡�./test_485 dev/ttySAC1 0����
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









