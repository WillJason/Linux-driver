#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#define BUZZER_C 2

int main(int argc,char *argv[v])
{
	char *buzzer_crl = "/dev/buzzer_crt";
	int fd;
	
	buzzer_c = BUZZER_C;
	
	if(atoi(argv[1]) >= buzzer_c)//���ַ���ת������������һ������
	{
		printf("argv[1] is 0 or 1\n");
		exit(1);
	}
	
	if((fd=open(buzzer_crl,O_RDWR|O_NOCTTY|O_NDELAY)) < 0)
		//�ڶ�����ʱ��������������ݣ�O_NDELAY��ʹI/O�������Ϸ���0,����O_NONBLOCK
	{
		printf("open %s failed\n",buzzer_ctl);
		exit(1);
	}
	ret = ioctl(fd,atoi(argv[1]),atoi(argv[2]));
	close(fd);
	
	return 0;
}