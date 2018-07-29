#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


#define PWM_IOCTL_SET_FREQ		1
#define PWM_IOCTL_STOP			0

#define	ESC_KEY		0x1b


static int getch(void)
{
	struct termios oldt,newt;
	int ch;

	//主要功能是检查设备类型 , 判断文件描述词是否是为终端机。
	if (!isatty(STDIN_FILENO)) {
		fprintf(stderr, "this problem should be run at a terminal\n");
		exit(1);
	}
	// save terminal setting tcgetattr函数用于获取与终端相关的参数。
	//参数fd为终端的文件描述符,返回的结果保存在termios结构体中
	if(tcgetattr(STDIN_FILENO, &oldt) < 0) {
		perror("save the terminal setting");
		exit(1);
	}

	// set terminal as need
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );//使用标准输入模式.显示输入字符
	//TCSANOW：不等数据传输完毕就立即改变属性。
	if(tcsetattr(STDIN_FILENO,TCSANOW, &newt) < 0) {
		perror("set terminal");
		exit(1);
	}

	ch = getchar();

	// restore termial setting
	if(tcsetattr(STDIN_FILENO,TCSANOW,&oldt) < 0) {
		perror("restore the termial setting");
		exit(1);
	}
	return ch;
}


static int fd = -1;
static void close_pwm(void);
static void open_pwm(void)
{
	fd = open("/dev/pwm", 0);
	if (fd < 0) {
		perror("open pwm_buzzer device");
		exit(1);
	}

	// any function exit call will stop the buzzer
	atexit(close_pwm);//登记函数 ,正常程序退出时调用的函数
}

static void close_pwm(void)
{
	if (fd >= 0) {
		ioctl(fd, PWM_IOCTL_STOP);
		if (ioctl(fd, 2) < 0) {
			perror("ioctl 2:");
		}
		close(fd);
		fd = -1;
	}
}

static void set_pwm_freq(int freq)
{
	// this IOCTL command is the key to set frequency
	int ret = ioctl(fd, PWM_IOCTL_SET_FREQ, freq);//设置默认频率
	if(ret < 0) {
		perror("set the frequency of the buzzer");
		exit(1);
	}
}

static void stop_pwm(void)
{
	int ret = ioctl(fd, PWM_IOCTL_STOP);
	if(ret < 0) {
		perror("stop the buzzer");
		exit(1);
	}
	if (ioctl(fd, 2) < 0) {
		perror("ioctl 2:");
	}
}

int main(int argc,char **argv)
{
	int freq = 1000;
	
	open_pwm();//打开设备
	
	printf( "\nBUZZER TEST ( PWM Control )\n" );
	printf( "Press +/- to increase/reduce the frequency of the BUZZER\n" ) ;
	printf( "Press 'ESC' key to Exit this program\n\n" );
	
	while(1)
	{
		int key;
		
		set_pwm_freq(freq);
		printf( "\tFreq = %d\n", freq );
		
		key = getch();
		
		switch(key) {
		case '+':
			if( freq < 20000 )
				freq += 10;
			break;

		case '-':
			if( freq > 11 )
				freq -= 10 ;
			break;

		case ESC_KEY:
		case EOF:
			stop_pwm();
			exit(0);

		default:
			break;
		}
	}
}