#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/fs.h>
#include <errno.h>
#include <string.h>

#define ADC_SET_CHANNEL         0xc000fa01
#define ADC_SET_ADCTSC          0xc000fa02

#define CHANNELCOUNT 6

int main(int argc,char **argv)
{
	int channels[CHANNELCOUNT] = {0,1,4,5,6,7}; //for 6410
	
	int channel;
	int i=0;
	fprintf(stderr, "press Ctrl-C to stop\n");
	
	int fd = open("/dev/adc", 0);//打开设备
	if (fd < 0) {
		perror("open ADC device:");
		return 1;
	}
	
	char output[255];
	for(;;)
	{
		 puts("\033[2J");//puts（）将buffer中的字符输出到标准输出，直到遇到空字符（’\0’)为止
		 output[0] = 0;
		 for(i=0;i<CHANNELCOUNT;i++)
		 {
		 		channel = channels[i];//轮询每个通道
		 	 if (ioctl(fd, ADC_SET_CHANNEL, channel) < 0) {
                perror("Can't set channel for /dev/adc!");
                close(fd);
                return 1;
       }

       char buffer[30];
       int len = read(fd, buffer, sizeof buffer -1);
            if (len > 0) {
                buffer[len] = '\0';
                int value = -1;
                sscanf(buffer, "%d", &value);//读取格式化的字符串中的数据。存入value
                char outbuff[255];
                sprintf(outbuff, "AIN%d %d\n", channel, value);
                strcat(output, outbuff);//把src所指字符串添加到dest结尾处(覆盖dest结尾处的'\0')。outbuff的数据放到output结尾
            } else {
                perror("read ADC device:");
                close(fd);
                return 1;
            }
		 }
		 printf("%s",output);
        usleep(300* 1000);
	}
	close(fd);
}