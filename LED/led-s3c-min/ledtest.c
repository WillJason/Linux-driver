
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>

/*
  *  ledtest <dev> <on|off>
  主设备号是帮助我们在内核的数组里面找到那一项，根据那一项找到我们的设备驱动，但次设备号怎么用还没有说，次设备号由我们来管理
  */

void print_usage(char *file)
{
    printf("Usage:\n");
    printf("%s <dev> <on|off>\n",file);
    printf("eg. \n");
    printf("%s /dev/leds on\n", file);
    printf("%s /dev/leds off\n", file);
    printf("%s /dev/led1 on\n", file);
    printf("%s /dev/led1 off\n", file);
}

int main(int argc, char **argv)
{
    int fd;
    char* filename;
    char val;

    if (argc != 3)
    {
        print_usage(argv[0]);
        return 0;
    }

    filename = argv[1];

    fd = open(filename, O_RDWR);
    if (fd < 0)
    {
        printf("error, can't open %s\n", filename);
        return 0;
    }

    if (!strcmp("on", argv[2]))
    {
        // 亮灯
        val = 0;
        write(fd, &val, 1);
    }
    else if (!strcmp("off", argv[2]))
    {
        // 灭灯
        val = 1;
        write(fd, &val, 1);
    }
    else
    {
        print_usage(argv[0]);
        return 0;
    }
    
    
    return 0;
}

