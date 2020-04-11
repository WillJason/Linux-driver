/*
硬件平台：s3c2440
系统：linux-2.6.22.6
cpu在同一时间只能做一件事情，数据很大话这段时间就不能做其他事情了。
所以我们引入另外一个东西叫DMA，DMA就是direct memory access 内存直接访问，
刚才的过程由DMA来做，我把这个源src告诉DMA，把这个目的dst告诉DMA，长度告诉
DMA，然后启动DMA。启动之后就会自动把数据进行拷贝。拷贝完成之后会产生一个
中断给CPU。
*/
#define MEM_CPY_NO_DMA  0
#define MEM_CPY_DMA     1

#define BUF_SIZE  (512*1024)

#define DMA0_BASE_ADDR  0x4B000000
#define DMA1_BASE_ADDR  0x4B000040
#define DMA2_BASE_ADDR  0x4B000080
#define DMA3_BASE_ADDR  0x4B0000C0

struct s3c_dma_regs {
	unsigned long disrc;
	unsigned long disrcc;
	unsigned long didst;
	unsigned long didstc;
	unsigned long dcon;
	unsigned long dstat;
	unsigned long dcsrc;
	unsigned long dcdst;
	unsigned long dmasktrig;
};


static int major = 0;

static char *src;
static u32 src_phys;

static char *dst;
static u32 dst_phys;

static struct class *cls;

static volatile struct s3c_dma_regs *dma_regs;

static DECLARE_WAIT_QUEUE_HEAD(dma_waitq);
/* 中断事件标志, 中断服务程序将它置1，ioctl将它清0 */
static volatile int ev_dma = 0;

static int s3c_dma_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int i;

	memset(src, 0xAA, BUF_SIZE);
	memset(dst, 0x55, BUF_SIZE);
	
	switch (cmd)
	{
		case MEM_CPY_NO_DMA :
		{
			for (i = 0; i < BUF_SIZE; i++)
				dst[i] = src[i];
			if (memcmp(src, dst, BUF_SIZE) == 0)
			{
				printk("MEM_CPY_NO_DMA OK\n");
			}
			else
			{
				printk("MEM_CPY_DMA ERROR\n");
			}
			break;
		}

		case MEM_CPY_DMA :
		{
			ev_dma = 0;
			
			/* 把源,目的,长度告诉DMA */
			dma_regs->disrc      = src_phys;        /* 源的物理地址 */
			dma_regs->disrcc     = (0<<1) | (0<<0); /* 源位于AHB总线, 源地址递增 */
			dma_regs->didst      = dst_phys;        /* 目的的物理地址 */
			dma_regs->didstc     = (0<<2) | (0<<1) | (0<<0); /* 目的位于AHB总线, 目的地址递增 */
			dma_regs->dcon       = (1<<30)|(1<<29)|(0<<28)|(1<<27)|(0<<23)|(0<<20)|(BUF_SIZE<<0);  /* 使能中断,单个传输,软件触发, */

			/* 启动DMA */
			dma_regs->dmasktrig  = (1<<1) | (1<<0);

			/* 如何知道DMA什么时候完成? */
			/* 休眠 */
			wait_event_interruptible(dma_waitq, ev_dma);

			if (memcmp(src, dst, BUF_SIZE) == 0)
			{
				printk("MEM_CPY_DMA OK\n");
			}
			else
			{
				printk("MEM_CPY_DMA ERROR\n");
			}
			
			break;
		}
	}

	return 0;
}

static struct file_operations dma_fops = {
	.owner  = THIS_MODULE,
	.ioctl  = s3c_dma_ioctl,
};

static irqreturn_t s3c_dma_irq(int irq, void *devid)
{
	/* 唤醒 */
	ev_dma = 1;
    wake_up_interruptible(&dma_waitq);   /* 唤醒休眠的进程 */
	return IRQ_HANDLED;
}

static int s3c_dma_init(void)
{
	if (request_irq(IRQ_DMA3, s3c_dma_irq, 0, "s3c_dma", 1))
	{
		printk("can't request_irq for DMA\n");
		return -EBUSY;
	}
	
	/* 分配SRC, DST对应的缓冲区 */
	src = dma_alloc_writecombine(NULL, BUF_SIZE, &src_phys, GFP_KERNEL);
	if (NULL == src)
	{
		printk("can't alloc buffer for src\n");
		free_irq(IRQ_DMA3, 1);
		return -ENOMEM;
	}
	
	dst = dma_alloc_writecombine(NULL, BUF_SIZE, &dst_phys, GFP_KERNEL);
	if (NULL == dst)
	{
		free_irq(IRQ_DMA3, 1);
		dma_free_writecombine(NULL, BUF_SIZE, src, src_phys);
		printk("can't alloc buffer for dst\n");
		return -ENOMEM;
	}

	major = register_chrdev(0, "s3c_dma", &dma_fops);

	/* 为了自动创建设备节点 */
	cls = class_create(THIS_MODULE, "s3c_dma");
	class_device_create(cls, NULL, MKDEV(major, 0), NULL, "dma"); /* /dev/dma */

	dma_regs = ioremap(DMA3_BASE_ADDR, sizeof(struct s3c_dma_regs));
		
	return 0;
}

static void s3c_dma_exit(void)
{
	iounmap(dma_regs);
	class_device_destroy(cls, MKDEV(major, 0));
	class_destroy(cls);
	unregister_chrdev(major, "s3c_dma");
	dma_free_writecombine(NULL, BUF_SIZE, src, src_phys);
	dma_free_writecombine(NULL, BUF_SIZE, dst, dst_phys);	
	free_irq(IRQ_DMA3, 1);
}
/*参考的应用测试代码*/
/* ./dma_test nodma
 * ./dma_test dma
 */
#define MEM_CPY_NO_DMA  0
#define MEM_CPY_DMA     1

void print_usage(char *name)
{
	printf("Usage:\n");
	printf("%s <nodma | dma>\n", name);
}


int main(int argc, char **argv)
{
	int fd;
	
 	if (argc != 2)
	{
		print_usage(argv[0]);
		return -1;
	}

	fd = open("/dev/dma", O_RDWR);
	if (fd < 0)
	{
		printf("can't open /dev/dma\n");
		return -1;
	}

	if (strcmp(argv[1], "nodma") == 0)
	{
		while (1)
		{
			ioctl(fd, MEM_CPY_NO_DMA);
		}
	}
	else if (strcmp(argv[1], "dma") == 0)
	{
		while (1)
		{
			ioctl(fd, MEM_CPY_DMA);
		}
	}
	else
	{
		print_usage(argv[0]);
		return -1;
	}
	return 0; 	
}
/*
#./dma_test nodma &
未使用DMA，这个程序一直在消耗CPU资源，我们执行其他程序需要很久才能执行
#./dma_test dma &
现在我们执行其他命令，比如ls，很快就执行了
*/














