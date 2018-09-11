#include <rtthread.h>
#include <stdio.h>
#include <string.h>

rt_device_t uart_2,uart_1;		//�豸��ʶ��uart_2 ���������� ��uart_1 �򴮿�д����

static struct rt_semaphore sem_1;		
static struct rt_messagequeue msgqueue_1;		//���о��
static char msgqueue_pool_1[1024]={0};

static char read_buf[64]={0};

struct rx_msg{
	rt_device_t dev;
	rt_size_t size;
};

rt_err_t uart2_rx(rt_device_t dev, rt_size_t size){		//���������ص�����
	struct rx_msg msg;
	rt_kprintf("uart2 callback receive byte: %d\r\n",size);
	
	msg.dev=dev;
	msg.size=size;
	
	rt_mq_send(&msgqueue_1,&msg,sizeof(struct rx_msg));		//���͵�����
	return RT_EOK;
}


#define thread_prio_1	10
struct rt_thread thread_1;
static rt_uint8_t thread_stack_1[256]={0};
rt_uint32_t thread_stack_size_1=sizeof(thread_stack_1);

#define thread_prio_2	11
struct rt_thread thread_2;
static rt_uint8_t thread_stack_2[1024]={0};
rt_uint32_t thread_stack_size_2=sizeof(thread_stack_2);

static void thread_func_1(void* parameter){
	int num=0;
	rt_sem_take(&sem_1,RT_WAITING_FOREVER);
	for(;;){
		//num=rt_device_write(uart_2,0,"hello\r\n",sizeof("hello\r\n"));		//д���ݵ��豸
		//rt_kprintf("thread_1 write byte: %d\r\n",num);
		rt_thread_delay(rt_tick_from_millisecond(500));				
	}
	rt_sem_release(&sem_1);
}

static void thread_func_2(void* parameter){
	rt_size_t num=0;
	
	struct rx_msg msg;
	rt_sem_take(&sem_1,RT_WAITING_FOREVER); 

	for(;;){
		//rt_kprintf("wait msgqueue\r\n");
		memset(read_buf,0,sizeof(read_buf));
		
		rt_mq_recv(&msgqueue_1,&msg,sizeof(struct rx_msg),RT_WAITING_FOREVER);		//����Ϣ������ȡ��Ϣ��û����Ϣ������
		
		num = (sizeof(read_buf) - 1) > msg.size ? msg.size : sizeof(read_buf) - 1;		//���û�������������� ����=msg.size
																																									//�����ͽض�
		num=rt_device_read(		//��ȡ�豸����
						msg.dev,
						0,
						&read_buf[0],
						num
		);
		read_buf[num] = '\0';
		//rt_kprintf("thread_2 read byte: %d\r\n",num);
		num=rt_device_write(		//д���ݵ��豸
					uart_1,
					0,
					&read_buf[0],
					num
		);		
		//rt_kprintf("thread_2 write byte: %d\r\n",num);
	
		rt_thread_delay(rt_tick_from_millisecond(300));			
	}
	rt_sem_release(&sem_1);
}


int main(void){
	rt_err_t err=RT_EOK;
	
	uart_2=rt_device_find("uart2");
	uart_1=rt_device_find("uart1");
	if(uart_2!=RT_NULL&&uart_1!=RT_NULL){
		rt_kprintf("++++++ uart1 and uart2 successful\r\n");
		//rt_device_init(uart_2);
		
		err|=rt_device_set_rx_indicate(uart_2,uart2_rx);				//�󶨻ص�����: uart2_rx
		
		/* ���豸 uart1 uart2 */
		err|=rt_device_open(uart_1,RT_DEVICE_OFLAG_RDWR);		
		err|=rt_device_open(uart_2,RT_DEVICE_OFLAG_RDWR|RT_DEVICE_FLAG_INT_RX);			//ע�⣺���Ҫ���� �����RT_DEVICE_FLAG_INT_RX����Ȼ�޷������ж�
		
		err|=rt_sem_init(&sem_1,"sem_1",2,RT_IPC_FLAG_FIFO);		//��ʼ���ź�������Դ�� = 2
		err|=rt_mq_init(&msgqueue_1,"msgqueue_1",&msgqueue_pool_1[0],128-sizeof(void* ),sizeof(msgqueue_pool_1),RT_IPC_FLAG_FIFO);		//��ʼ����Ϣ����
		
		if(err!=0)
			return -RT_ERROR;
	}

	
	rt_thread_init(
							&thread_1,
							"thread_1",
							thread_func_1,
							RT_NULL,
							&thread_stack_1,
							thread_stack_size_1,
							thread_prio_1,
							10
	);
	rt_thread_startup(&thread_1);
	
	rt_thread_init(
							&thread_2,
							"thread_2",
							thread_func_2,
							RT_NULL,
							&thread_stack_2,
							thread_stack_size_2,
							thread_prio_2,
							10
	);
	rt_thread_startup(&thread_2);
	for(;;){

		//rt_kprintf("RT-Thread runing\r\n");
		rt_thread_delay(rt_tick_from_millisecond(5000));				
	}
	return 0;
}
