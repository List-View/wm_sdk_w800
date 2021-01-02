#include <string.h>
#include "wm_include.h"
#include "wm_config.h"
#include "iperf.h"

#if TLS_CONFIG_WIFI_PERF_TEST
extern int tls_perf(void* data);

#define THT_QUEUE_SIZE	32
#define THT_TASK_PRIO	32
#define THT_TASK_STACK_SIZE 1024
struct tht_param gThtSys;
tls_os_queue_t *tht_q = NULL;
OS_STK ThtTaskStk[THT_TASK_STACK_SIZE]; 
int testing = 0;
u8  iperf_cpu_freq = 0;
void tht_task(void *sdata)
{
	void *tht = (struct tht_param *)sdata;
	void *msg;
	for(;;) 
	{
		printf("\n tht_task \n");
		//msg = OSQPend(tht_q, 0, &error);
	    tls_os_queue_receive(tht_q,(void **)&msg,0,0);

		printf("\n msg =%d\n",(u32)msg);
		switch((u32)msg)
		{
			case TLS_MSG_WIFI_PERF_TEST_START:
				printf("\nTHT_TEST_START\n");
				tls_perf(tht);
				break;
			default:
				break;
		}
	}

}


void CreateThroughputTask(void)
{
    if(!testing)
    {
        memset(&gThtSys, 0 ,sizeof(struct tht_param));
        iperf_cpu_freq = tls_reg_read32(HR_CLK_DIV_CTL)&0xFF;
        tls_os_queue_create(&tht_q, THT_QUEUE_SIZE);

        tls_os_task_create(NULL, NULL,
                           tht_task,
                           (void *)&gThtSys,
                           (u8 *)ThtTaskStk,
                           THT_TASK_STACK_SIZE * sizeof(u32),
                           THT_TASK_PRIO,
                           0);
        testing = 1;
        printf("CreateThroughputTask\n");
    }
}

#endif 

