/* SPDX-License-Identifier: GPL-2.0 */
#include "sensor_fusion_queue.h"
#include "sensor_fusion_main.h"
#include <linux/slab.h>

queue * siasun_create_queue(void)
{
    queue *q;

    q=(queue *)kmalloc(sizeof(queue),GFP_KERNEL);
    if(q==NULL)
    {
        DPRINTK(KERN_ERR "queue malloc failed\n");
        return NULL;
    }

    memset(q->data,0,sizeof(q->data));
    q->front = 0;
    q->rear = 0;
    q->num = 0;

    return q;
}

int siasun_queue_add(queue *q,unsigned char value)
{
    if (value > 15)
    {
        DPRINTK(KERN_ERR "value is error!\n");
        return -EINVAL;
    }

    if((q->rear + 1) % QUEUE_SIZE == q->front)
    {
        DPRINTK(KERN_ERR "queue q is full!\n");
        return -ENOMEM;
    }

    q->data[q->rear] = value;
    q->rear = (q->rear + 1) % QUEUE_SIZE;
    q->num++;
    return 0;
}

unsigned char siasun_queue_del(queue *q)
{
    unsigned char ret;
    ret = q->data[q->front];
    q->front = (q->front+1) % QUEUE_SIZE;
    q->num--;

    return ret;
}

int siasun_queue_empty(queue *q)
{
    return (q->rear == q->front ? 1:0);
}

int siasun_queue_full(queue *q)
{
    return ((q->rear+1)%QUEUE_SIZE==q->front ? 1:0);
}

int siasun_queue_clear(queue *q)
{
    q->rear = 0;
    q->front = 0;
    q->num = 0;
    return 0;
}

void siasun_destory_queue(queue *q)
{
    kfree(q);
    q=NULL;
}

