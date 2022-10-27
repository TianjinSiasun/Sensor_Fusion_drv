/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SISAUN_SF_QUEUE_H__
#define __SISAUN_SF_QUEUE_H__

#define QUEUE_SIZE  16

/**
 * queue - the fifo for record buf index of data
 * @data: buf index of data in ddr3
 * @front: current position of fifo
 * @rear: last position of fifo
 */
typedef struct{
    unsigned int data[QUEUE_SIZE];
    int front;
    int rear;
}queue;

queue * siasun_create_queue(void);
int siasun_queue_add(queue *q,unsigned long value);
unsigned long siasun_queue_del(queue *q);
int siasun_queue_empty(queue *q);
int siasun_queue_full(queue *q);
int siasun_queue_clear(queue *q);
void siasun_destory_queue(queue *q);

#endif /* __SISAUN_SF_QUEUE_H__ */
