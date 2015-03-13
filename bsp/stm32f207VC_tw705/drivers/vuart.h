#ifndef __RT_VUART_H__
#define __RT_VUART_H__

#include <rthw.h>
#include <rtthread.h>


#include <stm32f2xx.h>

#define UART_RX_BUFFER_SIZE		64
#define UART_TX_DMA_NODE_SIZE	4

extern struct rt_device			dev_vuart;

void uart1_baud(int buad);
rt_err_t rt_hw_serial_register(rt_device_t device, const char *name, rt_uint32_t flag, struct stm32_serial_device *serial);

void rt_hw_serial_isr(rt_device_t device);
void rt_hw_serial_dma_tx_isr(rt_device_t device);

#endif

