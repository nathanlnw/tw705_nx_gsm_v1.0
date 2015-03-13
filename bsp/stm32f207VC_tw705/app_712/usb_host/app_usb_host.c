/*
   app Usb host
*/

#include <rtthread.h>
#include <rthw.h>
#include "stm32f2xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

#include  <stdlib.h>//Êý×Ö×ª»»³É×Ö·û´®
#include  <stdio.h>
#include  <string.h>
#include "app_usb_host.h"


static rt_device_t _usbhost_device = RT_NULL;



/* app_usbhost thread */
struct rt_thread usbhost_thread;
char usbhost_thread_stack[1024];
struct rt_semaphore usbhost_sem;

#ifdef RT_USING_DEVICE
rt_device_t usbhost_device;
#endif




#ifdef RT_USING_DEVICE
/**
 * This function returns the device using in console.
 *
 * @return the device using in console or RT_NULL
 */
rt_device_t rt_usbhost_get_device(void)
{
    return _usbhost_device;
}

/**
 * This function will set a device as console device.
 * After set a device to console, all output of rt_kprintf will be
 * redirected to this new device.
 *
 * @param name the name of new console device
 *
 * @return the old console device handler
 */
rt_device_t rt_usbhost_set_device(const char *name)
{
    rt_device_t new, old;

    /* save old device */
    old = _usbhost_device;

    /* find new console device */
    new = rt_device_find(name);
    if (new != RT_NULL)
    {
        if (_usbhost_device != RT_NULL)
        {
            /* close old console device */
            rt_device_close(_usbhost_device);
        }

        /* set new console device */
        _usbhost_device = new;
        rt_device_open(_usbhost_device, RT_DEVICE_OFLAG_RDWR);
    }

    return old;
}
#endif


/* write one character to serial, must not trigger interrupt */
void rt_hw_usbhost_putc(const char c)
{
    /*
    	to be polite with serial console add a line feed
    	to the carriage return character
    */

}

void rt_hw_usbhost_output(const char *str)
{
    /* empty console output */
    //--------  add by  nathanlnw ---------
    while (*str)
    {
        rt_hw_usbhost_putc (*str++);
    }
    //--------  add by  nathanlnw  --------
}

#ifdef RT_USING_DEVICE
static rt_err_t usbhost_rx_ind(rt_device_t dev, rt_size_t size)
{
    /* release semaphore to let finsh thread rx data */
    rt_sem_release(&usbhost_sem);

    return RT_EOK;
}

void usbhost_set_device(char *device_name)
{
    rt_device_t dev = RT_NULL;

    dev = rt_device_find(device_name);
    if (dev != RT_NULL && rt_device_open(dev, RT_DEVICE_OFLAG_RDWR) == RT_EOK)
    {
        if (usbhost_device != RT_NULL)
        {
            /* close old finsh device */
            rt_device_close(usbhost_device);
        }
        usbhost_device = dev;
        rt_device_set_rx_indicate(dev, usbhost_rx_ind);
    }
    else
    {
        rt_kprintf("usbhost: can not find device:%s\n", device_name);
    }
}
#endif



void usbhost_thread_entry(void *parameter)
{
    char ch;
    char line[128];
    int  pos = 0 ;

    GPIO_InitTypeDef  GPIO_InitStructure;


    memset(line, 0, sizeof(line));
    pos = 0;

    while (1)
    {
        /* wait receive */
        //if (rt_sem_take(&shell->rx_sem, RT_WAITING_FOREVER) != RT_EOK) continue;
        if (rt_sem_take(&usbhost_sem, RT_WAITING_FOREVER) != RT_EOK)   continue;

        /* read one character from device */
        while (rt_device_read(usbhost_device, 0, &ch, 1) == 1)
        {

            line[pos] = ch;   //add new byte
            pos++;
            if(pos >= sizeof(line))
                pos = 0;
            /* handle end of line, break */
            if ( ch == '\n')
            {
                rt_kprintf("%s", line);
                rt_kprintf("%s", "\r\n usbhost mode\r\n");
                memset(line, 0, sizeof(line));
                pos = 0;
                break;
            }

        } /* end of device read */

    }
}


/* init usbhost */
void usbhost_app_init(void)
{
    rt_err_t result;

    rt_sem_init(&usbhost_sem, "usbhost", 0, 0);

    result = rt_thread_init(&usbhost_thread,
                            "usbhost",
                            usbhost_thread_entry, RT_NULL,
                            &usbhost_thread_stack[0], sizeof(usbhost_thread_stack),
                            6, 8);

    if (result == RT_EOK)
    {
        rt_thread_startup(&usbhost_thread);
    }
}






