#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG    "UART"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

static rt_size_t rt_serial_rb_put_update(struct rt_ringbuffer *rb, rt_uint16_t length)
{
    rt_uint16_t size;

    RT_ASSERT(rb != RT_NULL);

    /* whether has enough space */
    size = rt_ringbuffer_space_len(rb);

    /* no space */
    if (size == 0)
        return 0;

    /* drop some data */
    if (size < length)
        length = size;

    if (rb->buffer_size - rb->write_index > length)
    {
        /* this should not cause overflow because there is enough space for
         * length of data in current mirror */
        rb->write_index += length;
        return length;
    }

    /* we are going into the other side of the mirror */
    rb->write_mirror = ~rb->write_mirror;
    rb->write_index = length - (rb->buffer_size - rb->write_index);

    return length;
}

static rt_err_t rt_serial_init(struct rt_device *dev)
{
    RT_ASSERT(dev != RT_NULL);

    rt_err_t result = RT_EOK;
    struct rt_serial_device *serial = (struct rt_serial_device *)dev;

    rt_base_t level = rt_hw_interrupt_disable();
    serial->error_flag = 0;
    rt_ringbuffer_reset(&serial->rx_rb);
    result = serial->ops->init(serial, &serial->config);
    rt_hw_interrupt_enable(level);

    return result;
}

static rt_err_t rt_serial_close(struct rt_device *dev)
{
    RT_ASSERT(dev != RT_NULL);

    struct rt_serial_device *serial = (struct rt_serial_device *)dev;

    rt_base_t level = rt_hw_interrupt_disable();
    serial->error_flag = 0;
    serial->ops->deinit(serial);
    rt_hw_interrupt_enable(level);

    dev->flag &= ~RT_DEVICE_FLAG_ACTIVATED;
    
    return RT_EOK;
}

static rt_size_t rt_serial_read(struct rt_device *dev,
                                rt_off_t          pos,
                                void             *buffer,
                                rt_size_t         size)
{
    RT_ASSERT(dev != RT_NULL);

    struct rt_serial_device *serial = (struct rt_serial_device *)dev;

    if(!dev->ref_count)
        return 0;
    if(size == 0) 
        return 0;
    if(serial->error_flag)
        return 0;

    rt_size_t len = 0;

    rt_base_t level = rt_hw_interrupt_disable();
    len = rt_ringbuffer_get(&serial->rx_rb, buffer, size);
    rt_hw_interrupt_enable(level);

    return len;
}

static rt_size_t rt_serial_write(struct rt_device *dev,
                                 rt_off_t          pos,
                                 const void       *buffer,
                                 rt_size_t         size)
{
    RT_ASSERT(dev != RT_NULL);

    struct rt_serial_device *serial = (struct rt_serial_device *)dev;

    if(!dev->ref_count)
        return 0;
    if(size == 0)
        return 0;
    if(serial->error_flag)
        return 0;

    return serial->ops->start_tx(serial, buffer, size);
}

static rt_err_t rt_serial_control(struct rt_device *dev,
                                  int              cmd,
                                  void             *args)
{
    RT_ASSERT(dev != RT_NULL);

    struct rt_serial_device *serial = (struct rt_serial_device *)dev;

    rt_err_t ret = -RT_ERROR;

    switch (cmd)
    {
        case RT_DEVICE_CTRL_CONFIG:
        {
            if(args == RT_NULL)
                break;

            struct serial_configure *pconfig = (struct serial_configure *)args;
            serial->config = *pconfig;

            if(dev->ref_count)
                rt_serial_init(dev);

            ret = RT_EOK;
        }
        break;

        case RT_SERIAL_CTRL_RESET:
        {
            if(dev->ref_count)
                rt_serial_init(dev);

            ret = RT_EOK;
        }
        break;

        case RT_SERIAL_CTRL_FLUSH:
        {
            if(dev->ref_count)
            {
                rt_base_t level = rt_hw_interrupt_disable();
                serial->ops->stop(serial);
                rt_ringbuffer_reset(&serial->rx_rb);
                serial->ops->restart_rx(serial);
                rt_hw_interrupt_enable(level);
            }

            ret = RT_EOK;
        }
        break;

        case RT_SERIAL_CTRL_ASYNC:
        {
            if(!(dev->flag & RT_SERIAL_FLAG_ASYNC))
                break;
            
            int enable = (int)args;
            if(enable)
                serial->sync_flag = 0;
            else
                serial->sync_flag = 1;
            
            ret = RT_EOK;
        }
        break;

        default:
            ret = serial->ops->control(serial, cmd, args);
        break;
    }

    return ret;
}

static void rt_serial_check_buffer_size(struct rt_serial_device *serial)
{
#if !defined(RT_USING_ULOG) || defined(ULOG_USING_ISR_LOG)
    LOG_W("[%s]Warning: There is no enough buffer for saving data,"
          " please increase the rx_rb buf size.", serial->parent.parent.name);
#endif
}

static rt_err_t rt_serial_rx_indicate(struct rt_serial_device *serial, rt_size_t size)
{
    if(serial->error_flag)
        return -RT_ERROR;
    if(size == 0)
        return RT_EOK;
    
    rt_base_t level = rt_hw_interrupt_disable();
    if(rt_serial_rb_put_update(&serial->rx_rb, size) != size)
    {
        rt_hw_interrupt_enable(level);
        rt_serial_check_buffer_size(serial);
        serial->error_indicate(serial, RT_SERIAL_ERROR_RX_FULL);

        return -RT_ERROR;
    }
    size = rt_ringbuffer_data_len(&serial->rx_rb);
    rt_hw_interrupt_enable(level);

    if(serial->parent.rx_indicate)
        serial->parent.rx_indicate(&serial->parent, size);
    
    return RT_EOK;
}

static rt_err_t rt_serial_tx_complete(struct rt_serial_device *serial, void *buffer)
{
    if(serial->error_flag)
        return -RT_ERROR;
    
    serial->ops->stop_tx(serial);

    if(serial->parent.tx_complete)
        serial->parent.tx_complete(&serial->parent, buffer);

    return RT_EOK;
}

static rt_err_t rt_serial_error_indicate(struct rt_serial_device *serial, rt_uint16_t error_flag)
{
    serial->error_flag |= error_flag;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops serial_ops = 
{
    rt_serial_init,
    RT_NULL,
    rt_serial_close,
    rt_serial_read,
    rt_serial_write,
    rt_serial_control
};
#endif

/*
 * serial register
 */
rt_err_t rt_hw_serial_register(struct rt_serial_device *serial,
                               const char              *name,
                               rt_uint32_t              flag,
                               void                    *data)
{
    RT_ASSERT(serial != RT_NULL);

    rt_err_t ret;
    struct rt_device *device = &(serial->parent);

    device->type        = RT_Device_Class_Char;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    device->ops         = &serial_ops;
#else
    device->init        = rt_serial_init;
    device->open        = RT_NULL;
    device->close       = rt_serial_close;
    device->read        = rt_serial_read;
    device->write       = rt_serial_write;
    device->control     = rt_serial_control;
#endif
    device->user_data   = data;

    /* register a character device */
    ret = rt_device_register(device, name, flag);

#if defined(RT_USING_POSIX)
    /* set fops */
    device->fops        = &_serial_fops;
#endif

    if(flag & RT_SERIAL_FLAG_ASYNC)
        serial->sync_flag = 0;
    else
        serial->sync_flag = 1;

    serial->rx_indicate = rt_serial_rx_indicate;
    serial->tx_complete = rt_serial_tx_complete;
    serial->error_indicate = rt_serial_error_indicate;

    return ret;
}
