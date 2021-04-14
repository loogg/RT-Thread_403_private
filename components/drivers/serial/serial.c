#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG    "UART"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

static void _serial_check_buffer_size(void)
{
    static rt_bool_t already_output = RT_FALSE;

    if (already_output == RT_FALSE)
    {
#if !defined(RT_USING_ULOG) || defined(ULOG_USING_ISR_LOG)
        LOG_W("Warning: There is no enough buffer for saving data,"
              " please increase the rx_rb buf size.");
#endif
        already_output = RT_TRUE;
    }
}

static rt_err_t rt_serial_init(struct rt_device *dev)
{
    RT_ASSERT(dev != RT_NULL);

    rt_err_t result = RT_EOK;
    struct rt_serial_device *serial = (struct rt_serial_device *)dev;

    /* initialize rx/tx */
    serial->serial_rx = RT_NULL;
    serial->serial_tx = RT_NULL;

    /* apply configuration */
    if (serial->ops->configure)
        result = serial->ops->configure(serial, &serial->config);

    return result;
}

static rt_err_t rt_serial_close(struct rt_device *dev)
{
    RT_ASSERT(dev != RT_NULL);

    struct rt_serial_device *serial = (struct rt_serial_device *)dev;

    serial->ops->deinit(serial);

    
    return RT_EOK;
}

static rt_size_t rt_serial_read(struct rt_device *dev,
                                rt_off_t          pos,
                                void             *buffer,
                                rt_size_t         size)
{
    struct rt_serial_device *serial;

    RT_ASSERT(dev != RT_NULL);
    if (size == 0) return 0;

    serial = (struct rt_serial_device *)dev;

    if (dev->open_flag & RT_DEVICE_FLAG_INT_RX)
    {
        return _serial_int_rx(serial, (rt_uint8_t *)buffer, size);
    }
#ifdef RT_SERIAL_USING_DMA
    else if (dev->open_flag & RT_DEVICE_FLAG_DMA_RX)
    {
        return _serial_dma_rx(serial, (rt_uint8_t *)buffer, size);
    }
#endif /* RT_SERIAL_USING_DMA */    

    return _serial_poll_rx(serial, (rt_uint8_t *)buffer, size);
}

static rt_size_t rt_serial_write(struct rt_device *dev,
                                 rt_off_t          pos,
                                 const void       *buffer,
                                 rt_size_t         size)
{
    struct rt_serial_device *serial;

    RT_ASSERT(dev != RT_NULL);
    if (size == 0) return 0;

    serial = (struct rt_serial_device *)dev;

    if (dev->open_flag & RT_DEVICE_FLAG_INT_TX)
    {
        return _serial_int_tx(serial, (const rt_uint8_t *)buffer, size);
    }
#ifdef RT_SERIAL_USING_DMA    
    else if (dev->open_flag & RT_DEVICE_FLAG_DMA_TX)
    {
        return _serial_dma_tx(serial, (const rt_uint8_t *)buffer, size);
    }
#endif /* RT_SERIAL_USING_DMA */
    else
    {
        return _serial_poll_tx(serial, (const rt_uint8_t *)buffer, size);
    }
}

#ifdef RT_USING_POSIX_TERMIOS
struct speed_baudrate_item
{
    speed_t speed;
    int baudrate;
};

const static struct speed_baudrate_item _tbl[] =
{
    {B2400, BAUD_RATE_2400},
    {B4800, BAUD_RATE_4800},
    {B9600, BAUD_RATE_9600},
    {B19200, BAUD_RATE_19200},
    {B38400, BAUD_RATE_38400},
    {B57600, BAUD_RATE_57600},
    {B115200, BAUD_RATE_115200},
    {B230400, BAUD_RATE_230400},
    {B460800, BAUD_RATE_460800},
    {B921600, BAUD_RATE_921600},
    {B2000000, BAUD_RATE_2000000},
    {B3000000, BAUD_RATE_3000000},
};

static speed_t _get_speed(int baudrate)
{
    int index;

    for (index = 0; index < sizeof(_tbl)/sizeof(_tbl[0]); index ++)
    {
        if (_tbl[index].baudrate == baudrate)
            return _tbl[index].speed;
    }

    return B0;
}

static int _get_baudrate(speed_t speed)
{
    int index;

    for (index = 0; index < sizeof(_tbl)/sizeof(_tbl[0]); index ++)
    {
        if (_tbl[index].speed == speed)
            return _tbl[index].baudrate;
    }

    return 0;
}

static void _tc_flush(struct rt_serial_device *serial, int queue)
{
    rt_base_t level;
    int ch = -1;
    struct rt_serial_rx_fifo *rx_fifo = RT_NULL;
    struct rt_device *device = RT_NULL;

    RT_ASSERT(serial != RT_NULL);

    device = &(serial->parent);
    rx_fifo = (struct rt_serial_rx_fifo *) serial->serial_rx;

    switch(queue)
    {
        case TCIFLUSH:
        case TCIOFLUSH:

            RT_ASSERT(rx_fifo != RT_NULL);

            if((device->open_flag & RT_DEVICE_FLAG_INT_RX) || (device->open_flag & RT_DEVICE_FLAG_DMA_RX))
            {
                RT_ASSERT(RT_NULL != rx_fifo);
                level = rt_hw_interrupt_disable();
                rt_memset(rx_fifo->buffer, 0, serial->config.bufsz);
                rx_fifo->put_index = 0;
                rx_fifo->get_index = 0;
                rx_fifo->is_full = RT_FALSE;
                rt_hw_interrupt_enable(level);
            }
            else
            {
                while (1)
                {
                    ch = serial->ops->getc(serial);
                    if (ch == -1) break;
                }
            }

            break;

         case TCOFLUSH:
            break;
    }

}

#endif

static rt_err_t rt_serial_control(struct rt_device *dev,
                                  int              cmd,
                                  void             *args)
{
    rt_err_t ret = RT_EOK;
    struct rt_serial_device *serial;

    RT_ASSERT(dev != RT_NULL);
    serial = (struct rt_serial_device *)dev;

    switch (cmd)
    {
        case RT_DEVICE_CTRL_SUSPEND:
            /* suspend device */
            dev->flag |= RT_DEVICE_FLAG_SUSPENDED;
            break;

        case RT_DEVICE_CTRL_RESUME:
            /* resume device */
            dev->flag &= ~RT_DEVICE_FLAG_SUSPENDED;
            break;

        case RT_DEVICE_CTRL_CONFIG:
            if (args)
            {
                struct serial_configure *pconfig = (struct serial_configure *) args;
                if (pconfig->bufsz != serial->config.bufsz && serial->parent.ref_count)
                {
                    /*can not change buffer size*/
                    return RT_EBUSY;
                }
                /* set serial configure */
                serial->config = *pconfig;
                if (serial->parent.ref_count)
                {
                    /* serial device has been opened, to configure it */
                    serial->ops->configure(serial, (struct serial_configure *) args);
                }
            }

            break;
#ifdef RT_USING_POSIX
#ifdef RT_USING_POSIX_TERMIOS
        case TCGETA:
            {
                struct termios *tio = (struct termios*)args;
                if (tio == RT_NULL) return -RT_EINVAL;

                tio->c_iflag = 0;
                tio->c_oflag = 0;
                tio->c_lflag = 0;

                /* update oflag for console device */
                if (rt_console_get_device() == dev)
                    tio->c_oflag = OPOST | ONLCR;

                /* set cflag */
                tio->c_cflag = 0;
                if (serial->config.data_bits == DATA_BITS_5)
                    tio->c_cflag = CS5;
                else if (serial->config.data_bits == DATA_BITS_6)
                    tio->c_cflag = CS6;
                else if (serial->config.data_bits == DATA_BITS_7)
                    tio->c_cflag = CS7;
                else if (serial->config.data_bits == DATA_BITS_8)
                    tio->c_cflag = CS8;

                if (serial->config.stop_bits == STOP_BITS_2)
                    tio->c_cflag |= CSTOPB;

                if (serial->config.parity == PARITY_EVEN)
                    tio->c_cflag |= PARENB;
                else if (serial->config.parity == PARITY_ODD)
                    tio->c_cflag |= (PARODD | PARENB);

                cfsetospeed(tio, _get_speed(serial->config.baud_rate));
            }
            break;

        case TCSETAW:
        case TCSETAF:
        case TCSETA:
            {
                int baudrate;
                struct serial_configure config;

                struct termios *tio = (struct termios*)args;
                if (tio == RT_NULL) return -RT_EINVAL;

                config = serial->config;

                baudrate = _get_baudrate(cfgetospeed(tio));
                config.baud_rate = baudrate;

                switch (tio->c_cflag & CSIZE)
                {
                case CS5:
                    config.data_bits = DATA_BITS_5;
                    break;
                case CS6:
                    config.data_bits = DATA_BITS_6;
                    break;
                case CS7:
                    config.data_bits = DATA_BITS_7;
                    break;
                default:
                    config.data_bits = DATA_BITS_8;
                    break;
                }

                if (tio->c_cflag & CSTOPB) config.stop_bits = STOP_BITS_2;
                else config.stop_bits = STOP_BITS_1;

                if (tio->c_cflag & PARENB)
                {
                    if (tio->c_cflag & PARODD) config.parity = PARITY_ODD;
                    else config.parity = PARITY_EVEN;
                }
                else config.parity = PARITY_NONE;

                serial->ops->configure(serial, &config);
            }
            break;
        case TCFLSH:
            {
                int queue = (int)args;

                _tc_flush(serial, queue);
            }

            break;
        case TCXONC:
            break;
#endif /*RT_USING_POSIX_TERMIOS*/
        case FIONREAD:
            {
                rt_size_t recved = 0;
                rt_base_t level;

                level = rt_hw_interrupt_disable();
                recved = _serial_fifo_calc_recved_len(serial);
                rt_hw_interrupt_enable(level);

                *(rt_size_t *)args = recved;
            }
            break;
        case TIOCSWINSZ:
            {
                struct winsize* p_winsize;

                p_winsize = (struct winsize*)args;
                rt_kprintf("\x1b[8;%d;%dt", p_winsize->ws_col, p_winsize->ws_row);
            }
            break;
#endif /*RT_USING_POSIX*/
        default :
            /* control device */
            ret = serial->ops->control(serial, cmd, args);
            break;
    }

    return ret;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops serial_ops = 
{
    rt_serial_init,
    rt_serial_open,
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
    rt_err_t ret;
    struct rt_device *device;
    RT_ASSERT(serial != RT_NULL);

    device = &(serial->parent);

    device->type        = RT_Device_Class_Char;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    device->ops         = &serial_ops;
#else
    device->init        = rt_serial_init;
    device->open        = rt_serial_open;
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

    return ret;
}

/* ISR for serial interrupt */
void rt_hw_serial_isr(struct rt_serial_device *serial, int event)
{
    switch (event & 0xff)
    {
        case RT_SERIAL_EVENT_RX_IND:
        {
            int ch = -1;
            rt_base_t level;
            struct rt_serial_rx_fifo* rx_fifo;

            /* interrupt mode receive */
            rx_fifo = (struct rt_serial_rx_fifo*)serial->serial_rx;
            RT_ASSERT(rx_fifo != RT_NULL);

            while (1)
            {
                ch = serial->ops->getc(serial);
                if (ch == -1) break;


                /* disable interrupt */
                level = rt_hw_interrupt_disable();

                rx_fifo->buffer[rx_fifo->put_index] = ch;
                rx_fifo->put_index += 1;
                if (rx_fifo->put_index >= serial->config.bufsz) rx_fifo->put_index = 0;

                /* if the next position is read index, discard this 'read char' */
                if (rx_fifo->put_index == rx_fifo->get_index)
                {
                    rx_fifo->get_index += 1;
                    rx_fifo->is_full = RT_TRUE;
                    if (rx_fifo->get_index >= serial->config.bufsz) rx_fifo->get_index = 0;

                    _serial_check_buffer_size();
                }

                /* enable interrupt */
                rt_hw_interrupt_enable(level);
            }

            /* invoke callback */
            if (serial->parent.rx_indicate != RT_NULL)
            {
                rt_size_t rx_length;

                /* get rx length */
                level = rt_hw_interrupt_disable();
                rx_length = (rx_fifo->put_index >= rx_fifo->get_index)? (rx_fifo->put_index - rx_fifo->get_index):
                    (serial->config.bufsz - (rx_fifo->get_index - rx_fifo->put_index));
                rt_hw_interrupt_enable(level);

                if (rx_length)
                {
                    serial->parent.rx_indicate(&serial->parent, rx_length);
                }
            }
            break;
        }
        case RT_SERIAL_EVENT_TX_DONE:
        {
            struct rt_serial_tx_fifo* tx_fifo;

            tx_fifo = (struct rt_serial_tx_fifo*)serial->serial_tx;
            rt_completion_done(&(tx_fifo->completion));
            break;
        }
#ifdef RT_SERIAL_USING_DMA
        case RT_SERIAL_EVENT_TX_DMADONE:
        {
            const void *data_ptr;
            rt_size_t data_size;
            const void *last_data_ptr;
            struct rt_serial_tx_dma *tx_dma;

            tx_dma = (struct rt_serial_tx_dma*) serial->serial_tx;

            rt_data_queue_pop(&(tx_dma->data_queue), &last_data_ptr, &data_size, 0);
            if (rt_data_queue_peek(&(tx_dma->data_queue), &data_ptr, &data_size) == RT_EOK)
            {
                /* transmit next data node */
                tx_dma->activated = RT_TRUE;
                serial->ops->dma_transmit(serial, (rt_uint8_t *)data_ptr, data_size, RT_SERIAL_DMA_TX);
            }
            else
            {
                tx_dma->activated = RT_FALSE;
            }

            /* invoke callback */
            if (serial->parent.tx_complete != RT_NULL)
            {
                serial->parent.tx_complete(&serial->parent, (void*)last_data_ptr);
            }
            break;
        }
        case RT_SERIAL_EVENT_RX_DMADONE:
        {
            int length;
            rt_base_t level;

            /* get DMA rx length */
            length = (event & (~0xff)) >> 8;

            if (serial->config.bufsz == 0)
            {
                struct rt_serial_rx_dma* rx_dma;

                rx_dma = (struct rt_serial_rx_dma*) serial->serial_rx;
                RT_ASSERT(rx_dma != RT_NULL);

                RT_ASSERT(serial->parent.rx_indicate != RT_NULL);
                serial->parent.rx_indicate(&(serial->parent), length);
                rx_dma->activated = RT_FALSE;
            }
            else
            {
                /* disable interrupt */
                level = rt_hw_interrupt_disable();
                /* update fifo put index */
                rt_dma_recv_update_put_index(serial, length);
                /* calculate received total length */
                length = rt_dma_calc_recved_len(serial);
                /* enable interrupt */
                rt_hw_interrupt_enable(level);
                /* invoke callback */
                if (serial->parent.rx_indicate != RT_NULL)
                {
                    serial->parent.rx_indicate(&(serial->parent), length);
                }
            }
            break;
        }
#endif /* RT_SERIAL_USING_DMA */
    }
}
