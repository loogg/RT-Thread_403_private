#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <rtthread.h>

#define DATA_BITS_5                     5
#define DATA_BITS_6                     6
#define DATA_BITS_7                     7
#define DATA_BITS_8                     8
#define DATA_BITS_9                     9

#define STOP_BITS_1                     0
#define STOP_BITS_2                     1
#define STOP_BITS_3                     2
#define STOP_BITS_4                     3

#define PARITY_NONE                     0
#define PARITY_ODD                      1
#define PARITY_EVEN                     2

#define BIT_ORDER_LSB                   0
#define BIT_ORDER_MSB                   1

#define RT_SERIAL_CTRL_RESET            0x05
#define RT_SERIAL_CTRL_FLUSH            0x06
#define RT_SERIAL_CTRL_SYNC             0x07

/* Default config for serial_configure structure */
#define RT_SERIAL_CONFIG_DEFAULT           \
{                                          \
    115200,           /* 115200 bits/s */  \
    DATA_BITS_8,      /* 8 databits */     \
    STOP_BITS_1,      /* 1 stopbit */      \
    PARITY_NONE,      /* No parity  */     \
    BIT_ORDER_LSB,    /* LSB first sent */ \
    0                                      \
}

struct serial_configure
{
    rt_uint32_t baud_rate;

    rt_uint32_t data_bits               :4;
    rt_uint32_t stop_bits               :2;
    rt_uint32_t parity                  :2;
    rt_uint32_t bit_order               :1;
    rt_uint32_t reserved                :23;
};

struct rt_serial_device
{
    struct rt_device parent;

    rt_uint8_t sync_flag;
    struct serial_configure config;
    struct rt_ringbuffer rx_rb;
    const struct rt_uart_ops *ops;
    rt_err_t (*rx_indicate)(struct rt_serial_device *serial, rt_size_t size);
    rt_err_t (*tx_complete)(struct rt_serial_device *serial, void *buffer);
};
typedef struct rt_serial_device rt_serial_t;

/**
 * uart operators
 */
struct rt_uart_ops
{
    rt_err_t (*init)(struct rt_serial_device *serial, struct serial_configure *cfg);
    rt_err_t (*deinit)(struct rt_serial_device *serial);
    rt_err_t (*stop)(struct rt_serial_device *serial);
    rt_err_t (*control)(struct rt_serial_device *serial, int cmd, void *arg);

    rt_size_t (*start_tx)(struct rt_serial_device *serial, const void *buffer, rt_size_t size);
    rt_err_t (*stop_tx)(struct rt_serial_device *serial);

    rt_err_t (*restart_rx)(struct rt_serial_device *serial);
};

rt_err_t rt_hw_serial_register(struct rt_serial_device *serial,
                               const char              *name,
                               rt_uint32_t              flag,
                               void                    *data);

#endif
