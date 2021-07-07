/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-04-14     chenyong     first version
 */

#include <at.h>
#include <stdlib.h>
#include <stdio.h>

static char send_buf[AT_CMD_MAX_LEN];
static rt_size_t last_cmd_len = 0;

#ifdef AT_PRINT_RAW_CMD
static rt_uint8_t print_raw_enable = 0;
#endif

/**
 * dump hex format data to console device
 *
 * @param name name for hex object, it will show on log header
 * @param buf hex buffer
 * @param size buffer size
 */
void at_print_raw_cmd(const char *name, const char *buf, rt_size_t size)
{
#define __is_print(ch)       ((unsigned int)((ch) - ' ') < 127u - ' ')
#define WIDTH_SIZE           32

#ifdef AT_PRINT_RAW_CMD
    if(!print_raw_enable)
        return;
#endif

    rt_size_t i, j;

    for (i = 0; i < size; i += WIDTH_SIZE)
    {
        rt_kprintf("[D/AT] %s: %04X-%04X: ", name, i, i + WIDTH_SIZE);
        for (j = 0; j < WIDTH_SIZE; j++)
        {
            if (i + j < size)
            {
                rt_kprintf("%02X ", buf[i + j]);
            }
            else
            {
                rt_kprintf("   ");
            }
            if ((j + 1) % 8 == 0)
            {
                rt_kprintf(" ");
            }
        }
        rt_kprintf("  ");
        for (j = 0; j < WIDTH_SIZE; j++)
        {
            if (i + j < size)
            {
                rt_kprintf("%c", __is_print(buf[i + j]) ? buf[i + j] : '.');
            }
        }
        rt_kprintf("\n");
    }
}

#ifdef AT_PRINT_RAW_CMD

void at_print_raw_ctrl(rt_uint8_t enable)
{
    if(enable)
        print_raw_enable = 1;
    else
        print_raw_enable = 0;
}

static long at_enable_print_raw(void)
{
    print_raw_enable = 1;

    return RT_EOK;
}
MSH_CMD_EXPORT(at_enable_print_raw, enable at print raw);

static long at_disable_print_raw(void)
{
    print_raw_enable = 0;

    return RT_EOK;
}
MSH_CMD_EXPORT(at_disable_print_raw, disable at print raw);

#endif

RT_WEAK rt_size_t at_device_send(rt_device_t dev,
                                 rt_off_t    pos,
                                 const void *buffer,
                                 rt_size_t   size)
{
    return rt_device_write(dev, pos, buffer, size);
}

const char *at_get_last_cmd(rt_size_t *cmd_size)
{
    *cmd_size = last_cmd_len;
    return send_buf;
}

rt_size_t at_vprintf(rt_device_t device, const char *format, va_list args)
{
    last_cmd_len = vsnprintf(send_buf, sizeof(send_buf), format, args);
    if(last_cmd_len > sizeof(send_buf))
        last_cmd_len = sizeof(send_buf);

#ifdef AT_PRINT_RAW_CMD
    at_print_raw_cmd("sendline", send_buf, last_cmd_len);
#endif

    return at_device_send(device, 0, send_buf, last_cmd_len);
}

rt_size_t at_vprintfln(rt_device_t device, const char *format, va_list args)
{
    rt_size_t len;

    last_cmd_len = vsnprintf(send_buf, sizeof(send_buf) - 2, format, args);
    if(last_cmd_len > sizeof(send_buf) - 2)
        last_cmd_len = sizeof(send_buf) - 2;
    rt_memcpy(send_buf + last_cmd_len, "\r\n", 2);

    len = last_cmd_len + 2;

#ifdef AT_PRINT_RAW_CMD
    at_print_raw_cmd("sendline", send_buf, len);
#endif

    return at_device_send(device, 0, send_buf, len);
}
