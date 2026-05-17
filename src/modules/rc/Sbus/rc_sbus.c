// Tip: 遥控器接收模块
#include "stm32h7xx_hal.h"
#include "rc_sbus.h"
#include "rm_config.h"
#include <string.h>
#include "cmsis_os.h"
#include "stdlib.h"

extern UART_HandleTypeDef huart5;
/* 数据有效性检查 */
#define VALID_CHANNEL(val) (abs(val) <= RC_MAX_VALUE)

sbus_data_t sbus_data[2];   // [0]:当前数据NOW,[1]:上一次的数据LAST


/**
 * @brief 初始化sbus_rc
 *
 * @return rc_obj_t* 指向NOW和LAST两次数据的数组起始地址
 */
sbus_data_t *sbus_rc_init(void)
{
    // USART_DMAEx_MultiBuffer_Init(&huart5,SBUS_MultiRx_Buf[0], SBUS_MultiRx_Buf[1],36);
    // 遥控器离线检测定时器相关
    return sbus_data;
}

/**
 * @brief 遥控器sbus数据解析
 *
 * @param 指向实例的指针
 */
int sbus_data_unpack(uint8_t* data, uint16_t len)
{
    // if ((data[0] != SBUS_HEAD) || (data[24] != SBUS_END))
    // {
    //     sbus_data[NOW].online = 0;
    //     return;
    // }
    // sbus_data[NOW].online = (data[23] & 0x0C) ? 0 : 1;
    if (data[0] == 0x0F)
    {
        /* 下面是正常遥控器数据的处理 */
        sbus_data[NOW].ch1 = ((data[1] | data[2] << 8) & 0x07FF) - 1024;
        sbus_data[NOW].ch2 = ((data[2] >> 3 | data[3] << 5) & 0x07FF) - 1024;
        sbus_data[NOW].ch3 = ((data[3] >> 6 | data[4] << 2 | data[5] << 10) & 0x07FF) - 1024;
        sbus_data[NOW].ch4 = ((data[5] >> 1 | data[6] << 7) & 0x07FF) - 1024;

        /* 旋钮值获取 */
        sbus_data[NOW].ch5 = ((data[6] >> 4 | data[7] << 4) & 0x07FF) - 1024;
        sbus_data[NOW].ch6 = ((data[7] >> 7 | data[8] << 1 | data[9] << 9) & 0x07FF) - 1024;
        /* 防止遥控器零点有偏差 */
        if(sbus_data[NOW].ch1 <= 10 && sbus_data[NOW].ch1 >= -10)
            sbus_data[NOW].ch1 = 0;
        if(sbus_data[NOW].ch2 <= 10 && sbus_data[NOW].ch2 >= -10)
            sbus_data[NOW].ch2 = 0;
        if(sbus_data[NOW].ch3 <= 10 && sbus_data[NOW].ch3 >= -10)
            sbus_data[NOW].ch3 = 0;
        if(sbus_data[NOW].ch4 <= 10 && sbus_data[NOW].ch4 >= -10)
            sbus_data[NOW].ch4 = 0;
        if(sbus_data[NOW].ch5 <= 10 && sbus_data[NOW].ch5 >= -10)
            sbus_data[NOW].ch5 = 0;
        if(sbus_data[NOW].ch6 <= 10 && sbus_data[NOW].ch6 >= -10)
            sbus_data[NOW].ch6 = 0;

        /* 拨杆值获取 */
        sbus_data[NOW].sw1 = ((data[9] >> 2 | data[10] << 6) & 0x07FF);
        sbus_data[NOW].sw2 = ((data[10] >> 5 | data[11] << 3) & 0x07FF);
        sbus_data[NOW].sw3 = ((data[12] | data[13] << 8) & 0x07FF);
        sbus_data[NOW].sw4 =((data[13] >> 3 | data[14] << 5) & 0x07FF);

        /* 数据有效性检查 */
        /* 遥控器异常值处理，函数直接返回 */
        if ((abs(sbus_data[NOW].ch1) > RC_MAX_VALUE) || \
          (abs(sbus_data[NOW].ch2) > RC_MAX_VALUE) || \
          (abs(sbus_data[NOW].ch3) > RC_MAX_VALUE) || \
          (abs(sbus_data[NOW].ch4) > RC_MAX_VALUE))
        {
            memset(&sbus_data[NOW], 0, sizeof(sbus_data_t));
            return -1;
        }

        sbus_data[LAST] = sbus_data[NOW];
    }
}
