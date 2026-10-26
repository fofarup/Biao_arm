/*
  程序说明: JY901S 姿态、角速度和加速度解析，供末端调平使用。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-23
*/

#include "jy901s.h"
#include "usart.h"
#include <string.h>

#define JY901S_FRAME_SIZE (11U)
#define JY901S_MAX_AGE_MS  (100U)

static uint8_t rx_byte;
static uint8_t frame[JY901S_FRAME_SIZE];
static uint8_t frame_index;
volatile JY901S_Data_t latest;

/*
  函数说明: 把低字节在前的两个字节组合为有符号 16 位数。
  调用示例: JY901S_Int16(low, high);
  参数意义: low 为低字节。
            high 为高字节。
*/
static int16_t JY901S_Int16(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)low | ((uint16_t)high << 8U));
}

/*
  函数说明: 解析 JY901S 字节流，校验后分别更新加速度、角速度和姿态缓存。
  调用示例: JY901S_ParseByte(value);
  参数意义: value 为新收到的一个字节。
*/
static void JY901S_ParseByte(uint8_t value)
{
    uint8_t index;
    uint8_t checksum = 0U;
    uint32_t now;

    if (frame_index == 0U)
    {
        if (value != 0x55U)
        {
            return;
        }
        frame[frame_index++] = value;
        return;
    }
    if (frame_index == 1U && (value < 0x50U || value > 0x5AU))
    {
        frame_index = (value == 0x55U) ? 1U : 0U;
        return;
    }

    frame[frame_index++] = value;
    if (frame_index < JY901S_FRAME_SIZE)
    {
        return;
    }
    frame_index = 0U;
    for (index = 0U; index < JY901S_FRAME_SIZE - 1U; ++index)
    {
        checksum = (uint8_t)(checksum + frame[index]);
    }
    if (checksum != frame[JY901S_FRAME_SIZE - 1U])
    {
        ++latest.checksum_error_count;
        return;
    }

    now = HAL_GetTick();
    if (frame[1] == 0x51U)
    {
        latest.accel_x_g = (float)JY901S_Int16(frame[2], frame[3]) * (16.0f / 32768.0f);
        latest.accel_y_g = (float)JY901S_Int16(frame[4], frame[5]) * (16.0f / 32768.0f);
        latest.accel_z_g = (float)JY901S_Int16(frame[6], frame[7]) * (16.0f / 32768.0f);
        latest.accel_tick_ms = now;
    }
    else if (frame[1] == 0x52U)
    {
        latest.gyro_x_dps = (float)JY901S_Int16(frame[2], frame[3]) * (2000.0f / 32768.0f);
        latest.gyro_y_dps = (float)JY901S_Int16(frame[4], frame[5]) * (2000.0f / 32768.0f);
        latest.gyro_z_dps = (float)JY901S_Int16(frame[6], frame[7]) * (2000.0f / 32768.0f);
        latest.gyro_tick_ms = now;
    }
    else if (frame[1] == 0x53U)
    {
        latest.roll_deg = (float)JY901S_Int16(frame[2], frame[3]) * (180.0f / 32768.0f);
        latest.pitch_deg = (float)JY901S_Int16(frame[4], frame[5]) * (180.0f / 32768.0f);
        latest.yaw_deg = (float)JY901S_Int16(frame[6], frame[7]) * (180.0f / 32768.0f);
        latest.angle_tick_ms = now;
        ++latest.angle_count;
    }
}

/*
  函数说明: 启动 USART1 单字节接收中断；已启动时直接返回当前状态。
  调用示例: JY901S_StartReceive();
  参数意义: 无。
*/
HAL_StatusTypeDef JY901S_StartReceive(void)
{
    frame_index = 0U;
    (void)HAL_UART_AbortReceive(&huart1);
    return HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}

/*
  函数说明: 读取完整且新鲜的 IMU 数据；缺帧、过期或参数错误返回 HAL_ERROR。
  调用示例: JY901S_ReadData(data);
  参数意义: data 为数据缓冲区或传感器快照。
*/
HAL_StatusTypeDef JY901S_ReadData(JY901S_Data_t *data)
{
    uint32_t now;
    if (data == NULL)
    {
        return HAL_ERROR;
    }
    JY901S_GetLatest(data);
    now = HAL_GetTick();
    if (data->angle_count == 0U ||
        data->accel_tick_ms == 0U || data->gyro_tick_ms == 0U ||
        (now - data->angle_tick_ms) > JY901S_MAX_AGE_MS ||
        (now - data->accel_tick_ms) > JY901S_MAX_AGE_MS ||
        (now - data->gyro_tick_ms) > JY901S_MAX_AGE_MS)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/*
  函数说明: 处理 USART1 接收完成事件，解析一个字节并续接下一字节。
  调用示例: JY901S_UART_RxCpltCallback(huart);
  参数意义: huart 为HAL UART 句柄。
*/
void JY901S_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        JY901S_ParseByte(rx_byte);
        (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
    }
}

/*
  函数说明: 处理 USART1 接收错误，清理状态并尝试恢复接收。
  调用示例: JY901S_UART_ErrorCallback(huart);
  参数意义: huart 为HAL UART 句柄。
*/
void JY901S_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        frame_index = 0U;
        (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
    }
}

/*
  函数说明: 复制最近一次 IMU 缓存，供调试查看；该接口不保证数据新鲜。
  调用示例: JY901S_GetLatest(data);
  参数意义: data 为数据缓冲区或传感器快照。
*/
void JY901S_GetLatest(JY901S_Data_t *data)
{
    uint32_t primask;
    if (data == NULL)
    {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(data, (const void *)&latest, sizeof(*data));
    __set_PRIMASK(primask);
}
