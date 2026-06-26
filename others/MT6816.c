#include "MT6816.h"

void stm32_spi_send_recv(uint8_t* tx_buf, uint8_t* rx_buf, uint16_t size)
{
    HAL_SPI_TransmitReceive(&hspi3, tx_buf, rx_buf, size, 0xFFFF);
}

void stm32_set_cs(uint8_t state)
{
    if (state == 0)
    {
        HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_SET);
    }
}

void mt6816_init(mt6816_HandleTypeDef* mt6816)
{
    mt6816->spi_send_recv = stm32_spi_send_recv;
    mt6816->set_cs = stm32_set_cs;
}

float mt6816_update_angle(mt6816_HandleTypeDef* mt6816)
{
    uint8_t tx_buf[3] = {0};
    uint8_t rx_buf[3] = {0};
    
    tx_buf[0] = 0x80 | 0x03; // 读取角度寄存器命令

    mt6816->set_cs(0); // 片选拉低
    mt6816->spi_send_recv(tx_buf, rx_buf, 3); // 发送命令并接收数据
    mt6816->set_cs(1); // 片选拉高

    uint16_t angle_raw = ((rx_buf[1] << 8) | (rx_buf[2] & 0xFC)) >> 2;
    float angle = (float)angle_raw * (2*3.14159265359f) / 16384.0f; // 转换为弧度

    mt6816->angle_raw = angle_raw;
    mt6816->angle = angle;

    return angle;
}