/*
 * SPDX-FileCopyrightText: 2026 FryingCat
 * SPDX-License-Identifier: MIT
 */

#ifndef MT6816_H
#define MT6816_H

#include "spi.h"
#include "gpio.h"
#include "main.h"

typedef struct
{
    uint16_t angle_raw;
    float angle;
    void (*spi_send_recv)(uint8_t* tx_buf, uint8_t* rx_buf, uint16_t size);
    void (*set_cs)(uint8_t state);
}mt6816_HandleTypeDef;

void mt6816_init(mt6816_HandleTypeDef* mt6816);
float mt6816_update_angle(mt6816_HandleTypeDef* mt6816);

#endif // !MT6816_H