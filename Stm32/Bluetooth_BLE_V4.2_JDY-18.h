/**
 *      Data: 31/10/2022
 *      Universidade
 *      Autores: Pedro Otávio Fonseca Pires e Leandro Guatimosim
 *      Versão: 1.0
 *      Licença: MIT
 *      Nome da API: API para usode um Módulo Bluetooth BLE V4.2 JDY-18
 *      Arquivos relacionados: Bluetooth_BLE_V4.2_JDY-18.h
 *      					   Bluetooth_BLE_V4.2_JDY-18.c
 *      Resumo: Esta API foi desenvolvida como trabalho da disciplina de
 *Programação de Sistemas Embarcados da UFMG – Prof. Ricardo de Oliveira Duarte
 *– Departamento de Engenharia Eletrônica
 *------------------------------------------------------------------------------------------------------------------------------------------------------
 *      @file Bluetooth_BLE_V4.2_JDY-18.h
 *      @author Pedro Otávio Fonseca Pires and Leandro Guatimosim Gripp
 *      @brief This is the header file for the API of the
 *Bluetooth_BLE_V4.2_JDY-18 module. Datasheet:
 *https://manuals.plus/bluetooth-module/bluetooth-jdy-18-4-2-ble-module-usage-manual#axzz7jKHPWZQl
 */

#ifndef INC_BLUETOOTH_BLE_V4_2_JDY_18_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define INC_BLUETOOTH_BLE_V4_2_JDY_18_H_

#define ble_GPIOreset_Mode GPIO_MODE_OUTPUT_PP
#define ble_GPIOreset_Pull GPIO_NOPULL
#define ble_GPIOreset_Speed GPIO_SPEED_FREQ_LOW

#define ble_GPIOreset_PORT GPIOB
#define ble_GPIOreset_PIN GPIO_PIN_2
#define ble_GPIOreset_CLKEnable __HAL_RCC_GPIOB_CLK_ENABLE
#define ble_GPIOreset_turnedOnState false

#define ble_adress 0xa0
#define ble_reset_adress 0x10
//#define ble_sleep_adress 0x13
//#define ble_name_adress 0x30
#define ble_writeapp_adress 0xf0
#define ble_readapp_adress 0xf2
#define ble_disconnect_adress 0x15
#define ble_disconnect_bit 0x01
#define ble_reset_bit 0x01

#define ble_nameSize_max 15
#define ble_name_default "ble_pse_LP"

void ble_init();
HAL_StatusTypeDef ble_write(I2C_HandleTypeDef *hi2c,
                            uint8_t *buffer, uint32_t size);
HAL_StatusTypeDef ble_read(I2C_HandleTypeDef *hi2c,
                           uint8_t *buffer, uint32_t size);
uint16_t ble_send_info(I2C_HandleTypeDef *hi2c, const uint8_t *, uint8_t);
void ble_disconnect(I2C_HandleTypeDef *hi2c);
void ble_reset(I2C_HandleTypeDef *hi2c);

#endif /* INC_BLUETOOTH_BLE_V4_2_JDY_18_H_ */
