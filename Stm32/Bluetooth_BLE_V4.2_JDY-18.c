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
 *      @file Bluetooth_BLE_V4.2_JDY-18.c
 *      @author Pedro Otávio Fonseca Pires and Leandro Guatimosim Gripp
 *      @brief This is the source file for the API of the
 *Bluetooth_BLE_V4.2_JDY-18 module. Datasheet:
 *https://manuals.plus/bluetooth-module/bluetooth-jdy-18-4-2-ble-module-usage-manual#axzz7jKHPWZQl
 */

#include "Bluetooth_BLE_V4.2_JDY-18.h"

void ble_resetHardware() {
#if ble_GPIOreset_turnedOnState
  HAL_GPIO_WritePin(ble_GPIOreset_PORT, ble_GPIOreset_PIN, GPIO_PIN_SET);
#else
  HAL_GPIO_WritePin(ble_GPIOreset_PORT, ble_GPIOreset_PIN, GPIO_PIN_RESET);
#endif
}

void ble_initGPIO() {

  GPIO_InitTypeDef initStruct = {0};
  initStruct.Mode = ble_GPIOreset_Mode;
  initStruct.Pull = ble_GPIOreset_Pull;
  initStruct.Speed = ble_GPIOreset_Speed;

  initStruct.Pin = ble_GPIOreset_PIN;
  initStruct.Alternate = 0x00;
  HAL_GPIO_Init(ble_GPIOreset_PORT, &initStruct);

  ble_resetHardware();
}

void ble_init() { ble_initGPIO(); }

HAL_StatusTypeDef ble_write(I2C_HandleTypeDef *hi2c, uint8_t *buffer, uint32_t size) {
  return HAL_I2C_Master_Transmit(hi2c, ble_adress, buffer, size, 1000);
}

HAL_StatusTypeDef ble_read(I2C_HandleTypeDef *hi2c,
                           uint8_t *buffer, uint32_t size) {
  return HAL_I2C_Master_Receive(hi2c, ble_adress, buffer, size, 1000);
}

uint16_t ble_send_info(I2C_HandleTypeDef *hi2c, const uint8_t *ble_dado,
                       uint8_t dsize) {
  if (ble_write(hi2c, ble_writeapp_adress, (uint8_t *)ble_dado, dsize) ==
      HAL_OK) {
    ble_reset(hi2c);
    return 1;
  } else
    return 0;
}

void ble_disconnect(I2C_HandleTypeDef *hi2c) {
  uint8_t valor = ble_disconnect_bit;
  ble_write(hi2c, ble_disconnect_adress, &valor, 1);
}

void ble_reset(I2C_HandleTypeDef *hi2c) {
  uint8_t valor = ble_reset_bit;
  ble_write(hi2c, ble_reset_adress, &valor, 1);
}
