/*
* Data: 14/11/2022
* Universidade: UFMG - Universidade Federal de Minas Gerais
* Autores: Eduardo Cardoso Mendes e Tiago Rezende Valadares
* Versão: 1.0
* Tipo de licença:
* Nome da api: HMC5883L
* Arquivos: 
*        - HMC5883L.c
*        - HMC5883L.h
* Requisitos de hardware: 
*   - Módulo Bússola Eletrônica HMC5883L
      Datasheet: 
      https://www.generationrobots.com/media/module%20boussole%203%20axes%20HMC5883L/29133-HMC5883L-Compass-Module-IC-Documentation-v1.0.pdf
*   - Kit NUCLEO 64 – STM32F103RB
* Requisitos de software: 
*   - STM32CubeIDE 1.6.1 
*     Disponível em: https://www.st.com/en/development-tools/stm32cubeide.html
*
* Esta API foi desenvolvida como trabalho da disciplina de Programação de 
* Sistemas Embarcados da UFMG – Prof. Ricardo de Oliveira Duarte – 
* Departamento de Engenharia Eletrônica”.
*/
#ifndef _HMC5883L_H_
#define _HMC5883L_H_

#include "I2Cdev.h"


/*************************** I2C Device Addres ********************************/

#define HMC5883L_ADDRESS          0x1E

/****************************** Registeres ************************************/

#define REG_CONFIG_A              0x00
#define REG_CONFIG_B              0x01
#define REG_MODE                  0x02
#define REG_DATA_OUT_X_MSB        0x03
#define REG_DATA_OUT_X_LSB        0x04
#define REG_DATA_OUT_Z_MSB        0x05
#define REG_DATA_OUT_Z_LSB        0x06
#define REG_DATA_OUT_Y_MSB        0x07
#define REG_DATA_OUT_Y_LSB        0x08
#define REG_STATUS                0x09
#define REG_IDENT_A               0x0A
#define REG_IDENT_B               0x0B
#define REG_IDENT_C               0x0C


// Configuration Register A
// Layout for the bits of Configuration Register A (CRA)
#define HMC5883L_CRA_SA_POS       6 // SAMPLE AVERAGE position on CRA
#define HMC5883L_CRA_SA_TAM       2 // LENGTH of bits 
#define HMC5883L_CRA_DATARATE_POS 4 // DATA RATE position on CRA
#define HMC5883L_CRA_DATARATE_TAM 3
#define HMC5883L_CRA_MM_POS       1 // Measurement Mode position on CRA
#define HMC5883L_CRA_MM_TAM       2

// Samples averaged
#define HMC5883L_SAMPLE_AVERAGE_1 0x00 //(Default)
#define HMC5883L_SAMPLE_AVERAGE_2 0x01
#define HMC5883L_SAMPLE_AVERAGE_4 0x02
#define HMC5883L_SAMPLE_AVERAGE_8 0x03

// Typical Data Output Rate (Hz)
#define HMC5883L_DATARATE_0_75_HZ 0x00
#define HMC5883L_DATARATE_1_5HZ   0x01
#define HMC5883L_DATARATE_3HZ     0x02
#define HMC5883L_DATARATE_7_5HZ   0x03
#define HMC5883L_DATARATE_15HZ    0x04 //(Default)
#define HMC5883L_DATARATE_30HZ    0x05
#define HMC5883L_DATARATE_75HZ    0x06

// Measurement Mode
#define HMC5883L_MEAS_MODE_NORMAL 0x00 //(Default)
#define HMC5883L_MEAS_MODE_POS    0x01
#define HMC5883L_MEAS_MODE_NEG    0x02


// Configuration Register B
// Layout for the bits of Configuration Register B (CRB)
#define HMC5883L_CRB_RANGE_POS    7 // RANGE position on CRB
#define HMC5883L_CRB_RANGE_TAM    3 // LENGTH of bits 

// RANGE -> GAIN
#define HMC5883L_RANGE_0_88GA   	0x00
#define HMC5883L_RANGE_1_3GA	    0x01 //(Default)
#define HMC5883L_RANGE_1_9GA	    0x02
#define HMC5883L_RANGE_2_5GA      0x03
#define HMC5883L_RANGE_4GA        0x04
#define HMC5883L_RANGE_4_7GA      0x05
#define HMC5883L_RANGE_5_7GA      0x06
#define HMC5883L_RANGE_8_1GA      0x07

// Mode Register
// Layout for the bits of Mode Register
#define HMC5883L_MODEREG_POS      1
#define HMC5883L_MODEREG_TAM      2

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

/************************ Communication Functions ******************************/
/* As funções de comunicação com I2C utilizam a biblioteca "i2cdevlib", 
* disponível em:
* https://github.com/jrowberg/i2cdevlib
*/


/************************ Configuration Functions ******************************/
// Configuration Register A
uint8_t HMC5883L_getSampleAveraging();
void HMC5883L_setSampleAveraging(uint8_t averaging);
uint8_t HMC5883L_getDataRate();
void HMC5883L_setDataRate(uint8_t rate);
uint8_t HMC5883L_getMeasurementMode();
void HMC5883L_setMeasurementMode(uint8_t bias);

// CONFIG_B register
uint8_t HMC5883L_getGain();
void HMC5883L_setGain(uint8_t gain);

// MODE register
uint8_t HMC5883L_getMode();
void HMC5883L_setMode(uint8_t mode);

// General Functions
void HMC5883L_initialize(I2C_HandleTypeDef * hi2c);

bool HMC5883L_testConnection(I2C_HandleTypeDef * hi2c);

void HMC5883L_calibration();

void HMC5883L_measurement(int16_t *x, int16_t *y, int16_t *z);

#endif /* _HMC5883L_H_ */
