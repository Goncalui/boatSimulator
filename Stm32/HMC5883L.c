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
*   - Kit NUCLEO 64 – STM32F410RB
* Requisitos de software: 
*   - STM32CubeIDE 1.6.1 
*     Disponível em: https://www.st.com/en/development-tools/stm32cubeide.html
*
* Esta API foi desenvolvida como trabalho da disciplina de Programação de 
* Sistemas Embarcados da UFMG – Prof. Ricardo de Oliveira Duarte – 
* Departamento de Engenharia Eletrônica”.
*/

#include "HMC5883L.h"

static uint8_t devAddr;
static uint8_t buffer[6];
static uint8_t mode;


// Configuration Functions

// Configuration Register A
/*
	Description: Lê o número de amostras médias por medição
	@param[]: void
	@returnValue: Número de médias (0-3 para 1/2/4/8 respectivamente)
*/
uint8_t HMC5883L_getSampleAveraging(){
  I2Cdev_readBits(devAddr, REG_CONFIG_A, HMC5883L_CRA_SA_POS, HMC5883L_CRA_SA_TAM, buffer, 0);
  return buffer[0];
}

/*
	Description: Define o número de amostras médias por medição
	@param[averaging]: Número de médias 
	Valores: (0-3 para 1/2/4/8 respectivamente)
	@returnValue: void
*/
void HMC5883L_setSampleAveraging(uint8_t averaging){
  I2Cdev_writeBits(devAddr,REG_CONFIG_A, HMC5883L_CRA_SA_POS, HMC5883L_CRA_SA_TAM, averaging);
}

/*
	Description: Lê a taxa atual de dados de saída do registrador
	@param[]: void
	@returnValue: Taxa atual de saída de dados
*/
uint8_t HMC5883L_getDataRate(){
  I2Cdev_readBits(devAddr, REG_CONFIG_A, HMC5883L_CRA_DATARATE_POS, HMC5883L_CRA_DATARATE_TAM, buffer, 0);
  return buffer[0];
}

/*
	Description: Define a taxa de dados de saída do registrador
	@param[rate]: Taxa de dados
  Valores:
  HMC5883L_DATARATE_0_75_HZ 0x00
  HMC5883L_DATARATE_1_5HZ   0x01
  HMC5883L_DATARATE_3HZ     0x02
  HMC5883L_DATARATE_7_5HZ   0x03
  HMC5883L_DATARATE_15HZ    0x04 //(Default)
  HMC5883L_DATARATE_30HZ    0x05
  HMC5883L_DATARATE_75HZ    0x06
	@returnValue: void
*/
void HMC5883L_setDataRate(uint8_t rate){
 I2Cdev_writeBits(devAddr, REG_CONFIG_A, HMC5883L_CRA_DATARATE_POS, HMC5883L_CRA_DATARATE_TAM, rate);
}

/*
	Description: Lê o valor atual do bias da medição 
	@param[]: void
	@returnValue: valor atual de bias (Tendência) (normal (0) / positivo (1) / negativo (2))
*/
uint8_t HMC5883L_getMeasurementBias(){
  I2Cdev_readBits(devAddr,REG_CONFIG_A, HMC5883L_CRA_MM_POS, HMC5883L_CRA_MM_TAM, buffer, 0);
  return buffer[0];
}

/*
	Description: Define o tipo de bias (tendência) da medição
	@param[bias]: Novo valor de bias (Tendência) (normal (0) / positivo (1) / negativo (2))
	@returnValue: void
*/
void HMC5883L_setMeasurementBias(uint8_t bias){
  I2Cdev_writeBits(devAddr, REG_CONFIG_A, HMC5883L_CRA_MM_POS, HMC5883L_CRA_MM_TAM, bias);
}


// CONFIG_B register
/*
	Description: Lê o range no qual o sensor está configurado, consequentemente o ganho
	@param[]: void
	@returnValue: Valor do Range/Ganho atual
  Valores:
  Value | Field Range | Gain (LSB/Gauss)
  ------+-------------+-----------------
  0     | +/- 0.88 Ga | 1370
  1     | +/- 1.3 Ga  | 1090 (Default)
  2     | +/- 1.9 Ga  | 820
  3     | +/- 2.5 Ga  | 660
  4     | +/- 4.0 Ga  | 440
  5     | +/- 4.7 Ga  | 390
  6     | +/- 5.6 Ga  | 330
  7     | +/- 8.1 Ga  | 230
*/
uint8_t HMC5883L_getRange(){
  I2Cdev_readBits(devAddr, REG_CONFIG_B, HMC5883L_CRB_RANGE_POS, HMC5883L_CRB_RANGE_TAM, buffer, 0);
  return buffer[0];
}

/*
	Description: Define o range de medição - O range está relacionado com o ganho do sensor
	@param[range]: Novo range de medição
  Valores:
  HMC5883L_RANGE_0_88GA 0x00
  HMC5883L_RANGE_1_3GA  0x01 //(Default)
  HMC5883L_RANGE_1_9GA  0x02
  HMC5883L_RANGE_2_5GA  0x03
  HMC5883L_RANGE_4GA    0x04
  HMC5883L_RANGE_4_7GA  0x05
  HMC5883L_RANGE_5_7GA  0x06
  HMC5883L_RANGE_8_1GA  0x07

	@returnValue: void
*/
void HMC5883L_setRange(uint8_t range){
  I2Cdev_writeByte(devAddr, REG_CONFIG_B, range << (HMC5883L_CRB_RANGE_POS - HMC5883L_CRB_RANGE_TAM + 1));
}

// MODE register
/*
	Description: Lê o modo de medição que o sensor esta configurado
	@param[]: void
	@returnValue: Modo atual de medição
	Valores:
  HMC5883L_MODE_CONTINUOUS: Modo de medição contínua
  HMC5883L_MODE_SINGLE: Modo de medição única
  HMC5883L_MODE_IDLE
*/
uint8_t HMC5883L_getMode(){
  I2Cdev_readBits(devAddr, REG_MODE, HMC5883L_MODEREG_POS, HMC5883L_MODEREG_TAM, buffer, 0);
  return buffer[0];
}

/*
	Description: Define o modo de medição
	@param[newMode]: Novo modo de medição
	Valores:
  HMC5883L_MODE_CONTINUOUS 0x00
  HMC5883L_MODE_SINGLE 0x01
  HMC5883L_MODE_IDLE 0x02
	@returnValue: void
*/
void HMC5883L_setMode(uint8_t newMode){
  // È necessário realizar a manipulação do parâmetro para que os bits específicos no registrador estejam setados corretamente.
  I2Cdev_writeByte(devAddr, REG_MODE, newMode << (HMC5883L_MODEREG_POS - HMC5883L_MODEREG_TAM + 1));
  mode = newMode;
}

// General Functions
/*
	Description: Inicializa o módulo com os valores defaults de número de amostras média por medição,
               frequênia de saída de dados e os bits de configuração de medição. 
               Também configura como medição única e o range de 1.3 Gauss
	@param[]:    void
	@returnValue: void
*/
void HMC5883L_initialize(I2C_HandleTypeDef * hi2c){
    I2Cdev_init(hi2c);
     // write CONFIG_A register
     // È necessário realizar os deslocmentos para manipular os bits específicos no registrador para cada função 
    I2Cdev_writeByte(devAddr, REG_CONFIG_A,
        (HMC5883L_SAMPLE_AVERAGE_8 << (HMC5883L_CRA_SA_POS - HMC5883L_CRA_SA_TAM + 1)) |
        (HMC5883L_DATARATE_15HZ     << (HMC5883L_CRA_DATARATE_POS - HMC5883L_CRA_DATARATE_TAM + 1)) |
        (HMC5883L_MEAS_MODE_NORMAL << (HMC5883L_CRA_MM_POS - HMC5883L_CRA_MM_TAM + 1)));

    // write CONFIG_B register
    HMC5883L_setRange(HMC5883L_RANGE_1_3GA);

    // write MODE register
    HMC5883L_setMode(HMC5883L_MODE_SINGLE);
}

/*
	Description: Verifica se conseguiu conectar ao magnetêmtro
	@param[]:    void
	@returnValue: void
*/
bool HMC5883L_testConnection(I2C_HandleTypeDef * hi2c1) {
    if (HAL_I2C_IsDeviceReady(hi2c1, HMC5883L_ADDRESS, 1, 20000) != HAL_OK) {
	    return false;
  	}
    return true;
}

/*
	Description: Faz a leitura dos três eixos do magnetômetro
	@param[x]: Ponteiro para um inteiro com sinal de 16-bits para as medições do eixo x
	@param[y]: Ponteiro para um inteiro com sinal de 16-bits para as medições do eixo y
	@param[z]: Ponteiro para um inteiro com sinal de 16-bits para as medições do eixo z
	@returnValue: void
*/
void HMC5883L_measurement(int16_t *x, int16_t *y, int16_t *z){
  I2Cdev_readBytes(devAddr, REG_DATA_OUT_X_MSB, 6, buffer, 0);
  if (mode == HMC5883L_MODE_SINGLE){
    I2Cdev_writeByte(devAddr, REG_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_POS - HMC5883L_MODE_IDLE + 1));
  }
  *x = (((int16_t)buffer[0]) << 8) | buffer[1];
  *y = (((int16_t)buffer[4]) << 8) | buffer[5];
  *z = (((int16_t)buffer[2]) << 8) | buffer[3];
}

/*
	Description: 
	@param[]:
	@param[]:
	@param[]:
	@returnValue:
*/
void HMC5883L_calibration(){

}
