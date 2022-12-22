/*
 *  API_STM32F410_SERVOMOTOR_MG90D.h
 *  This API was specifically developed to board NUCLEO-F410RB, in which has STM32F410RBT6 as mounted device.
 *
 *  Copyright (C) 2022 - Danilo G. Mariano <danilogm85@ufmg.br> / Raynner S. Carvalho <raynnercarvalho@ufmg.br>
 *  Version 1.0 - API with the following implemented functions:
 *  void 	   ConfigPeriod(void);
 *  void     SetPosition(int8_t angle);
 *  void     SetPWM(uint32_t pulse);
 *  void     SetZero(void);
 *  void     Set90_Right(void);
 *  void     Set90_Left(void);
 *  void     Jogging(void);
 *  uint32_t Get_APB2_CLK(void);
 *
 *  Created on: November 7th, 2022
 *  Institution: UFMG
 *  Authors:	Danilo Garcia Mariano
 *  			Raynner Schnneider Carvalho
 *
 *  EN: This API was developed as part of the Embedded Systems Programming course at UFMG - Prof. Ricardo de Oliveira Duarte - Electronics Engineering Department.
 *  PT: Esta API foi desenvolvida como trabalho da disciplina de Programação de Sistemas Embarcados da UFMG – Prof. Ricardo de Oliveira Duarte – Departamento de Engenharia Eletrônica.
 *
 *  This API contain functions to provide use of some hardware resources from Motor Shield LD293D H-Bridge available at:
 *  https://www.filipeflop.com/produto/motor-shield-l293d-driver-ponte-h-para-arduino/
 *  Control up to 4 DC motors, 2 Stepper Motors or 2 Servo Motors
 *  Output voltage: 4.5V ~ 16V
 *  Output current: 600mA per channel
 *
 *  Through the use of the Motor Shield LD293D H-Bridge is possible to control the Micro Servo Motor MG90S available at:
 *  https://www.filipeflop.com/produto/micro-servo-mg90s-towerpro/
 *  Operation voltage: 4.8V ~ 6.0V
 *  Torque: 1.8Kg/cm (4.8V) - 2.2Kg/cm (6.0V)
 *  Weight: ~13.4g
 *  Dimensions: 35.5 x 32.5 x 12mm
 *  PWM Frequency: 50Hz
 *  0.5 ms = 90º (left)
 *  1.5 ms =  0º (neutral position)
 *  2.5 ms = 90º (right)
 */

#ifndef API_STM32F410_SERVOMOTOR_MG90D_H_
#define API_STM32F410_SERVOMOTOR_MG90D_H_

	#include <stdint.h>
	#include <stdio.h>
	#include <stdlib.h>
	#include "stm32f4xx_hal.h"

	#define INCREASE 0
	#define DECREASE 1
	#define SERVO_FREQ 50
	#define RESOLUTION 1/100
	#define B1_Pin GPIO_PIN_13
	#define B1_GPIO_Port GPIOC

	HAL_StatusTypeDef SetupServo(GPIO_TypeDef *Port, uint16_t Pin);
	void     SetPosition(float angle);
	void     SetPWM(uint32_t pulse);
	void     SetZero(void);
	void     Set90_Right(void);
	void     Set90_Left(void);
	void     Jogging(void);
	void 	 SetSmoothPosition(float angle, uint16_t speed);
	void 	 AddSmoothAngle(float angle, uint16_t speed);
	void	 AddAngle(float angle);
	void 	 ZeroCalibration();

#endif /* API_STM32F410_SERVOMOTOR_MG90D_H_ */
