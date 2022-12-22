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

#include "API_STM32F410_SERVOMOTOR_MG90D.h"

//Prototypes of internal functions, user wont use them
HAL_StatusTypeDef ConfigPWM(TIM_TypeDef* timer_instance,  uint32_t channel);
uint32_t Get_APB2_CLK(void);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel_NoStop(TIM_HandleTypeDef *htim,TIM_OC_InitTypeDef *sConfig,uint32_t Channel);
static void TIM_OC1_SetConfig_NoStop(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
static void TIM_OC2_SetConfig_NoStop(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
static void TIM_OC3_SetConfig_NoStop(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
static void TIM_OC4_SetConfig_NoStop(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);


TIM_HandleTypeDef htim1;
uint32_t channel;
TIM_OC_InitTypeDef sConfigOC = {0};
float position;
float zero_calibration=0;

/**
  * @brief  Perform the intire servo setup for the selected port/pin
  * @param  Port: GPIO port; Pin: GPIO pin
  * @retval HAL_StatusTypeDef: HAL_OK or HAL_ERROR
  */
HAL_StatusTypeDef SetupServo(GPIO_TypeDef *Port, uint16_t Pin) {

  TIM_TypeDef* timer_instance;

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if(Port==GPIOA){
	    timer_instance = TIM1;
	    __HAL_RCC_GPIOA_CLK_ENABLE();
	    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	    __HAL_RCC_TIM1_CLK_ENABLE();
	    switch (Pin) {
	      case GPIO_PIN_8:
	        channel = TIM_CHANNEL_1;
	        break;
	      case GPIO_PIN_9:
	        channel = TIM_CHANNEL_2;
	        break;
	      case GPIO_PIN_10:
	        channel = TIM_CHANNEL_3;
	        break;
	      case GPIO_PIN_11:
	        channel = TIM_CHANNEL_4;
	        break;
	      default:
	    	  printf("O pino selecionado não possui PWM");
	      return(HAL_ERROR);
	    }
  }
  else if(Port==GPIOB){
	    timer_instance = TIM11;
	    __HAL_RCC_GPIOB_CLK_ENABLE();
	    GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;
	    __HAL_RCC_TIM11_CLK_ENABLE();
	    if (Pin == GPIO_PIN_9) {
	      channel = TIM_CHANNEL_1;
	    }
	    else {
	      printf("O pino selecionado não possui PWM");
	      return(HAL_ERROR);
	    }
  }
  else if(Port==GPIOC){
	    timer_instance = TIM9;
	    __HAL_RCC_GPIOC_CLK_ENABLE();
	    GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
	    __HAL_RCC_TIM9_CLK_ENABLE();
	    switch (Pin) {
	      case GPIO_PIN_4:
	        channel = TIM_CHANNEL_1;
	        break;
	      case GPIO_PIN_5:
	        channel = TIM_CHANNEL_2;
	        break;
	      default:
	        printf("O pino selecionado não possui PWM");
	        return(HAL_ERROR);
	    }
  }
  else{
		printf("O port selecionado não possui PWM");
		return(HAL_ERROR);
  }

  GPIO_InitStruct.Pin = Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Port, &GPIO_InitStruct);
  return(ConfigPWM(timer_instance, channel));
}

/**
  * @brief  Configure timer and PWM for a given timer instance and channel
  * @param  timer_instance: name of the timer (TIM1,TIM9...); channel: timer channel for PWM output
  * @retval HAL_StatusTypeDef: HAL_OK or HAL_ERROR
  */
HAL_StatusTypeDef ConfigPWM(TIM_TypeDef* timer_instance,  uint32_t _channel){

  uint32_t TIM_CLK = Get_APB2_CLK();

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  // O valor do periodo deve ser menor que 65535
  if (TIM_CLK/SERVO_FREQ <= 65535)
  {
    htim1.Init.Prescaler = 0;
    htim1.Init.Period = TIM_CLK/SERVO_FREQ;
  }
  else // Senao, tem que setar o prescaler
  {
    // Prescaler calculado para permitir que o periodo seja menor que 65535
    // mesmo que o clock seja máximo.
    htim1.Init.Prescaler = TIM_CLK/(65535 * SERVO_FREQ) + 1;
    htim1.Init.Period = TIM_CLK / (SERVO_FREQ * htim1.Init.Prescaler);
  }

  htim1.Instance = timer_instance;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
	  return(HAL_ERROR);
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
	  return(HAL_ERROR);
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
	  return(HAL_ERROR);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, _channel) != HAL_OK) {
	  return(HAL_ERROR);
  }

  HAL_TIM_PWM_Start(&htim1, channel);

  if(timer_instance == TIM1){
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    	return(HAL_ERROR);
    }

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
    	return(HAL_ERROR);
    }
  }
  SetZero();
  return(HAL_OK);
}

/**
  * @brief  Function for calibration of the zero of the user scale. Servo will move along all its span
  *      staring at 0° and oscilating continuously between +90° and -90° until user press the board's blue button
  *      If user press the button twice in 3 seconds time interval, the servo will reverse the movement direction.
  *      If user press the button only once, the position at that very moment will be recorded as the zero position.
  * @param
  * @retval
  */
void ZeroCalibration(){
	  float angle = 0;

	  enum state_type {INIT=1,INC,SWITCH_OR_END_1,DEC,SWITCH_OR_END_2,END};
	  enum state_type state = INIT;
	  enum state_type next_state = INIT;
	  SetZero();

	  while(state!=END){

		  switch(state){
		  case INIT:
			  next_state = INIT;
			  if(!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)){
				  while(!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin));
				  HAL_Delay(200);
				  next_state = INC;
			  }
			  break;
		  case INC:
			  next_state = INC;
			  while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)) {
			      if(angle > 90) {
			    	  next_state = DEC;
			    	  break;
			      }
			      else {
			        SetPosition(angle);
			        angle+=0.01;
			      }
			      HAL_Delay(1);
			    }
			  while(!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin));
			  HAL_Delay(200);
			  if(next_state == INC) next_state = SWITCH_OR_END_1;

			  break;
		  case SWITCH_OR_END_1:
			  next_state = SWITCH_OR_END_1;
			  for(uint16_t aux_delay = 0; aux_delay<=3000; aux_delay++){
				  if(!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)){
					  while(!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin));
					  HAL_Delay(200);
					  next_state = DEC;
					  break;
				  }
				  HAL_Delay(1);
			  }

			  if(next_state == SWITCH_OR_END_1) next_state = END;

			  break;
		  case DEC:
			  next_state = DEC;
			  while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)) {
				  if(angle < -90) {
					  next_state = INC;
					  break;
				  }
				  else {
					SetPosition(angle);
					angle-=0.01;
				  }
				  HAL_Delay(1);
			  }
			  while(!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin));
			  HAL_Delay(200);
			  if(next_state == DEC) next_state = SWITCH_OR_END_2;

			  break;

		  case SWITCH_OR_END_2:
			  next_state = SWITCH_OR_END_2;
			  for(uint16_t aux_delay = 0; aux_delay<=3000; aux_delay++){
				  if(!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)){
					  while(!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin));
					  HAL_Delay(200);
					  next_state = INC;
					  break;
				  }
				  HAL_Delay(1);
			  }

			  if(next_state == SWITCH_OR_END_2) next_state = END;

			  break;

		  case END:
			  next_state = END;
			  break;
		  default:
			  next_state = INIT;
			  break;
		  }

		  state=next_state;

	  }

	  zero_calibration = zero_calibration-position;

}

/**
  * @brief  Set the axis's rotation according to the angle specified.
  * @param  int8_t angle: specifies the angle desired to set up the Servo Motor.
  * @retval None
  */
void SetPosition(float angle) {

  uint32_t pulse;

  if((angle > 90)||(angle < -90)) {
    printf("The angle's value is out of range.");
    //Error_Handler();
  }
  else {
    pulse = (1.5 + ((angle-zero_calibration) * RESOLUTION))*htim1.Init.Period/20;
    SetPWM(pulse);
  }
  position = angle;
}

/**
  * @brief  Set the axis's rotation according to the angle and speed specified.
  * @param  int8_t angle: specifies the angle desired to set up the Servo Motor;uint16_t speed: speed in °/s
  * @retval None
  */
void SetSmoothPosition(float angle, uint16_t speed){

	  if((angle > 90)||(angle < -90)) {
	    printf("The angle's value is out of range.");
	    //Error_Handler();
	  }
	  if((speed > 500)||(speed < 0)) {
	    printf("The speed's value is out of range.");
	    //Error_Handler();
	  }

	  float aux_position = position;
	  uint32_t delay_ms = 1000/speed;

	  if(position>angle){
		  while(position>angle){
			  aux_position-=1;
			  SetPosition(aux_position);
			  HAL_Delay(delay_ms);
		  }
	  }
	  else{
		  while(position<angle){
			  aux_position+=1;
			  SetPosition(aux_position);
			  HAL_Delay(delay_ms);
		  }
	  }
}

/**
  * @brief  Increment the actual position by the given angle
  * @param  int8_t angle: specifies the desired angle increment;
  * @retval None
  */
void AddAngle(float angle){
	SetPosition(position+angle);
}

/**
  * @brief  Increment the actual position by the given angle at the given speed
  * @param  int8_t angle: specifies the desired angle increment;uint16_t speed: speed in °/s
  * @retval None
  */
void AddSmoothAngle(float angle, uint16_t speed){
	SetSmoothPosition(position+angle,speed);
}

/**
  * @brief  Set the axis's rotation to 0º (neutral position)
  * @param  None
  * @retval None
  */
void SetZero() {
  SetPosition(0);
}

/**
  * @brief  Set the axis's rotation to +90º (maximum angle to the right)
  * @param  None
  * @retval None
  */
void Set90_Right() {
  SetPosition(90);
}

/**
  * @brief  Set the axis's rotation -90º (maximum angle to the left)
  * @param  None
  * @retval None
  */
void Set90_Left() {
  SetPosition(-90);
}

/**
  * @brief  Test function that rotates the servo motor's axis to right and left while the button is not pressed
  * @param  None
  * @retval None
  */
void Jogging() {

  float angle = 0;
  uint8_t status = INCREASE;

  SetZero();
  while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)) {
    if(status == INCREASE) {
      if(angle > 90) {
        status = DECREASE;
      }
      else {
        SetPosition(angle);
        angle+=0.01;
      }
    }
    else {
      if(angle < -90) {
        status = INCREASE;
      }
      else {
        SetPosition(angle);
        angle-=0.01;
      }
    }
    HAL_Delay(0.5);
  }
}

/**
  * @brief  Start the PWM with new value for duty cycle
  * @param  uint32_t pulse: Duty cycle desired
  * @retval None
  */
void SetPWM(uint32_t pulse) {

  sConfigOC.Pulse = pulse;

  HAL_TIM_PWM_ConfigChannel_NoStop(&htim1, &sConfigOC, channel);
  TIM_CHANNEL_STATE_SET(&htim1, channel, HAL_TIM_CHANNEL_STATE_READY);
  HAL_TIM_PWM_Start(&htim1, channel);
}

/**
  * @brief  Get the actual frequency value for Timer
  * @param  None
  * @retval None
  */
uint32_t Get_APB2_CLK(void) {
  /* Get PCLK2 frequency */
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();

  /* Get PCLK2 prescaler */
  if ((RCC->CFGR & RCC_CFGR_PPRE2) == 0) {
    /* PCLK2 prescaler equal to 1 => TIMCLK = PCLK2 */
    return (pclk2);
  }
  else {
    /* PCLK2 prescaler different from 1 => TIMCLK = 2 * PCLK2 */
    return (2*pclk2);
  }
}

//---------------------------------------------------------------------
//The following functions are modified HAL functions from the stm32f4xx_hal_tim library
// that allows updating the PWM pulse value without turning off the PWM generation, which was causing problems in servo control

HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel_NoStop(TIM_HandleTypeDef *htim,
                                            TIM_OC_InitTypeDef *sConfig,
                                            uint32_t Channel)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Check the parameters */
  assert_param(IS_TIM_CHANNELS(Channel));
  assert_param(IS_TIM_PWM_MODE(sConfig->OCMode));
  assert_param(IS_TIM_OC_POLARITY(sConfig->OCPolarity));
  assert_param(IS_TIM_FAST_STATE(sConfig->OCFastMode));

  /* Process Locked */
  __HAL_LOCK(htim);

  switch (Channel)
  {
    case TIM_CHANNEL_1:
    {
      /* Check the parameters */
      assert_param(IS_TIM_CC1_INSTANCE(htim->Instance));

      /* Configure the Channel 1 in PWM mode */
      TIM_OC1_SetConfig_NoStop(htim->Instance, sConfig);

      /* Set the Preload enable bit for channel1 */
      htim->Instance->CCMR1 |= TIM_CCMR1_OC1PE;

      /* Configure the Output Fast mode */
      htim->Instance->CCMR1 &= ~TIM_CCMR1_OC1FE;
      htim->Instance->CCMR1 |= sConfig->OCFastMode;
      break;
    }

    case TIM_CHANNEL_2:
    {
      /* Check the parameters */
      assert_param(IS_TIM_CC2_INSTANCE(htim->Instance));

      /* Configure the Channel 2 in PWM mode */
      TIM_OC2_SetConfig_NoStop(htim->Instance, sConfig);

      /* Set the Preload enable bit for channel2 */
      htim->Instance->CCMR1 |= TIM_CCMR1_OC2PE;

      /* Configure the Output Fast mode */
      htim->Instance->CCMR1 &= ~TIM_CCMR1_OC2FE;
      htim->Instance->CCMR1 |= sConfig->OCFastMode << 8U;
      break;
    }

    case TIM_CHANNEL_3:
    {
      /* Check the parameters */
      assert_param(IS_TIM_CC3_INSTANCE(htim->Instance));

      /* Configure the Channel 3 in PWM mode */
      TIM_OC3_SetConfig_NoStop(htim->Instance, sConfig);

      /* Set the Preload enable bit for channel3 */
      htim->Instance->CCMR2 |= TIM_CCMR2_OC3PE;

      /* Configure the Output Fast mode */
      htim->Instance->CCMR2 &= ~TIM_CCMR2_OC3FE;
      htim->Instance->CCMR2 |= sConfig->OCFastMode;
      break;
    }

    case TIM_CHANNEL_4:
    {
      /* Check the parameters */
      assert_param(IS_TIM_CC4_INSTANCE(htim->Instance));

      /* Configure the Channel 4 in PWM mode */
      TIM_OC4_SetConfig_NoStop(htim->Instance, sConfig);

      /* Set the Preload enable bit for channel4 */
      htim->Instance->CCMR2 |= TIM_CCMR2_OC4PE;

      /* Configure the Output Fast mode */
      htim->Instance->CCMR2 &= ~TIM_CCMR2_OC4FE;
      htim->Instance->CCMR2 |= sConfig->OCFastMode << 8U;
      break;
    }

    default:
      status = HAL_ERROR;
      break;
  }

  __HAL_UNLOCK(htim);

  return status;
}

/**
  * @brief  Timer Output Compare 1 configuration
  * @param  TIMx to select the TIM peripheral
  * @param  OC_Config The output configuration structure
  * @retval None
  */
static void TIM_OC1_SetConfig_NoStop(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config)
{
  uint32_t tmpccmrx;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Disable the Channel 1: Reset the CC1E Bit */
  //TIMx->CCER &= ~TIM_CCER_CC1E;

  /* Get the TIMx CCER register value */
  tmpccer = TIMx->CCER;
  /* Get the TIMx CR2 register value */
  tmpcr2 =  TIMx->CR2;

  /* Get the TIMx CCMR1 register value */
  tmpccmrx = TIMx->CCMR1;

  /* Reset the Output Compare Mode Bits */
  tmpccmrx &= ~TIM_CCMR1_OC1M;
  tmpccmrx &= ~TIM_CCMR1_CC1S;
  /* Select the Output Compare Mode */
  tmpccmrx |= OC_Config->OCMode;

  /* Reset the Output Polarity level */
  tmpccer &= ~TIM_CCER_CC1P;
  /* Set the Output Compare Polarity */
  tmpccer |= OC_Config->OCPolarity;

  if (IS_TIM_CCXN_INSTANCE(TIMx, TIM_CHANNEL_1))
  {
    /* Check parameters */
    assert_param(IS_TIM_OCN_POLARITY(OC_Config->OCNPolarity));

    /* Reset the Output N Polarity level */
    tmpccer &= ~TIM_CCER_CC1NP;
    /* Set the Output N Polarity */
    tmpccer |= OC_Config->OCNPolarity;
    /* Reset the Output N State */
    tmpccer &= ~TIM_CCER_CC1NE;
  }

  if (IS_TIM_BREAK_INSTANCE(TIMx))
  {
    /* Check parameters */
    assert_param(IS_TIM_OCNIDLE_STATE(OC_Config->OCNIdleState));
    assert_param(IS_TIM_OCIDLE_STATE(OC_Config->OCIdleState));

    /* Reset the Output Compare and Output Compare N IDLE State */
    tmpcr2 &= ~TIM_CR2_OIS1;
    tmpcr2 &= ~TIM_CR2_OIS1N;
    /* Set the Output Idle state */
    tmpcr2 |= OC_Config->OCIdleState;
    /* Set the Output N Idle state */
    tmpcr2 |= OC_Config->OCNIdleState;
  }

  /* Write to TIMx CR2 */
  TIMx->CR2 = tmpcr2;

  /* Write to TIMx CCMR1 */
  TIMx->CCMR1 = tmpccmrx;

  /* Set the Capture Compare Register value */
  TIMx->CCR1 = OC_Config->Pulse;

  /* Write to TIMx CCER */
  TIMx->CCER = tmpccer;
}

/**
  * @brief  Timer Output Compare 2 configuration
  * @param  TIMx to select the TIM peripheral
  * @param  OC_Config The output configuration structure
  * @retval None
  */
static void TIM_OC2_SetConfig_NoStop(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config)
{
  uint32_t tmpccmrx;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Disable the Channel 2: Reset the CC2E Bit */
  //TIMx->CCER &= ~TIM_CCER_CC2E;

  /* Get the TIMx CCER register value */
  tmpccer = TIMx->CCER;
  /* Get the TIMx CR2 register value */
  tmpcr2 =  TIMx->CR2;

  /* Get the TIMx CCMR1 register value */
  tmpccmrx = TIMx->CCMR1;

  /* Reset the Output Compare mode and Capture/Compare selection Bits */
  tmpccmrx &= ~TIM_CCMR1_OC2M;
  tmpccmrx &= ~TIM_CCMR1_CC2S;

  /* Select the Output Compare Mode */
  tmpccmrx |= (OC_Config->OCMode << 8U);

  /* Reset the Output Polarity level */
  tmpccer &= ~TIM_CCER_CC2P;
  /* Set the Output Compare Polarity */
  tmpccer |= (OC_Config->OCPolarity << 4U);

  if (IS_TIM_CCXN_INSTANCE(TIMx, TIM_CHANNEL_2))
  {
    assert_param(IS_TIM_OCN_POLARITY(OC_Config->OCNPolarity));

    /* Reset the Output N Polarity level */
    tmpccer &= ~TIM_CCER_CC2NP;
    /* Set the Output N Polarity */
    tmpccer |= (OC_Config->OCNPolarity << 4U);
    /* Reset the Output N State */
    tmpccer &= ~TIM_CCER_CC2NE;

  }

  if (IS_TIM_BREAK_INSTANCE(TIMx))
  {
    /* Check parameters */
    assert_param(IS_TIM_OCNIDLE_STATE(OC_Config->OCNIdleState));
    assert_param(IS_TIM_OCIDLE_STATE(OC_Config->OCIdleState));

    /* Reset the Output Compare and Output Compare N IDLE State */
    tmpcr2 &= ~TIM_CR2_OIS2;
    tmpcr2 &= ~TIM_CR2_OIS2N;
    /* Set the Output Idle state */
    tmpcr2 |= (OC_Config->OCIdleState << 2U);
    /* Set the Output N Idle state */
    tmpcr2 |= (OC_Config->OCNIdleState << 2U);
  }

  /* Write to TIMx CR2 */
  TIMx->CR2 = tmpcr2;

  /* Write to TIMx CCMR1 */
  TIMx->CCMR1 = tmpccmrx;

  /* Set the Capture Compare Register value */
  TIMx->CCR2 = OC_Config->Pulse;

  /* Write to TIMx CCER */
  TIMx->CCER = tmpccer;
}

/**
  * @brief  Timer Output Compare 3 configuration
  * @param  TIMx to select the TIM peripheral
  * @param  OC_Config The output configuration structure
  * @retval None
  */
static void TIM_OC3_SetConfig_NoStop(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config)
{
  uint32_t tmpccmrx;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Disable the Channel 3: Reset the CC2E Bit */
  //TIMx->CCER &= ~TIM_CCER_CC3E;

  /* Get the TIMx CCER register value */
  tmpccer = TIMx->CCER;
  /* Get the TIMx CR2 register value */
  tmpcr2 =  TIMx->CR2;

  /* Get the TIMx CCMR2 register value */
  tmpccmrx = TIMx->CCMR2;

  /* Reset the Output Compare mode and Capture/Compare selection Bits */
  tmpccmrx &= ~TIM_CCMR2_OC3M;
  tmpccmrx &= ~TIM_CCMR2_CC3S;
  /* Select the Output Compare Mode */
  tmpccmrx |= OC_Config->OCMode;

  /* Reset the Output Polarity level */
  tmpccer &= ~TIM_CCER_CC3P;
  /* Set the Output Compare Polarity */
  tmpccer |= (OC_Config->OCPolarity << 8U);

  if (IS_TIM_CCXN_INSTANCE(TIMx, TIM_CHANNEL_3))
  {
    assert_param(IS_TIM_OCN_POLARITY(OC_Config->OCNPolarity));

    /* Reset the Output N Polarity level */
    tmpccer &= ~TIM_CCER_CC3NP;
    /* Set the Output N Polarity */
    tmpccer |= (OC_Config->OCNPolarity << 8U);
    /* Reset the Output N State */
    tmpccer &= ~TIM_CCER_CC3NE;
  }

  if (IS_TIM_BREAK_INSTANCE(TIMx))
  {
    /* Check parameters */
    assert_param(IS_TIM_OCNIDLE_STATE(OC_Config->OCNIdleState));
    assert_param(IS_TIM_OCIDLE_STATE(OC_Config->OCIdleState));

    /* Reset the Output Compare and Output Compare N IDLE State */
    tmpcr2 &= ~TIM_CR2_OIS3;
    tmpcr2 &= ~TIM_CR2_OIS3N;
    /* Set the Output Idle state */
    tmpcr2 |= (OC_Config->OCIdleState << 4U);
    /* Set the Output N Idle state */
    tmpcr2 |= (OC_Config->OCNIdleState << 4U);
  }

  /* Write to TIMx CR2 */
  TIMx->CR2 = tmpcr2;

  /* Write to TIMx CCMR2 */
  TIMx->CCMR2 = tmpccmrx;

  /* Set the Capture Compare Register value */
  TIMx->CCR3 = OC_Config->Pulse;

  /* Write to TIMx CCER */
  TIMx->CCER = tmpccer;
}

/**
  * @brief  Timer Output Compare 4 configuration
  * @param  TIMx to select the TIM peripheral
  * @param  OC_Config The output configuration structure
  * @retval None
  */
static void TIM_OC4_SetConfig_NoStop(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config)
{
  uint32_t tmpccmrx;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Disable the Channel 4: Reset the CC4E Bit */
  //TIMx->CCER &= ~TIM_CCER_CC4E;

  /* Get the TIMx CCER register value */
  tmpccer = TIMx->CCER;
  /* Get the TIMx CR2 register value */
  tmpcr2 =  TIMx->CR2;

  /* Get the TIMx CCMR2 register value */
  tmpccmrx = TIMx->CCMR2;

  /* Reset the Output Compare mode and Capture/Compare selection Bits */
  tmpccmrx &= ~TIM_CCMR2_OC4M;
  tmpccmrx &= ~TIM_CCMR2_CC4S;

  /* Select the Output Compare Mode */
  tmpccmrx |= (OC_Config->OCMode << 8U);

  /* Reset the Output Polarity level */
  tmpccer &= ~TIM_CCER_CC4P;
  /* Set the Output Compare Polarity */
  tmpccer |= (OC_Config->OCPolarity << 12U);

  if (IS_TIM_BREAK_INSTANCE(TIMx))
  {
    /* Check parameters */
    assert_param(IS_TIM_OCIDLE_STATE(OC_Config->OCIdleState));

    /* Reset the Output Compare IDLE State */
    tmpcr2 &= ~TIM_CR2_OIS4;

    /* Set the Output Idle state */
    tmpcr2 |= (OC_Config->OCIdleState << 6U);
  }

  /* Write to TIMx CR2 */
  TIMx->CR2 = tmpcr2;

  /* Write to TIMx CCMR2 */
  TIMx->CCMR2 = tmpccmrx;

  /* Set the Capture Compare Register value */
  TIMx->CCR4 = OC_Config->Pulse;

  /* Write to TIMx CCER */
  TIMx->CCER = tmpccer;
}

