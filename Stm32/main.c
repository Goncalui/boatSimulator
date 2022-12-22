#include <stdint.h>
#include "L293D_STM32F410.h"
#include "API_STM32F410_SERVOMOTOR_MG90D.h"
#include "QMC5883.h"
#include "I2Cdev.h"
#include "Sail_Algorithms.h"

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void GetBeaconBTData(Beacon *b);
void GetCompassData();
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  b1.x = UTM_B1_X;
  b1.y = UTM_B1_Y;
  b1.d = 0;
  b2.x = UTM_B2_X;
  b2.y = UTM_B2_Y;
  b2.d = 0;
  b3.x = UTM_B3_X;
  b3.y = UTM_B3_Y ;
  b3.d = 0;
  config.ADDR_Control_RegisterA = ADDR_REG_A;
  config.ADDR_Control_RegisterB = ADDR_REG_B;
  config.ADDR_Mode_Register = ADDR_REG_MODE;
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    GetBeaconBTData(&b1);
    GetBeaconBTData(&b2);
    GetBeaconBTData(&b3);
    GetCompassData();
	target_angle = GetTargetAngle(b1, b2, b3);
    rudder_angle = GetRudderAngle(boat_angle, target_angle);
    if (duty_cylce_dc_motor < MIN_DC_MOTOR_DUTY_CYCLE) {
      duty_cylce_dc_motor = MIN_DC_MOTOR_DUTY_CYCLE;
    } else if (duty_cylce_dc_motor > MAX_DC_MOTOR_DUTY_CYCLE) {
      duty_cylce_dc_motor = MAX_DC_MOTOR_DUTY_CYCLE;
    }
    if (rudder_angle < MIN_SERVO_ANGLE) {
      rudder_angle = MIN_SERVO_ANGLE;
    } else if (rudder_angle > MAX_SERVO_ANGLE) {
      rudder_angle = MAX_SERVO_ANGLE;
    }
    SetDCMotorDutyCycle(duty_cylce_dc_motor);
    SetServoAngle(rudder_angle);
    HAL_Delay(10);
  }
/* USER CODE END 3 */
}
void GetBeaconBTData(Beacon *b) {
  uint8_t bt_buffer[16];
  if (HAL_UART_Receive(&huart2, bt_buffer, sizeof(bt_buffer), 100) == HAL_OK) {
    char distance_str[8];
    if (sscanf((char*)bt_buffer, "D:%s", distance_str) == 1) {
      b->d = strtof(distance_str, NULL);
    }
  }
}
void GetCompassData() {
  int16_t raw_data[3];
  QMC5883_Read_Raw_Data(raw_data);
  boat_angle = QMC5883_Calculate_Heading(raw_data);
}
void SetDCMotorDutyCycle(uint8_t duty_cycle) {
  L293D_Set_Duty_Cycle(duty_cycle);
}
void SetServoAngle(float angle) {
  Servo_MG90D_Set_Angle(angle);
}
float GetTargetAngle(Beacon b1, Beacon b2, Beacon b3) {
  Trilateration(b1, b2, b3, &boat_x, &boat_y);
  float delta_x = b1.x - boat_x;
  float delta_y = b1.y - boat_y;
  float target_radians = atan2f(delta_y, delta_x);
  return target_radians * 180 / M_PI;
}

float GetRudderAngle(float boat_angle, float target_angle) {
  float delta_angle = target_angle - boat_angle;
  if (delta_angle > 180) {
    delta_angle -= 360;
  } else if (delta_angle < -180) {
    delta_angle += 360;
  }
  if (delta_angle > 90) {
    delta_angle = 90;
  } else if (delta_angle < -90) {
    delta_angle = -90;
  }
  float rudder_angle = delta_angle / 90 * MAX_SERVO_ANGLE;
  return rudder_angle;
}
