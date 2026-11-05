/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : EV Battery Thermal Runaway Node - Physical Sensors Only
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_ADDR          (0x27 << 1) // Change to (0x3F << 1) if using 0x3F backpack
#define ONE_WIRE_PORT     GPIOB
#define ONE_WIRE_PIN      GPIO_PIN_6
#define RELAY_PORT        GPIOB
#define RELAY_PIN         GPIO_PIN_0

#define CRITICAL_TEMP     65.0f       // deg C
#define CRITICAL_DT_DT    0.40f       // deg C/s
#define OVERCURRENT_MA    1500.0f     // 1.5A cutoff
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float live_temp = 25.0f;
float last_temp = 25.0f;
float live_voltage = 3.70f;
float live_current_ma = 0.0f;
float filtered_current_ma = 0.0f;
float acs712_vzero = 2.50f;
float rri_score = 0.0f;
uint8_t relay_tripped = 0; // 0 = Closed/ON, 1 = Tripped/OFF
uint8_t sensor_detected = 0;

uint32_t last_sample_time = 0;
char tx_buffer[180];

// UART RX Circular buffer for Web Control
uint8_t rx_byte;
char rx_cmd_buffer[32];
uint8_t rx_index = 0;
volatile uint8_t web_cmd_flag = 0;
char pending_cmd[32];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void DWT_Init(void);
void delay_us(uint32_t us);
uint8_t DS18B20_Reset(void);
void DS18B20_WriteByte(uint8_t data);
uint8_t DS18B20_ReadByte(void);
float DS18B20_ReadTemperature(void);

float Read_Battery_Voltage(void);
float Read_Battery_Current_mA(void);
void Evaluate_Runaway_Risk(float dt);
void Set_Relay_State(uint8_t trip);

void LCD_Send_Cmd(char cmd);
void LCD_Send_Data(char data);
void LCD_Init(void);
void LCD_Send_String(char *str);
void LCD_Set_Cursor(int row, int col);
void Update_LCD(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void DWT_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us) {
  uint32_t start_cycles = DWT->CYCCNT;
  uint32_t target_cycles = us * (HAL_RCC_GetHCLKFreq() / 1000000);
  while ((DWT->CYCCNT - start_cycles) < target_cycles);
}

// ----------------- DS18B20 Driver -----------------
uint8_t DS18B20_Reset(void) {
  uint8_t presence = 0;
  HAL_GPIO_WritePin(ONE_WIRE_PORT, ONE_WIRE_PIN, GPIO_PIN_RESET);
  delay_us(480);
  HAL_GPIO_WritePin(ONE_WIRE_PORT, ONE_WIRE_PIN, GPIO_PIN_SET);
  delay_us(70);
  if (HAL_GPIO_ReadPin(ONE_WIRE_PORT, ONE_WIRE_PIN) == GPIO_PIN_RESET) {
    presence = 1;
  }
  delay_us(410);
  return presence;
}

void DS18B20_WriteByte(uint8_t data) {
  for (int i = 0; i < 8; i++) {
    if ((data & (1 << i)) != 0) {
      HAL_GPIO_WritePin(ONE_WIRE_PORT, ONE_WIRE_PIN, GPIO_PIN_RESET);
      delay_us(3);
      HAL_GPIO_WritePin(ONE_WIRE_PORT, ONE_WIRE_PIN, GPIO_PIN_SET);
      delay_us(60);
    } else {
      HAL_GPIO_WritePin(ONE_WIRE_PORT, ONE_WIRE_PIN, GPIO_PIN_RESET);
      delay_us(60);
      HAL_GPIO_WritePin(ONE_WIRE_PORT, ONE_WIRE_PIN, GPIO_PIN_SET);
      delay_us(5);
    }
  }
}

uint8_t DS18B20_ReadByte(void) {
  uint8_t value = 0;
  for (int i = 0; i < 8; i++) {
    HAL_GPIO_WritePin(ONE_WIRE_PORT, ONE_WIRE_PIN, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(ONE_WIRE_PORT, ONE_WIRE_PIN, GPIO_PIN_SET);
    delay_us(10);
    if (HAL_GPIO_ReadPin(ONE_WIRE_PORT, ONE_WIRE_PIN) != GPIO_PIN_RESET) {
      value |= (1 << i);
    }
    delay_us(50);
  }
  return value;
}

float DS18B20_ReadTemperature(void) {
  if (!DS18B20_Reset()) {
    sensor_detected = 0;
    return -999.0f;
  }
  sensor_detected = 1;

  DS18B20_WriteByte(0xCC);
  DS18B20_WriteByte(0x44);

  if (!DS18B20_Reset()) return -999.0f;
  DS18B20_WriteByte(0xCC);
  DS18B20_WriteByte(0xBE);

  uint8_t temp_lsb = DS18B20_ReadByte();
  uint8_t temp_msb = DS18B20_ReadByte();
  int16_t raw_temp = (temp_msb << 8) | temp_lsb;

  return (float)raw_temp / 16.0f;
}

// ----------------- ADC Sensors -----------------
float Read_Battery_Voltage(void) {
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return ((float)raw / 4095.0f) * 6.78f;
  }
  HAL_ADC_Stop(&hadc1);
  return 0.0f;
}

float Read_Battery_Current_mA(void) {
  uint32_t raw_sum = 0;
  // 128x oversampling to stabilize the 66mV/A slope
  for (int i = 0; i < 128; i++) {
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 2) == HAL_OK) {
      raw_sum += HAL_ADC_GetValue(&hadc2);
    }
    HAL_ADC_Stop(&hadc2);
  }
  float avg_raw = (float)raw_sum / 128.0f;
  float v_pin = (avg_raw / 4095.0f) * 3.3f;

  // ACS712-30A: 66 mV/A
  float instant_current_ma = ((v_pin - acs712_vzero) / 0.066f) * 1000.0f;

  // EMA Low-Pass Filter
  filtered_current_ma = (0.20f * instant_current_ma) + (0.80f * filtered_current_ma);

  // Deadband clamp: suppress residual idle drift within +/- 80mA
  if (fabsf(filtered_current_ma) < 80.0f) {
    return 0.0f;
  }
  return filtered_current_ma;
}

// ----------------- Relay Actuation -----------------
void Set_Relay_State(uint8_t trip) {
  relay_tripped = trip;
  if (trip) {
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_SET);   // Relay OFF (Isolated)
  } else {
    HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET); // Relay ON (Energized)
  }
}

// ----------------- Risk Index (Single Sensor Temporal + Electrical) -----------------
void Evaluate_Runaway_Risk(float dt) {
  // 1. Temperature Temporal Rise (dT/dt)
  float raw_dtdt = (live_temp - last_temp) / dt;
  if (raw_dtdt < 0.0f) raw_dtdt = 0.0f;
  float f_dtdt = raw_dtdt / CRITICAL_DT_DT;
  if (f_dtdt > 1.0f) f_dtdt = 1.0f;

  // 2. Absolute Temperature Threat Factor
  float f_temp = (live_temp - 30.0f) / (CRITICAL_TEMP - 30.0f);
  if (f_temp > 1.0f) f_temp = 1.0f;
  if (f_temp < 0.0f) f_temp = 0.0f;

  // 3. Current Overload Threat Factor
  float i_mag = fabsf(live_current_ma);
  float f_curr = i_mag / OVERCURRENT_MA;
  if (f_curr > 1.0f) f_curr = 1.0f;

  // 4. Voltage Deviation Factor
  float f_volt = 0.0f;
  if (live_voltage < 3.0f) {
    f_volt = (3.0f - live_voltage) / 1.0f;
  } else if (live_voltage > 4.25f) {
    f_volt = (live_voltage - 4.25f) / 0.5f;
  }
  if (f_volt > 1.0f) f_volt = 1.0f;

  // Weighted Composite Risk
  float instant_rri = (0.45f * f_dtdt) + (0.25f * f_temp) + (0.20f * f_curr) + (0.10f * f_volt);
  rri_score = (0.15f * instant_rri) + (0.85f * rri_score);

  // Safety Trigger
  if (i_mag >= OVERCURRENT_MA || live_temp >= CRITICAL_TEMP || rri_score >= 0.75f) {
    Set_Relay_State(1);
    rri_score = 1.00f;
  }
}

// ----------------- PCF8574 LCD Driver -----------------
void LCD_Send_Cmd(char cmd) {
  char data_u = (cmd & 0xf0);
  char data_l = ((cmd << 4) & 0xf0);
  uint8_t data_t[4] = {
    (uint8_t)(data_u | 0x0C), (uint8_t)(data_u | 0x08),
    (uint8_t)(data_l | 0x0C), (uint8_t)(data_l | 0x08)
  };
  HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_t, 4, 100);
}

void LCD_Send_Data(char data) {
  char data_u = (data & 0xf0);
  char data_l = ((data << 4) & 0xf0);
  uint8_t data_t[4] = {
    (uint8_t)(data_u | 0x0D), (uint8_t)(data_u | 0x09),
    (uint8_t)(data_l | 0x0D), (uint8_t)(data_l | 0x09)
  };
  HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_t, 4, 100);
}

void LCD_Init(void) {
  HAL_Delay(50);
  LCD_Send_Cmd(0x30); HAL_Delay(5);
  LCD_Send_Cmd(0x30); HAL_Delay(1);
  LCD_Send_Cmd(0x30); HAL_Delay(10);
  LCD_Send_Cmd(0x20); HAL_Delay(10);
  LCD_Send_Cmd(0x28); HAL_Delay(1);
  LCD_Send_Cmd(0x08); HAL_Delay(1);
  LCD_Send_Cmd(0x01); HAL_Delay(2);
  LCD_Send_Cmd(0x06); HAL_Delay(1);
  LCD_Send_Cmd(0x0C);
}

void LCD_Send_String(char *str) {
  while (*str) LCD_Send_Data(*str++);
}

void LCD_Set_Cursor(int row, int col) {
  LCD_Send_Cmd((row == 0) ? (0x80 + col) : (0xC0 + col));
}

void Update_LCD(void) {
  char line[17];

  // Line 1: Temperature & Voltage
  LCD_Set_Cursor(0, 0);
  int t_w = (sensor_detected) ? (int)live_temp : 0;
  int t_d = (sensor_detected) ? (int)(fabsf(live_temp - t_w) * 10.0f) : 0;
  int v_w = (int)live_voltage;
  int v_d = (int)(fabsf(live_voltage - v_w) * 100.0f);
  snprintf(line, sizeof(line), "T:%2d.%1dC V:%1d.%02dV", t_w, t_d, v_w, v_d);
  LCD_Send_String(line);

  // Line 2: Current and Relay / Risk status
  LCD_Set_Cursor(1, 0);
  if (relay_tripped) {
    snprintf(line, sizeof(line), "TRIP! RL:OFF    ");
  } else {
    int i_mag = (int)fabsf(live_current_ma);
    int r_w = (int)rri_score;
    int r_d = (int)(fabsf(rri_score - r_w) * 100.0f);
    snprintf(line, sizeof(line), "I:%4dmA R:%1d.%02d", i_mag, r_w, r_d);
  }
  LCD_Send_String(line);
}

// ----------------- UART RX ISR -----------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    if (rx_byte == '\n' || rx_byte == '\r') {
      if (rx_index > 0) {
        rx_cmd_buffer[rx_index] = '\0';
        strncpy(pending_cmd, rx_cmd_buffer, sizeof(pending_cmd));
        web_cmd_flag = 1;
        rx_index = 0;
      }
    } else if (rx_index < sizeof(rx_cmd_buffer) - 1) {
      rx_cmd_buffer[rx_index++] = rx_byte;
    }
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  DWT_Init();

  // Force PB6 Open-Drain & Pull-Up
  GPIOB->MODER   &= ~(3U << (6 * 2));
  GPIOB->MODER   |=  (1U << (6 * 2));
  GPIOB->OTYPER  |=  (1U << 6);
  GPIOB->PUPDR   &= ~(3U << (6 * 2));
  GPIOB->PUPDR   |=  (1U << (6 * 2));
  GPIOB->OSPEEDR |=  (3U << (6 * 2));
  GPIOB->ODR     |=  (1U << 6);

  // Enable UART1 Interrupt explicitly
  NVIC_EnableIRQ(USART1_IRQn);
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  LCD_Init();
  LCD_Set_Cursor(0, 0);
  LCD_Send_String("EV Safety Guard");
  LCD_Set_Cursor(1, 0);
  LCD_Send_String("Zero Calibrating");

  // Calibrate ACS712 zero offset across 256 samples
  uint32_t cal_sum = 0;
  for (int i = 0; i < 256; i++) {
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 5) == HAL_OK) {
      cal_sum += HAL_ADC_GetValue(&hadc2);
    }
    HAL_ADC_Stop(&hadc2);
    HAL_Delay(1);
  }
  acs712_vzero = ((float)cal_sum / 256.0f / 4095.0f) * 3.3f;

  Set_Relay_State(0); // Engage Relay on Boot
  HAL_Delay(500);
  LCD_Send_Cmd(0x01);

  float t_init = DS18B20_ReadTemperature();
  if (t_init > -50.0f && t_init < 125.0f) {
    live_temp = t_init;
    last_temp = t_init;
  }
  last_sample_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Process incoming Web commands safely outside ISR
    if (web_cmd_flag) {
      web_cmd_flag = 0;
      if (strstr(pending_cmd, "RELAY_ON") != NULL) {
        Set_Relay_State(0);
      } else if (strstr(pending_cmd, "RELAY_OFF") != NULL) {
        Set_Relay_State(1);
      }
    }

    uint32_t now = HAL_GetTick();
    if (now - last_sample_time >= 350) {
      float dt = (now - last_sample_time) / 1000.0f;
      last_sample_time = now;

      // 1. Read Physical Sensors
      float t_read = DS18B20_ReadTemperature();
      if (sensor_detected && t_read > -50.0f && t_read < 125.0f) {
        live_temp = t_read;
      }
      live_voltage = Read_Battery_Voltage();
      live_current_ma = Read_Battery_Current_mA();

      // 2. Hardware Button Cutoff (Nucleo B1 Blue Button PC13)
      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
        Set_Relay_State(1);
      }

      // 3. Risk Evaluation
      Evaluate_Runaway_Risk(dt);
      last_temp = live_temp;

      // 4. Update LCD
      Update_LCD();

      // 5. Transmit Clean JSON to ESP32 (USART1) & PC (USART2)
      int t_w = (sensor_detected) ? (int)live_temp : 0;
      int t_d = (sensor_detected) ? (int)(fabsf(live_temp - t_w) * 10.0f) : 0;
      int v_w = (int)live_voltage;
      int v_d = (int)(fabsf(live_voltage - v_w) * 100.0f);
      int i_val = (int)live_current_ma;
      int r_w = (int)rri_score;
      int r_d = (int)(fabsf(rri_score - r_w) * 100.0f);

      snprintf(tx_buffer, sizeof(tx_buffer),
               "{\"t\":%d.%d,\"v\":%d.%02d,\"i\":%d,\"rri\":%d.%02d,\"trip\":%d}\n",
               t_w, t_d, v_w, v_d, i_val, r_w, r_d, relay_tripped);

      HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer, strlen(tx_buffer), 100);
      HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, strlen(tx_buffer), 100);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
