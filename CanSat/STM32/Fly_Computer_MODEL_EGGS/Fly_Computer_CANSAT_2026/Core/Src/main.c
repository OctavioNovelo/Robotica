/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "sh2.h"
#include "sh2_hal.h"
#include "sh2_SensorValue.h"
#include "telemetry.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//VARIABLES VOLATILES DEBUG LECTURAS BME280
volatile int32_t  g_t_centi = 0;   // temperatura en centi-°C (ej 2534 = 25.34°C)
volatile uint32_t g_p_pa    = 0;   // presión en Pa
volatile uint32_t g_h_centi = 0;   // humedad en centi-%RH (ej 4550 = 45.50%RH)
volatile uint8_t  g_bme_ok  = 0;   // 1 si lectura OK

//DEBUG BNO085 RESPUESTA I2C
volatile uint8_t g_bno_addr = 0;      // 0x4A o 0x4B si se detecta, 0 si no
volatile uint8_t g_bno_ok   = 0;      // 1 si responde por I2C, 0 si no

//DEBUG LECTURA DE VALORES BNO
volatile uint8_t  g_bno_stream_ok = 0;
volatile float    g_ax=0, g_ay=0, g_az=0;
volatile float    g_gx=0, g_gy=0, g_gz=0;

//DEBUG INTERRUPCIONES BNO
static volatile uint8_t bno_int_flag = 0;
static volatile uint32_t bno_int_time_us = 0;
volatile uint32_t g_bno_last_check_ms = 0;

//DEBUG LIBRERIA SH2
volatile int g_sh2_open_rc = 999;
volatile uint32_t g_bno_reads = 0;
volatile uint32_t g_bno_read_err = 0;
volatile uint8_t g_bno_int_seen = 0;

//DEBUG CONTADORES DE LECTURA BNO
volatile uint32_t g_bno_writes = 0;
volatile uint32_t g_bno_write_err = 0;

volatile uint16_t g_bno_last_len = 0;
volatile uint8_t  g_bno_hdr0 = 0, g_bno_hdr1 = 0, g_bno_chan = 0, g_bno_seq = 0;


//DEFINICIONES DE PINES
#define BNO_I2C_ADDR_7B  0x4B
#define BNO_RST_GPIO_Port GPIOB
#define BNO_RST_Pin       GPIO_PIN_9

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define BNO085_ADDR_1 0x4A
#define BNO085_ADDR_2 0x4B


extern sh2_Hal_t g_sh2_hal;
extern UART_HandleTypeDef huart2;

static void sh2_event_cb(void *cookie, sh2_AsyncEvent_t *pEvent)
{
  (void)cookie; (void)pEvent;
  // Aquí podrías setear flags si quieres ver resets/eventos
}

static void sh2_sensor_cb(void *cookie, sh2_SensorEvent_t *pEvent)
{
  (void)cookie;
  sh2_SensorValue_t v;
  if (sh2_decodeSensorEvent(&v, pEvent) != 0) return;

  g_bno_stream_ok = 1;

  if (v.sensorId == SH2_ACCELEROMETER) {
    g_ax = v.un.accelerometer.x;
    g_ay = v.un.accelerometer.y;
    g_az = v.un.accelerometer.z;
  } else if (v.sensorId == SH2_GYROSCOPE_CALIBRATED) {
    g_gx = v.un.gyroscope.x; // rad/s
    g_gy = v.un.gyroscope.y;
    g_gz = v.un.gyroscope.z;
  }
}

static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t micros32(void)
{
  // SYSCLK = 84MHz -> 84 ciclos por microsegundo
  return (uint32_t)(DWT->CYCCNT / (SystemCoreClock / 1000000U));
}

// --- Funciones HAL que pide SH2 (sh2_hal.h) ---
static int sh2hal_open(sh2_Hal_t *self)
{
  (void)self;
  dwt_init();

  // Reset del BNO085 (activo en bajo)
  HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(50);

  bno_int_flag = 1; // deja que intente leer aunque no haya INT al inicio
  return 0;
}

static void sh2hal_close(sh2_Hal_t *self)
{
  (void)self;
  HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_RESET);
}

static int sh2hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
  (void)self;

  if (!bno_int_flag) return 0;     // solo leer si hay INT/flag

  // OJO: no borres el flag aún; bórralo hasta que una lectura sea válida.
  g_bno_reads++;

  unsigned maxRead = len;
  if (maxRead > 256) maxRead = 256;   // suficiente para accel/gyro

  if (maxRead < 4) return 0;

  // Lee “de un jalón”
  if (HAL_I2C_Master_Receive(&hi2c1, (BNO_I2C_ADDR_7B << 1), pBuffer, maxRead, 500) != HAL_OK) {
    g_bno_read_err++;
    // deja flag en 1 para reintentar
    return 0;
  }

  // Header SHTP
  g_bno_hdr0 = pBuffer[0];
  g_bno_hdr1 = pBuffer[1];
  g_bno_chan = pBuffer[2];
  g_bno_seq  = pBuffer[3];

  uint16_t total_len = (uint16_t)pBuffer[0] | ((uint16_t)(pBuffer[1] & 0x7F) << 8);
  g_bno_last_len = total_len;

  if (total_len < 4 || total_len > maxRead) {
    g_bno_read_err++;
    // deja flag en 1 para reintentar
    return 0;
  }

  // Ya consumimos un paquete válido → ahora sí baja flag
  bno_int_flag = 0;

  *t_us = bno_int_time_us ? bno_int_time_us : micros32();
  bno_int_time_us = 0;

  return (int)total_len;
}


static int sh2hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
  (void)self;
  if (HAL_I2C_Master_Transmit(&hi2c1, (BNO_I2C_ADDR_7B << 1), pBuffer, len, 200) != HAL_OK) {
    return 0;
  }
  return (int)len;
}

static uint32_t sh2hal_getTimeUs(sh2_Hal_t *self)
{
  (void)self;
  return micros32();
}

// Instancia HAL para SH2
sh2_Hal_t g_sh2_hal = {
  .open = sh2hal_open,
  .close = sh2hal_close,
  .read = sh2hal_read,
  .write = sh2hal_write,
  .getTimeUs = sh2hal_getTimeUs
};

// Llama esto desde tu EXTI callback (PB8)
void bno085_notify_int(void)
{
  bno_int_flag = 1;
  bno_int_time_us = micros32();
}

static void bno085_probe(void)
{
  // Devuelve HAL_OK si el dispositivo responde (ACK)
  if (HAL_I2C_IsDeviceReady(&hi2c1, (BNO085_ADDR_1 << 1), 3, 10) == HAL_OK) {
    g_bno_ok = 1;
    g_bno_addr = BNO085_ADDR_1;
    return;
  }
  if (HAL_I2C_IsDeviceReady(&hi2c1, (BNO085_ADDR_2 << 1), 3, 10) == HAL_OK) {
    g_bno_ok = 1;
    g_bno_addr = BNO085_ADDR_2;
    return;
  }

  g_bno_ok = 0;
  g_bno_addr = 0;
}


extern UART_HandleTypeDef huart2;

int _write(int file, char *ptr, int len) {
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}

// ---------- I2C helpers ----------


static HAL_StatusTypeDef i2c_wr(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len) {
  uint8_t buf[1 + 32];
  if (len > 32) return HAL_ERROR;
  buf[0] = reg;
  if (len) memcpy(&buf[1], data, len);
  return HAL_I2C_Master_Transmit(&hi2c1, (addr7 << 1), buf, 1 + len, 100);
}

static HAL_StatusTypeDef i2c_rd(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len) {
  if (HAL_I2C_Master_Transmit(&hi2c1, (addr7 << 1), &reg, 1, 100) != HAL_OK) return HAL_ERROR;
  return HAL_I2C_Master_Receive(&hi2c1, (addr7 << 1), data, len, 100);
}

// ---------- I2C scanner ----------
static void i2c_scan(void) {
  printf("\r\nI2C scan...\r\n");
  for (uint8_t a = 1; a < 127; a++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (a << 1), 2, 10) == HAL_OK) {
      printf("  Found: 0x%02X\r\n", a);
    }
  }
}

// ---------- Minimal BME280 driver (fixed-point) ----------
#define BME280_ADDR  0x76

// BME280 registers
#define REG_ID       0xD0
#define REG_RESET    0xE0
#define REG_CTRL_HUM 0xF2
#define REG_STATUS   0xF3
#define REG_CTRL_MEAS 0xF4
#define REG_CONFIG   0xF5
#define REG_PRESS_MSB 0xF7

typedef struct {
  // calibration data
  uint16_t dig_T1; int16_t dig_T2, dig_T3;
  uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
  uint8_t dig_H1; int16_t dig_H2; uint8_t dig_H3; int16_t dig_H4, dig_H5; int8_t dig_H6;
  int32_t t_fine;
} bme280_cal_t;

static bme280_cal_t bme;

static int bme280_read_calib(void) {
  uint8_t buf[26];

  // temp + pressure calib: 0x88..0xA1 (26 bytes)
  if (i2c_rd(BME280_ADDR, 0x88, buf, 26) != HAL_OK) return 0;
  bme.dig_T1 = (uint16_t)(buf[1]<<8 | buf[0]);
  bme.dig_T2 = (int16_t)(buf[3]<<8 | buf[2]);
  bme.dig_T3 = (int16_t)(buf[5]<<8 | buf[4]);

  bme.dig_P1 = (uint16_t)(buf[7]<<8 | buf[6]);
  bme.dig_P2 = (int16_t)(buf[9]<<8 | buf[8]);
  bme.dig_P3 = (int16_t)(buf[11]<<8 | buf[10]);
  bme.dig_P4 = (int16_t)(buf[13]<<8 | buf[12]);
  bme.dig_P5 = (int16_t)(buf[15]<<8 | buf[14]);
  bme.dig_P6 = (int16_t)(buf[17]<<8 | buf[16]);
  bme.dig_P7 = (int16_t)(buf[19]<<8 | buf[18]);
  bme.dig_P8 = (int16_t)(buf[21]<<8 | buf[20]);
  bme.dig_P9 = (int16_t)(buf[23]<<8 | buf[22]);
  bme.dig_H1 = buf[25];

  // humidity calib: 0xE1..0xE7 (7 bytes)
  uint8_t h[7];
  if (i2c_rd(BME280_ADDR, 0xE1, h, 7) != HAL_OK) return 0;
  bme.dig_H2 = (int16_t)(h[1]<<8 | h[0]);
  bme.dig_H3 = h[2];
  bme.dig_H4 = (int16_t)((h[3] << 4) | (h[4] & 0x0F));
  bme.dig_H5 = (int16_t)((h[5] << 4) | (h[4] >> 4));
  bme.dig_H6 = (int8_t)h[6];

  return 1;
}

static int bme280_init(void) {
  uint8_t id = 0;
  if (i2c_rd(BME280_ADDR, REG_ID, &id, 1) != HAL_OK) return 0;
  if (id != 0x60) { // BME280 chip id
    printf("BME280 ID mismatch: 0x%02X\r\n", id);
    return 0;
  }

  if (!bme280_read_calib()) return 0;

  // Config:
  // ctrl_hum: oversampling x1
  uint8_t v = 0x01;
  if (i2c_wr(BME280_ADDR, REG_CTRL_HUM, &v, 1) != HAL_OK) return 0;

  // ctrl_meas: temp os x1, press os x1, mode normal (3)
  v = (1<<5) | (1<<2) | 3;
  if (i2c_wr(BME280_ADDR, REG_CTRL_MEAS, &v, 1) != HAL_OK) return 0;

  // config: standby 0.5ms (0), filter off (0)
  v = 0x00;
  if (i2c_wr(BME280_ADDR, REG_CONFIG, &v, 1) != HAL_OK) return 0;

  return 1;
}

// compensation (fixed-point) -> returns:
// temp in 0.01°C, pressure in Pa, humidity in 0.01%RH
static int32_t bme280_comp_T(int32_t adc_T) {
  int32_t var1, var2;
  var1 = ((((adc_T>>3) - ((int32_t)bme.dig_T1<<1))) * ((int32_t)bme.dig_T2)) >> 11;
  var2 = (((((adc_T>>4) - ((int32_t)bme.dig_T1)) * ((adc_T>>4) - ((int32_t)bme.dig_T1))) >> 12) *
          ((int32_t)bme.dig_T3)) >> 14;
  bme.t_fine = var1 + var2;
  return (bme.t_fine * 5 + 128) >> 8;
}

static uint32_t bme280_comp_P(int32_t adc_P) {
  int64_t var1, var2, p;
  var1 = ((int64_t)bme.t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)bme.dig_P6;
  var2 = var2 + ((var1*(int64_t)bme.dig_P5)<<17);
  var2 = var2 + (((int64_t)bme.dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)bme.dig_P3)>>8) + ((var1 * (int64_t)bme.dig_P2)<<12);
  var1 = (((((int64_t)1)<<47) + var1)) * ((int64_t)bme.dig_P1) >> 33;

  if (var1 == 0) return 0; // avoid exception

  p = 1048576 - adc_P;
  p = (((p<<31) - var2) * 3125) / var1;
  var1 = (((int64_t)bme.dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)bme.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)bme.dig_P7)<<4);
  return (uint32_t)(p >> 8); // Pa
}

static uint32_t bme280_comp_H(int32_t adc_H) {
  int32_t v_x1;
  v_x1 = (bme.t_fine - ((int32_t)76800));
  v_x1 = (((((adc_H << 14) - (((int32_t)bme.dig_H4) << 20) - (((int32_t)bme.dig_H5) * v_x1)) + ((int32_t)16384)) >> 15) *
          (((((((v_x1 * ((int32_t)bme.dig_H6)) >> 10) * (((v_x1 * ((int32_t)bme.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
             ((int32_t)bme.dig_H2) + 8192) >> 14));
  v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((int32_t)bme.dig_H1)) >> 4));
  v_x1 = (v_x1 < 0 ? 0 : v_x1);
  v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
  return (uint32_t)(v_x1 >> 12); // 0.001%RH
}

static int bme280_read(int32_t *t_centi, uint32_t *p_pa, uint32_t *h_centi) {
  uint8_t d[8];
  if (i2c_rd(BME280_ADDR, REG_PRESS_MSB, d, 8) != HAL_OK) return 0;

  int32_t adc_P = (int32_t)((d[0]<<12) | (d[1]<<4) | (d[2]>>4));
  int32_t adc_T = (int32_t)((d[3]<<12) | (d[4]<<4) | (d[5]>>4));
  int32_t adc_H = (int32_t)((d[6]<<8) | d[7]);

  *t_centi = bme280_comp_T(adc_T);        // 0.01°C
  *p_pa    = bme280_comp_P(adc_P);        // Pa
  uint32_t h_milli = bme280_comp_H(adc_H); // 0.001%RH
  *h_centi = (h_milli + 5) / 10;          // 0.01%RH

  return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

void debug_print(char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // 1) Abrir SH2
  g_sh2_open_rc = sh2_open(&g_sh2_hal, sh2_event_cb, NULL);


  // 2) Registrar callback de sensores
  sh2_setSensorCallback(sh2_sensor_cb, NULL);

  // 3) Configurar reportes (ej. 50 Hz = 20000 us)
  sh2_SensorConfig_t cfg = {0};
  cfg.reportInterval_us = 20000;

  sh2_setSensorConfig(SH2_ACCELEROMETER, &cfg);
  sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &cfg);

  HAL_Delay(200);
  printf("\r\n--- CanSat Sensor Proto ---\r\n");

  i2c_scan(); // aquí verás 0x76 y el BNO085 (0x4A/0x4B) si está

  printf("Init BME280...\r\n");
  if (!bme280_init()) {
    printf("BME280 init FAILED\r\n");
  } else {
    printf("BME280 OK\r\n");
  }

  printf("Tip: BNO085 no tiene 'WHOAMI' simple; si aparece en scan (0x4A/0x4B), al menos responde ACK.\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  uint32_t t_bme = HAL_GetTick();
	  while (1)
	  {
		debug_print("STM32 ALIVE\n");

		TelemetryPacket pkt;

		telemetry_build(&pkt, altitude, pressure, temperature);

	    sh2_service();   // debe llamarse muy seguido

	    if (HAL_GetTick() - t_bme >= 500)
	    {
	      t_bme += 500;
	      g_bme_ok = bme280_read((int32_t*)&g_t_centi, (uint32_t*)&g_p_pa, (uint32_t*)&g_h_centi);

	      debug_print("SEQ:%d ALT:%d PRES:%d\n",
	                  pkt.seq,
	                  pkt.altitude,
	                  pkt.pressure);

	    }
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_8) {
    g_bno_int_seen = 1;
    bno085_notify_int();
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
