/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : OLED Display — receives from E32 UART1,
  *                   forwards to FTDI UART2, displays Freq+RPM on OLED
  *
  * Hardware:
  *   MCU   : STM32F103C8T6 (Blue Pill)
  *   Clock : HSE 8MHz + PLL → 72MHz
  *   I2C1  : PB6 (SCL), PB7 (SDA)
  *   OLED  : SSD1306/SSD1309 128x64
  *   UART1 : PA9(TX) PA10(RX) — E32 RF module
  *   UART2 : PA2(TX) PA3(RX)  — FTDI to PC terminal
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OLED_ADDR_PRIMARY   0x78
#define OLED_ADDR_FALLBACK  0x7A
#define RX_BUF_SIZE         80
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static char    rx_buf[RX_BUF_SIZE];
static uint8_t rx_idx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t OLED_Verify(void);
static void    Parse_And_Display(char *line);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ────────────────────────────────────────────────────────────────────────────
 * OLED_Verify()
 * ──────────────────────────────────────────────────────────────────────────*/
static uint8_t OLED_Verify(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_ADDR_PRIMARY,  3, 100) == HAL_OK) return 1;
    if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_ADDR_FALLBACK, 3, 100) == HAL_OK) return 1;
    return 0;
}

/* ────────────────────────────────────────────────────────────────────────────
 * Parse_And_Display()
 *
 * Expects incoming line format from transmitter:
 *   "Freq: 1000.0 Hz | RPM: 6000\r\n"
 *
 * Displays on OLED:
 *   Line 1 (y=10): Freq=1000.0Hz
 *   Line 2 (y=35): RPM =6000
 * ──────────────────────────────────────────────────────────────────────────*/
static void Parse_And_Display(char *line)
{
    char oled_line1[22] = "FREQ= ----";
    char oled_line2[22] = "RPM = ----";

    int freq = 0;
    int rval = 0;

    // Parse FREQ value
    char *ptr_freq = strstr(line, "FREQ:");
    if (ptr_freq != NULL)
        sscanf(ptr_freq, "FREQ:%d", &freq);

    // Parse RM, RP, or R — all show as RPM on OLED
    if (strstr(line, "RM:") != NULL)
        sscanf(strstr(line, "RM:"), "RM:%d", &rval);
    else if (strstr(line, "RP:") != NULL)
        sscanf(strstr(line, "RP:"), "RP:%d", &rval);
    else if (strstr(line, "R:") != NULL)
        sscanf(strstr(line, "R:"), "R:%d", &rval);

    // Build OLED strings
    snprintf(oled_line1, sizeof(oled_line1), "FREQ=%d", freq);
    snprintf(oled_line2, sizeof(oled_line2), "RPM =%d", rval);

    // Update OLED
    SSD1306_Fill(0);
    SSD1306_GotoXY(0, 10);
    SSD1306_Puts(oled_line1, &Font_7x10, 1);
    SSD1306_GotoXY(0, 35);
    SSD1306_Puts(oled_line2, &Font_7x10, 1);
    SSD1306_UpdateScreen();
}

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // ── OLED check ──────────────────────────────────────────────────────────
  if (OLED_Verify() == 0)
  {
      while (1)
      {
          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
          HAL_Delay(100);   // fast blink = OLED not found
      }
  }

  // 3 slow blinks = OLED found
  for (int i = 0; i < 3; i++)
  {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
      HAL_Delay(300);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      HAL_Delay(300);
  }

  // Init OLED — show waiting screen
  SSD1306_Init();
  SSD1306_Fill(0);
  SSD1306_GotoXY(0, 10);
  SSD1306_Puts("Freq= ----", &Font_7x10, 1);
  SSD1306_GotoXY(0, 35);
  SSD1306_Puts("RPM = ----", &Font_7x10, 1);
  SSD1306_UpdateScreen();

  // Ready message to PC via FTDI
  char header[] = "=== OLED Receiver Ready ===\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)header, strlen(header), 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint8_t byte;

    if (HAL_UART_Receive(&huart1, &byte, 1, 100) == HAL_OK)
    {
        // Mirror to FTDI → PC terminal
        HAL_UART_Transmit(&huart2, &byte, 1, 100);

        if (byte == '\n')
        {
            // Line complete — parse and display
            rx_buf[rx_idx] = '\0';
            Parse_And_Display(rx_buf);

            // Reset buffer
            rx_idx = 0;
            memset(rx_buf, 0, sizeof(rx_buf));
        }
        else if (byte != '\r')
        {
            if (rx_idx < RX_BUF_SIZE - 1)
                rx_buf[rx_idx++] = (char)byte;
        }
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
