/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : OLED Display — receives from E32 UART2,
  *                   forwards to FTDI UART1, displays Freq+RPM on OLED
  *
  * Hardware:
  *   MCU   : STM32F103C8T6 (Blue Pill)
  *   Clock : HSE 8MHz
  *   I2C1  : PB6 (SCL), PB7 (SDA)
  *   OLED  : SSD1306/SSD1309 128x64
  *   UART2 : PA2(TX) PA3(RX)  — E32 RF module
  *   UART1 : PA9(TX) PA10(RX) — FTDI to PC terminal
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rtc.h"
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

#define E32_M0_PIN          GPIO_PIN_14
#define E32_M0_PORT         GPIOC
#define E32_M1_PIN          GPIO_PIN_15
#define E32_M1_PORT         GPIOC

#define PWM_CHANNEL1        TIM_CHANNEL_1
#define PWM_DUTY_50         4999
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
static char   *Find_RPM_Ptr(char *line);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint8_t OLED_Verify(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_ADDR_PRIMARY,  3, 500) == HAL_OK) return 1;
    if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_ADDR_FALLBACK, 3, 500) == HAL_OK) return 1;
    return 0;
}

/**
 * @brief  Finds the RPM numeric value in a line regardless of
 *         how corrupted the prefix is.
 *
 *         Handles all observed variants from the transmitter:
 *           "RPM:3780"   — correct
 *           "RP:3780"    — missing M
 *           "RM:3780"    — missing P
 *           "RR:3780"    — double R
 *           "R:3780"     — only R left
 *
 *         Strategy: find the LAST occurrence of 'R' in the line
 *         that is followed eventually by ':', then read the number
 *         after the colon.
 *
 * @param  line  Null-terminated received string
 * @retval Pointer to the ':' character before the RPM digits,
 *         or NULL if no pattern found
 */
static char *Find_RPM_Ptr(char *line)
{
    /* Try exact match first */
    char *p = strstr(line, "RPM:");
    if (p != NULL) return p + 4;   /* point at digits */

    /* Fallback — try all known corrupted prefixes */
    const char *variants[] = { "RP:", "RM:", "RR:", "R:","M:","PM:","P:" };
    for (int i = 0; i < 4; i++)
    {
        p = strstr(line, variants[i]);
        if (p != NULL)
        {
            /* Make sure it is not part of "FREQ:" */
            if (p == line || *(p - 1) == ' ' || *(p - 1) == '\t')
            {
                return p + strlen(variants[i]);  /* point at digits */
            }
        }
    }
    return NULL;
}

/**
 * @brief  Parses FREQ and RPM from a received line and updates OLED.
 * @param  line  e.g. "FREQ:63 RP:3780"
 */
static void Parse_And_Display(char *line)
{
    char oled_line1[22];
    char oled_line2[22];

    uint32_t freq = 0;
    uint32_t rpm  = 0;

    /* --- Parse FREQ --- */
    char *ptr_freq = strstr(line, "FREQ:");
    if (ptr_freq != NULL)
    {
        sscanf(ptr_freq, "FREQ:%lu", &freq);
    }

    /* --- Parse RPM (tolerant of corrupted prefixes) --- */
    char *ptr_rpm = Find_RPM_Ptr(line);
    if (ptr_rpm != NULL)
    {
        sscanf(ptr_rpm, "%lu", &rpm);
    }

    /* --- Build display strings --- */
    snprintf(oled_line1, sizeof(oled_line1), "FREQ:%lu", freq);
    snprintf(oled_line2, sizeof(oled_line2), "RPM :%lu", rpm);

    /* --- Update OLED --- */
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

    /* Start PWM on TIM2 CH1 (PA0) at 50% duty cycle */
    HAL_TIM_PWM_Start(&htim2, PWM_CHANNEL1);
    __HAL_TIM_SET_COMPARE(&htim2, PWM_CHANNEL1, PWM_DUTY_50);

    /* Set E32 to normal mode: M0=0, M1=0 */
    HAL_GPIO_WritePin(E32_M0_PORT, E32_M0_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E32_M1_PORT, E32_M1_PIN, GPIO_PIN_RESET);

    /* Verify OLED present — fast blink and halt if missing */
    if (OLED_Verify() == 0)
    {
        while (1)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            HAL_Delay(100);
        }
    }

    /* Blink LED 3× to signal successful startup */
    for (int i = 0; i < 3; i++)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(300);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(300);
    }

    /* Initialise OLED and show waiting screen */
    SSD1306_Init();
    SSD1306_Fill(0);
    SSD1306_GotoXY(0, 10);
    SSD1306_Puts("FREQ: ----", &Font_7x10, 1);
    SSD1306_GotoXY(0, 35);
    SSD1306_Puts("RPM : ----", &Font_7x10, 1);
    SSD1306_UpdateScreen();

    /* Notify PC terminal */
    char header[] = "=== OLED Receiver Ready ===\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)header, strlen(header), 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        uint8_t byte;

        if (HAL_UART_Receive(&huart2, &byte, 1, 100) == HAL_OK)
        {
            /* Mirror every byte to FTDI/PC */
            HAL_UART_Transmit(&huart1, &byte, 1, 100);

            if (byte == '\n')
            {
                /* Complete line — parse and display */
                rx_buf[rx_idx] = '\0';
                Parse_And_Display(rx_buf);

                /* Short blink per line */
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                HAL_Delay(50);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

                /* Reset buffer */
                rx_idx = 0;
                memset(rx_buf, 0, sizeof(rx_buf));
            }
            else if (byte != '\r')
            {
                if (rx_idx < RX_BUF_SIZE - 1)
                    rx_buf[rx_idx++] = (char)byte;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
