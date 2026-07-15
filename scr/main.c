/*
需求：
①现在我使用TIM1的CH1和CH2通道捕获一路PWM信号的频率和占空比，并且使用硬件自复位。TIM1_CH1通道连接到PA3端口。预计捕获的PWM信号频率在500HZ~20KHz
②使用TIM3的CH1和CH2通道捕获另一路的PWM信号的频率和占空比，并使用硬件自复位。TIM3_CH1通道连接到PA2端口。预计捕获的PWM信号频率在500HZ~20KHz
③PB6端口复用为USART1_TX，PB7端口复用为USART1_RX，作为串口1，将TIM1和TIM3捕捉到的PWM信息（频率和占空比），每10ms打印一次到串口1钟，
④轮询任务计时，使用systick

按这个需求，帮我重新编写程序
*/
// 全局头文件与全局变量
#include "py32f0xx_hal.h"
#include <stdio.h>

/* 系统时钟参数（PY32F003标准24MHz HSI） */
#define SYS_HCLK        24000000UL
#define TIM_PSC         23U        // 24M/(23+1)=1MHz，1us计数刻度
#define TIM_CLK_FREQ    (SYS_HCLK / (TIM_PSC + 1UL))

/* TIM1 PWM1测量数据 */
uint32_t pwm1_period = 0;
uint32_t pwm1_high   = 0;
float    pwm1_freq   = 0.0f;
float    pwm1_duty   = 0.0f;

/* TIM3 PWM2测量数据 */
uint32_t pwm2_period = 0;
uint32_t pwm2_high   = 0;
float    pwm2_freq   = 0.0f;
float    pwm2_duty   = 0.0f;

/* 定时器句柄 */
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* 串口句柄 */
UART_HandleTypeDef huart1;

/* 10ms打印计时缓存 */
uint32_t print_tick_last = 0;

// 系统时钟初始化（24MHz HSI）
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHZ;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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
}

// GPIO初始化（PA2/PA3捕获输入、PB6/PB7串口)
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PA3 TIM1_CH1 复用输入捕获，内部下拉无信号时稳定低电平 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA2 TIM3_CH1 复用输入捕获 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PB6 USART1_TX AF0 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PB7 USART1_RX AF0 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// TIM1初始化（PA3 CH1+CH2 硬件PWMI复位）
static void MX_TIM1_Init(void)
{
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = TIM_PSC;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};

  /* CH1 上升沿捕获 DIRECTTI */
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0x03;
  HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1);

  /* CH2 下降沿 INDIRECTTI 复用TI1信号 */
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2);

  /* 从模式：TI1上升沿硬件复位计数器（硬件PWMI核心） */
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig);

  /* 开启捕获中断 */
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC2);

  /* NVIC中断配置 */
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}

// TIM3初始化（PA2 CH1+CH2 硬件PWMI复位）
static void MX_TIM3_Init(void)
{
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = TIM_PSC;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};

  /* CH1 上升沿捕获 DIRECTTI */
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0x03;
  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);

  /* CH2 下降沿 INDIRECTTI */
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2);

  /* TI3FP1上升沿自动复位CNT */
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig);

  /* 捕获中断使能 */
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC2);

  /* NVIC */
  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

// USART1初始化 PB6 TX / PB7 RX 9600波特
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
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

/* printf重定向到USART1 */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
  return ch;
}
 // 定时器启动封装函数
void TIM_PWMI_Start(void)
{
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  MX_TIM1_Init();
  MX_TIM3_Init();

  /* 两路PWMI同时开启捕获中断 */
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
}

// 中断服务函数与捕获回调
void TIM1_CC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* TIM1 PWM1 */
  if (htim->Instance == TIM1)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      pwm1_period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      pwm1_freq = (float)TIM_CLK_FREQ / pwm1_period;
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      pwm1_high = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
      if(pwm1_period != 0)
      {
        pwm1_duty = (float)pwm1_high / pwm1_period * 100.0f;
      }
    }
  }
  /* TIM3 PWM2 */
  else if (htim->Instance == TIM3)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      pwm2_period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      pwm2_freq = (float)TIM_CLK_FREQ / pwm2_period;
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      pwm2_high = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
      if(pwm2_period != 0)
      {
        pwm2_duty = (float)pwm2_high / pwm2_period * 100.0f;
      }
    }
  }
}

// 主函数（SysTick 10ms非阻塞打印）
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  TIM_PWMI_Start();

  print_tick_last = HAL_GetTick();

  while (1)
  {
    uint32_t now_tick = HAL_GetTick();

    /* 每10ms打印一次两路PWM数据 */
    if (now_tick - print_tick_last >= 10U)
    {
      print_tick_last = now_tick;
      printf("PWM1 Freq:%.2fHz Duty:%.1f%% | PWM2 Freq:%.2fHz Duty:%.1f%%\r\n",
             pwm1_freq, pwm1_duty, pwm2_freq, pwm2_duty);
    }
  }
}

// 错误处理函数
void Error_Handler(void)
{
  while(1);
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
