/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "gpio.h"
#include "periph.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_i2c.h"
#include "MPU6050.c"

void SystemClock_Config(void);

void MX_DMA1_Init(void)
{
    LL_DMA_SetChannelSelection(STM_DRIVER_DMA_I2C, STM_DRIVER_DMA_STREAM_I2C,STM_DRIVER_DMA_CHANNEL_I2C);
    LL_DMA_ConfigAddresses(STM_DRIVER_DMA_I2C, STM_DRIVER_DMA_STREAM_I2C,STM_DRIVER_DMA_SRC_ADDR_I2C, (uint32_t)malloc(STM_DRIVER_DMA_BUFFER_SIZE), STM_DRIVER_DMA_DIRECTION);
    LL_DMA_SetDataLength(STM_DRIVER_DMA_I2C, STM_DRIVER_DMA_STREAM_I2C,STM_DRIVER_DMA_BUFFER_SIZE);
    LL_DMA_SetMemoryIncMode(STM_DRIVER_DMA_I2C, STM_DRIVER_DMA_STREAM_I2C,STM_DRIVER_DMA_MEM_INC_MODE);
    LL_DMA_EnableStream(STM_DRIVER_DMA_I2C, STM_DRIVER_DMA_STREAM_I2C);
    LL_DMA_EnableIT_TC(STM_DRIVER_DMA_I2C, STM_DRIVER_DMA_STREAM_I2C);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    NVIC_SetPriority(STM_DRIVER_DMA_STREAM_IRQN_I2C, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),4, 0));
    NVIC_EnableIRQ(STM_DRIVER_DMA_STREAM_IRQN_I2C);
}

void MX_I2C1_Init(void)
{
    LL_I2C_InitTypeDef I2C_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
    LL_I2C_DisableOwnAddress2(I2C1);
    LL_I2C_DisableGeneralCall(I2C1);
    LL_I2C_EnableClockStretching(I2C1);
    I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
    I2C_InitStruct.ClockSpeed = 100000;
    I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
//    I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
//    I2C_InitStruct.DigitalFilter = 0;
    I2C_InitStruct.OwnAddress1 = 0;
    I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
    I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
    LL_I2C_Init(I2C1, &I2C_InitStruct);
    LL_I2C_SetOwnAddress2(I2C1, 0);
}

int main(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  LL_GPIO_SetPinMode(GPIOD,LL_GPIO_PIN_14,LL_GPIO_MODE_OUTPUT);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));


  SystemClock_Config();

  MX_GPIO_Init();
    MPU6050_Initialize();
    MX_DMA1_Init();
    MX_I2C1_Init();
    //DMA1_Stream0_IRQHandler();
  while (1)
  {
    
        LL_GPIO_SetOutputPin(GPIOD,LL_GPIO_PIN_14);
        LL_mDelay(1000);
        LL_GPIO_ResetOutputPin(GPIOD,LL_GPIO_PIN_14);
        LL_mDelay(1000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 168, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(168000000);
  LL_SetSystemCoreClock(168000000);
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

void DMA1_Stream0_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TC0(STM_DRIVER_DMA_I2C)){
            LL_DMA_ClearFlag_TC0(STM_DRIVER_DMA_I2C);
            LL_DMA_ClearFlag_HT0(STM_DRIVER_DMA_I2C);
            LL_DMA_EnableStream(STM_DRIVER_DMA_I2C, STM_DRIVER_DMA_STREAM_I2C);
    }
}

void DMA1_Channel7_IRQHandler(void)
{
    GPIOC->BSRR |= GPIO_BSRR_BS8;
    if (DMA_GetITStatus(DMA1_IT_TC7)==SET)
    {
        DMA_CleanITPendingBit(DMA1_IT_TC7);
        I2C1->CR1 &= ~LL_I2C_ACK;
        I2C1->CR1 |= LL_I2C_STOP;

        ACCEL_X=(malloc[1]<<8) | malloc[0];
        ACCEL_Y=(malloc[3]<<8) | malloc[2];
        ACCEL_Z=(malloc[5]<<8) | malloc[4];
        //TEMP=(malloc[7]<<8) | malloc[6];
        GYRO_X=(malloc[9]<<8) | malloc[8];
        GYRO_Y=(malloc[9]<<8) | malloc[8];
        GYRO_Z=(malloc[9]<<8) | malloc[8];
    }
    GPIOC->BRR |= GPIO_BRR_BS8;
}

#endif /* USE_FULL_ASSERT */

