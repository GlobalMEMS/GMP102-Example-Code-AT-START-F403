/*
 *
 ****************************************************************************
 * Copyright (C) 2018 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : main.c
 *
 * Usage: main function
 *
 ****************************************************************************
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 **************************************************************************/

/*! @file main.c
 *  @brief main program
 *  @author Joseph FC Tseng
 */

/* Includes ------------------------------------------------------------------*/
#include "i2c_gmems.h"
#include "bus_support.h"
#include "gmp102.h"
#include "pSensor_util.h"

/* Private macro -------------------------------------------------------------*/

/* global variables ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
USART_InitType USART_InitStructure;
static __IO uint32_t TimingDelay;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Inserts a delay time.
 * @param  nTime: specifies the delay time length, in milliseconds.
 * @retval None
 */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
 * @brief  Decrements the TimingDelay variable.
 * @param  None
 * @retval None
 */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00){
    TimingDelay--;
  }
}

/**
 * @brief  Retargets the C library printf function to the USART1.
 * @param
 * @retval
 */
int fputc(int ch, FILE *f)
{
  while((USART1->STS & 0X40) == 0)
    ;

  USART1->DT = (u8)ch;
  return ch;
}

/**
 * @brief  Configures the nested vectored interrupt controller.
 * @param  None
 * @retval None
 */
void NVIC_Configuration(void)
{
  NVIC_InitType NVIC_InitStructure;

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  Configures COM port.
 * @param  None
 * @retval None
 */
void USART_COMInit()
{

  GPIO_InitType GPIO_InitStructure;

  /* USARTx configured as follow:
     - BaudRate = 115200 baud
     - Word Length = 8 Bits
     - One Stop Bit
     - No parity check
     - Hardware flow control disabled (RTS and CTS signals)
     - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA, ENABLE);

  /* Enable UART clock */
  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_USART1, ENABLE);

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pins = TX_PIN_NUMBER;
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
  GPIO_Init(TXRX_GPIOx, &GPIO_InitStructure);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pins = RX_PIN_NUMBER;
  GPIO_Init(TXRX_GPIOx, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(USART1, &USART_InitStructure);

  /* Enable USART */
  USART_Cmd(USART1, ENABLE);

  /* Enable the EVAL_COM1 Receive interrupt: this interrupt is generated when the
     EVAL_COM1 receive data register is not empty */
  USART_INTConfig(USART1, USART_INT_RDNE, ENABLE);
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
    {}
}

#endif

/**
 * @brief   Main program
 * @param  None
 * @retval None
 */
int main(void)
{
  RCC_ClockType RccClkSource;
  bus_support_t gmp102_bus;
  float fCalibParam[GMP102_CALIBRATION_PARAMETER_COUNT], fT_Celsius, fP_Pa, fAlt_m;
  s16 s16T;
  s32 s32P;

  /* NVIC configuration */
  NVIC_Configuration();

  /* USART COM configuration */
  USART_COMInit();

  /* I2C1 initialization */
  I2C1_Init();

  RCC_GetClocksFreq(&RccClkSource);
  if (SysTick_Config(RccClkSource.AHBCLK_Freq / 1000)){
    /* Capture error */
    while(1);
  }

  /* GMP102 I2C bus setup */
  bus_init_I2C1(&gmp102_bus, GMP102_8BIT_I2C_ADDR);  //Initialize bus support to I2C1
  gmp102_bus_init(&gmp102_bus);  //Initailze GMP102 bus to I2C1

  /* GMP102 soft reset */
  gmp102_soft_reset();

  /* Wait 100ms for reset complete */
  Delay(100);

  /* GMP102 get the pressure calibration parameters */
  gmp102_get_calibration_param(fCalibParam);

  /* GMP102 initialization setup */
  gmp102_initialization();

	/* Select one of the following modes to suit your appliation */
	/* Refer to https://github.com/GlobalMEMS/Application-Notes/blob/master/GMP102%20Application%20Note%20on%20OSR%20setting%20V1.0.pdf */
	/* for more decriptions.                                     */
	/*                                                           */
	//gmp102_set_P_OSR(GMP102_P_OSR_256);   //Ultra low power mode
  gmp102_set_P_OSR(GMP102_P_OSR_1024);  //Low power mode
	//gmp102_set_P_OSR(GMP102_P_OSR_4096);  //Standard resolution
	//gmp102_set_P_OSR(GMP102_P_OSR_8192);  //High resolution
	//gmp102_set_P_OSR(GMP102_P_OSR_16384); //Ultra high resolution

  /* set sea leve reference pressure */
  //If not set, use default 101325 Pa for pressure altitude calculation
  set_sea_level_pressure_base(100110.f);

  while (1){

    /* Measure P */
    gmp102_measure_P(&s32P);
    printf("P(code)=%d\r", s32P);

    /* Mesaure T */
    gmp102_measure_T(&s16T);
    printf("T(code)=%d\r", s16T);

    /* Compensation */
    gmp102_compensation(s16T, s32P, fCalibParam, &fT_Celsius, &fP_Pa);
    printf("P(Pa)=%.1f\r", fP_Pa);
    printf("T(C)=%.3f\r", fT_Celsius);

    /* Pressure Altitude */
    fAlt_m = pressure2Alt(fP_Pa);
    printf("Alt(m)=%.3f\r", fAlt_m);

    printf("\n");

    /* Delay 1 sec */
    Delay(1000);
  }
}

