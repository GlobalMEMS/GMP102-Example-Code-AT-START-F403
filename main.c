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
#include "Lcd_Driver.h"
#include "GUI.h"
#include "usart.h"
#include "delay.h"
#include "key.h"
#include "string.h"
#include "math.h"

/* Private macro -------------------------------------------------------------*/

/* global variables ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

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

const u16 RESOLUTION_X = 128;
const u16 RESOLUTION_Y = 120;
const u16 FONT_HEIGHT = 16;
const u16 LINE_HEIGHT = FONT_HEIGHT + 2;
const u16 MAX_DISPLAY_ITEM = 9;
void showMsg(u16 x, u16 line, u8* str, u16 color, u8 reDraw){

  int i;
  char* subStr;

  if(reDraw) Lcd_Clear(GRAY0);

  subStr = strtok((char*)str, "\n");

  for(i = line; subStr; ++i){
    Gui_DrawFont_GBK16(x, LINE_HEIGHT * i, color, GRAY0, (u8*)subStr);
    subStr = strtok(NULL, "\n");
  }
}

void floatCatToStr(float fIn, u8 precision, u8* outStr){

  s32 i = 0;
  float fTmp;
  s32 s32Dec, s32Dig;

  if(fIn < 0){
    fIn = -fIn;
    strcat((char*)outStr, "-");
  }

  s32Dec = (s32)fIn;
  fTmp = fIn - s32Dec;
  for(i = 0; i < precision; ++i)
    fTmp *= 10;
  s32Dig = (s32)(fTmp + 0.5f);

  itoa(s32Dec, &outStr[strlen((const char*)outStr)]);
  strcat((char*)outStr, ".");

  fTmp = 1;
  for(i = 0; i < precision; ++i)
    fTmp *= 10;
  for(i = 0; i < precision; ++i){
    fTmp /= 10;
    if(s32Dig < fTmp){
      strcat((char*)outStr, "0");
    }
    else{
      itoa(s32Dig, &outStr[strlen((const char*)outStr)]);
      break;
    }
  }
}

/**
 * @brief   Main program
 * @param  None
 * @retval None
 */
int main(void)
{
  bus_support_t gmp102_bus;
  float fCalibParam[GMP102_CALIBRATION_PARAMETER_COUNT], fT_Celsius, fP_Pa, fAlt_m;
	s16 s16Value[GMP102_CALIBRATION_PARAMETER_COUNT];
	u8 u8Power[GMP102_CALIBRATION_PARAMETER_COUNT];
  s16 s16T;
  s32 s32P, s32P64_Pa, s32P32_Pa, s32T;
  u8 str[64];

  /* System Initialization */
  SystemInit();

  /* I2C1 initialization */
  I2C1_Init();

  /* Init Key */
  KEY_Init();

  /* Initialize the LCD */
  uart_init(19200);
  delay_init();
  Lcd_Init();

  /* GMP102 I2C bus setup */
  bus_init_I2C1(&gmp102_bus, GMP102_8BIT_I2C_ADDR);  //Initialize bus support to I2C1
  gmp102_bus_init(&gmp102_bus);  //Initailze GMP102 bus to I2C1

  /* GMP102 soft reset */
  gmp102_soft_reset();

  /* Wait 100ms for reset complete */
  delay_ms(100);

  /* GMP102 get the pressure calibration parameters */
  gmp102_get_calibration_param(fCalibParam);
	
	/* GMP102 get the pressure calibration parameters, fixed point format */
	gmp102_get_calibration_param_fixed_point(s16Value, u8Power);

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

  strcpy((char*)str, "P, T(code, code):");
  showMsg(0, 0, str, BLACK, 1);
  strcpy((char*)str, "P (Pa):");
  showMsg(0, 3, str, BLACK, 0);
	strcpy((char*)str, "P64 Pa:");
  showMsg(0, 4, str, BLACK, 0);
	strcpy((char*)str, "P32 Pa:");
  showMsg(0, 5, str, BLACK, 0);
  strcpy((char*)str, "T (C):");
  showMsg(0, 7, str, BLACK, 0);
  strcpy((char*)str, "Atl (m):");
  showMsg(0, 8, str, BLACK, 0);

  while (1){

    /* Measure P */
    gmp102_measure_P(&s32P);

    /* Mesaure T */
    gmp102_measure_T(&s16T);

    /* Compensation */
    gmp102_compensation(s16T, s32P, fCalibParam, &fT_Celsius, &fP_Pa);
		
		/* Compensation s64 */
		gmp102_compensation_fixed_point_s64(s16T, s32P, s16Value, u8Power, &s32T, &s32P64_Pa);
		
		/* Compensation s32 */
		gmp102_compensation_fixed_point_s32(s16T, s32P, s16Value, u8Power, &s32T, &s32P32_Pa);		

    /* Pressure Altitude */
    fAlt_m = pressure2Alt(fP_Pa);

    /* User message: Raw data*/
    strcpy((char*)str, "");
    itoa(s32P, &str[strlen((const char*)str)]);
    strcat((char*)str, ", ");
    itoa(s16T, &str[strlen((const char*)str)]);
    strcat((char*)str, "       ");
    showMsg(0, 1, str, BLUE, 0);

    /* User message: compensated P in Pa*/
    strcpy((char*)str, "");
    floatCatToStr(fP_Pa, 1, str);
    strcat((char*)str, "       ");
    showMsg(60, 3, str, BLUE, 0);
		
		/* User message: compensated P64 in Pa*/
    strcpy((char*)str, "");
    itoa(s32P64_Pa, &str[strlen((const char*)str)]);
    strcat((char*)str, "       ");
    showMsg(60, 4, str, BLUE, 0);
		
		/* User message: compensated P64 in Pa*/
    strcpy((char*)str, "");
    itoa(s32P32_Pa, &str[strlen((const char*)str)]);
    strcat((char*)str, "       ");
    showMsg(60, 5, str, BLUE, 0);		

    /* User message: T in Celsius */
    strcpy((char*)str, "");
    floatCatToStr(fT_Celsius, 2, str);
    strcat((char*)str, "       ");
    showMsg(70, 7, str, BLUE, 0);

    /* User message: Altitude */
    strcpy((char*)str, "");
    floatCatToStr(fAlt_m, 2, str);
    strcat((char*)str, "       ");
    showMsg(70, 8, str, BLUE, 0);

    /* Delay 1 sec */
    delay_ms(1000);
  }
}

