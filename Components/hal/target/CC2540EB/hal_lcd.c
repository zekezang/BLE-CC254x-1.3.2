/**************************************************************************************************
  文件名:       hal_lcd.c
  修改日期:     2013-02-03
  作者:         ghostyu在TI原有基础上修改

**************************************************************************************************/

/**************************************************************************************************
 *                                           INCLUDES
 **************************************************************************************************/
#include "hal_types.h"
#include "hal_lcd.h"
#include "OSAL.h"
#include "OnBoard.h"
#include "hal_assert.h"

#if defined (ZTOOL_P1) || defined (ZTOOL_P2)
  #include "DebugTrace.h"
#endif

/**************************************************************************************************
 *                                          CONSTANTS
 **************************************************************************************************/
/*
  LCD pins

Version-1:
  //control
  P0.0 - LCD_MODE
  P1.2 - LCD_CS

  //spi
  P1.5 - CLK
  P1.6 - MOSI
  P1.7 - MISO

Version-2:
  //control
  P1.7 - LCD_MODE(由MISO复用,LCD并未使用到MISO引脚)
  P1.2 - LCD_CS

  //spi
  P1.5 - CLK
  P1.6 - MOSI
  P1.7 - MISO

*/
#ifdef CC2530DK_V1
/* LCD Control lines */
#define HAL_LCD_MODE_PORT 0
#define HAL_LCD_MODE_PIN  0
#else
/* LCD Control lines */
#define HAL_LCD_MODE_PORT 1
#define HAL_LCD_MODE_PIN  7
#endif

#define HAL_LCD_CS_PORT 1
#define HAL_LCD_CS_PIN  2

/* LCD SPI lines */
#define HAL_LCD_CLK_PORT 1
#define HAL_LCD_CLK_PIN  5

#define HAL_LCD_MOSI_PORT 1
#define HAL_LCD_MOSI_PIN  6

#define HAL_LCD_MISO_PORT 1
#define HAL_LCD_MISO_PIN  7

/* SPI settings */
#define HAL_SPI_CLOCK_POL_LO       0x00
#define HAL_SPI_CLOCK_PHA_0        0x00
#define HAL_SPI_TRANSFER_MSB_FIRST 0x20

/* LCD lines */
#define LCD_MAX_LINE_COUNT              8
#define LCD_MAX_LINE_LENGTH             21
#define LCD_MAX_BUF                     25



/**************************************************************************************************
 *                                           MACROS
 **************************************************************************************************/

#define HAL_IO_SET(port, pin, val)        HAL_IO_SET_PREP(port, pin, val)
#define HAL_IO_SET_PREP(port, pin, val)   st( P##port##_##pin## = val; )

#define HAL_CONFIG_IO_OUTPUT(port, pin, val)      HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val)
#define HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val) st( P##port##SEL &= ~BV(pin); \
                                                      P##port##_##pin## = val; \
                                                      P##port##DIR |= BV(pin); )

#define HAL_CONFIG_IO_PERIPHERAL(port, pin)      HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin)
#define HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin) st( P##port##SEL |= BV(pin); )



/* SPI interface control */
#define LCD_SPI_BEGIN()     HAL_IO_SET(HAL_LCD_CS_PORT,  HAL_LCD_CS_PIN,  0); /* chip select */
#define LCD_SPI_END()                                                         \
{                                                                             \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  HAL_IO_SET(HAL_LCD_CS_PORT,  HAL_LCD_CS_PIN,  1); /* chip select */         \
}
/* clear the received and transmit byte status, write tx data to buffer, wait till transmit done */
#define LCD_SPI_TX(x)                   { U1CSR &= ~(BV(2) | BV(1)); U1DBUF = x; while( !(U1CSR & BV(1)) ); }
#define LCD_SPI_WAIT_RXRDY()            { while(!(U1CSR & BV(1))); }



/* Control macros */
#define LCD_DO_WRITE()        HAL_IO_SET(HAL_LCD_MODE_PORT,  HAL_LCD_MODE_PIN,  1);
#define LCD_DO_CONTROL()      HAL_IO_SET(HAL_LCD_MODE_PORT,  HAL_LCD_MODE_PIN,  0);


#if (HAL_LCD == TRUE)
/**************************************************************************************************
 *                                       LOCAL VARIABLES
 **************************************************************************************************/

static uint8 *Lcd_Line1;
const uint8 ascii_table_5x7[95][5];
/**************************************************************************************************
 *                                       FUNCTIONS - API
 **************************************************************************************************/

void HalLcd_HW_Init(void);
void HalLcd_HW_WaitUs(uint16 i);
void HalLcd_HW_Clear(void);
void HalLcd_HW_ClearAllSpecChars(void);
void HalLcd_HW_Control(uint8 cmd);
void HalLcd_HW_Write(uint8 data);
void HalLcd_HW_SetContrast(uint8 value);
void HalLcd_HW_WriteChar(uint8 line, uint8 col, char text);
void HalLcd_HW_WriteLine(uint8 line, const char *pText);

//ghostyu添加
void set_ddram_line_col(uint8 line,uint8 col);
void DisplayByte_5x7(uint8 page,uint8 column,uint8 text);


#endif //LCD

#if (HAL_LCD == TRUE)
/*
作用    设置LCD 文本显示的其实行和列
参数1   line,范围:0~7,即能够显示的行为1~8行，也就是lcd手册里提到的page
参数2   col,范围:0~127,即lcd的总列数，显示的起始位置可以设置到每一列
*/
void set_ddram_line_col(uint8 line,uint8 col)
{

  uint8 page,coll,coll_l,coll_h;
  page = line;
  coll = col;
  coll_h = coll>>4;
  coll_l = coll&0x0f;
  HalLcd_HW_Control(0xB0+page);
  HalLcd_HW_WaitUs(15); // 15 us
  HalLcd_HW_Control(0x10+coll_h);
  HalLcd_HW_WaitUs(15); // 15 us
  HalLcd_HW_Control(0x00+coll_l);
  HalLcd_HW_WaitUs(15); // 15 us
}

/*
作用     显示一个字节的字符，该字符大小为宽5个点，高7个点
参数1    page,范围0~7,共8行
参数2    column,范围0~127
参数3    text,要显示的字符，该值为ascii码
*/
void DisplayByte_5x7(uint8 page,uint8 column,uint8 text)
{
	int j,k;

	if((text>=0x20)&&(text<0x7e)){/*需要显示的文字*/
		j=text-0x20;/*寻址，通过字符的ascii码找到点阵库中的改字符的位置*/
		set_ddram_line_col(page,column);
		for(k=0;k<5;k++)
		{
			HalLcd_HW_Write(ascii_table_5x7[j][k]);/*显示5x7的ASCII字到LCD上，y为页地址，x为列地址，最后为数据*/
		}
		//第六列写入0，即清除上一次留下来的数据
		HalLcd_HW_Write(0x00);
		column+=6;
	}else if(text==0x00){/*不需要显示，清空指定位置*/
		set_ddram_line_col(page,column);
		for(k=0;k<5;k++){
			HalLcd_HW_Write(0x00); //清空指定的字符位置
		}
	}

}


#endif

/**************************************************************************************************
z-stack代码,未修改
 **************************************************************************************************/
void HalLcdInit(void)
{
#if (HAL_LCD == TRUE)
  Lcd_Line1 = NULL;
  HalLcd_HW_Init();
#endif
}

/*************************************************************************************************
 *                    LCD EMULATION FUNCTIONS
 *
 * Some evaluation boards are equipped with Liquid Crystal Displays
 * (LCD) which may be used to display diagnostic information. These
 * functions provide LCD emulation, sending the diagnostic strings
 * to Z-Tool via the RS232 serial port. These functions are enabled
 * when the "LCD_SUPPORTED" compiler flag is placed in the makefile.
 *
 * Most applications update both lines (1 and 2) of the LCD whenever
 * text is posted to the device. This emulator assumes that line 1 is
 * updated first (saved locally) and the formatting and send operation
 * is triggered by receipt of line 2. Nothing will be transmitted if
 * only line 1 is updated.
 *
 *************************************************************************************************/


/**************************************************************************************************
z-stack代码,未修改
 **************************************************************************************************/
void HalLcdWriteString ( char *str, uint8 option)
{
#if (HAL_LCD == TRUE)

  uint8 strLen = 0;
  uint8 totalLen = 0;
  uint8 *buf;
  uint8 tmpLen;

  if ( Lcd_Line1 == NULL )
  {
    Lcd_Line1 = osal_mem_alloc( HAL_LCD_MAX_CHARS+1 );
    HalLcdWriteString( "TexasInstruments", 1 );
  }

  strLen = (uint8)osal_strlen( (char*)str );

  /* Check boundries */
  if ( strLen > HAL_LCD_MAX_CHARS )
    strLen = HAL_LCD_MAX_CHARS;

  if ( option == HAL_LCD_LINE_1 )
  {
    /* Line 1 gets saved for later */
    osal_memcpy( Lcd_Line1, str, strLen );
    Lcd_Line1[strLen] = '\0';
  }
  else
  {
    /* Line 2 triggers action */
    tmpLen = (uint8)osal_strlen( (char*)Lcd_Line1 );
    totalLen =  tmpLen + 1 + strLen + 1;
    buf = osal_mem_alloc( totalLen );
    if ( buf != NULL )
    {
      /* Concatenate strings */
      osal_memcpy( buf, Lcd_Line1, tmpLen );
      buf[tmpLen++] = ' ';
      osal_memcpy( &buf[tmpLen], str, strLen );
      buf[tmpLen+strLen] = '\0';

      /* Send it out */
#if defined (ZTOOL_P1) || defined (ZTOOL_P2)

#if defined(SERIAL_DEBUG_SUPPORTED)
      debug_str( (uint8*)buf );
#endif //LCD_SUPPORTED

#endif //ZTOOL_P1

      /* Free mem */
      osal_mem_free( buf );
    }
  }

  /* Display the string */
  HalLcd_HW_WriteLine (option, str);

#endif //HAL_LCD

}

/**************************************************************************************************
 * @fn      HalLcdWriteValue
 *
 * @brief   Write a value to the LCD,
            向lcd指定行写入一个32位的值
 *
 * @param   value  - value that will be displayed,
            需要显示的32位数
 *          radix  - 8, 10, 16,
            进制，8进制显示 10进制显示，16进制显示
 *          option - display options,
            指定行显示
 *
 * @return  None
 **************************************************************************************************/
void HalLcdWriteValue ( uint32 value, const uint8 radix, uint8 option)
{
#if (HAL_LCD == TRUE)
  uint8 buf[LCD_MAX_BUF];

  _ltoa( value, &buf[0], radix );
  HalLcdWriteString( (char*)buf, option );
#endif
}

/**************************************************************************************************
z-stack代码,未修改
 **************************************************************************************************/
void HalLcdWriteScreen( char *line1, char *line2 )
{
#if (HAL_LCD == TRUE)
  HalLcdWriteString( line1, 1 );
  HalLcdWriteString( line2, 2 );
#endif
}

/**************************************************************************************************
z-stack代码,未修改
 **************************************************************************************************/
void HalLcdWriteStringValue( char *title, uint16 value, uint8 format, uint8 line )
{
#if (HAL_LCD == TRUE)
  uint8 tmpLen;
  uint8 buf[LCD_MAX_BUF];
  uint32 err;

  tmpLen = (uint8)osal_strlen( (char*)title );
  osal_memcpy( buf, title, tmpLen );
  buf[tmpLen] = ' ';
  err = (uint32)(value);
  _ltoa( err, &buf[tmpLen+1], format );
  HalLcdWriteString( (char*)buf, line );		
#endif
}

/**************************************************************************************************
z-stack代码,未修改
 **************************************************************************************************/
void HalLcdWriteStringValueValue( char *title, uint16 value1, uint8 format1,
                                  uint16 value2, uint8 format2, uint8 line )
{

#if (HAL_LCD == TRUE)

  uint8 tmpLen;
  uint8 buf[LCD_MAX_BUF];
  uint32 err;

  tmpLen = (uint8)osal_strlen( (char*)title );
  if ( tmpLen )
  {
    osal_memcpy( buf, title, tmpLen );
    buf[tmpLen++] = ' ';
  }

  err = (uint32)(value1);
  _ltoa( err, &buf[tmpLen], format1 );
  tmpLen = (uint8)osal_strlen( (char*)buf );

  buf[tmpLen++] = ',';
  buf[tmpLen++] = ' ';
  err = (uint32)(value2);
  _ltoa( err, &buf[tmpLen], format2 );

  HalLcdWriteString( (char *)buf, line );		

#endif
}

/**************************************************************************************************
z-stack代码,未修改
 **************************************************************************************************/
void HalLcdDisplayPercentBar( char *title, uint8 value )
{
#if (HAL_LCD == TRUE)

  uint8 percent;
  uint8 leftOver;
  uint8 buf[17];
  uint32 err;
  uint8 x;

  /* Write the title: */
  HalLcdWriteString( title, HAL_LCD_LINE_1 );

  if ( value > 100 )
    value = 100;

  /* convert to blocks */
  percent = (uint8)(value / 10);
  leftOver = (uint8)(value % 10);

  /* Make window */
  osal_memcpy( buf, "[          ]  ", 15 );

  for ( x = 0; x < percent; x ++ )
  {
    buf[1+x] = '>';
  }

  if ( leftOver >= 5 )
    buf[1+x] = '+';

  err = (uint32)value;
  _ltoa( err, (uint8*)&buf[13], 10 );

  HalLcdWriteString( (char*)buf, HAL_LCD_LINE_2 );

#endif

}

#if (HAL_LCD == TRUE)
/**************************************************************************************************
 *                                    HARDWARE LCD
 **************************************************************************************************/

/**************************************************************************************************
lcd所需的GPIO口配置
 **************************************************************************************************/
static void halLcd_ConfigIO(void)
{
  /* GPIO configuration */
#ifdef CC2530DK_V1
  HAL_CONFIG_IO_OUTPUT(HAL_LCD_MODE_PORT,  HAL_LCD_MODE_PIN,  1);
#else

#endif
  HAL_CONFIG_IO_OUTPUT(HAL_LCD_CS_PORT,    HAL_LCD_CS_PIN,    1);
}

/**************************************************************************************************
SPI总线寄存器配置
 **************************************************************************************************/
static void halLcd_ConfigSPI(void)
{
  /* UART/SPI Peripheral configuration */

   uint8 baud_exponent;
   uint8 baud_mantissa;

  /* Set SPI on UART 1 alternative 2 */
  PERCFG |= 0x02;

  /* Configure clk, master out and master in lines */
  HAL_CONFIG_IO_PERIPHERAL(HAL_LCD_CLK_PORT,  HAL_LCD_CLK_PIN);
  HAL_CONFIG_IO_PERIPHERAL(HAL_LCD_MOSI_PORT, HAL_LCD_MOSI_PIN);
  HAL_CONFIG_IO_PERIPHERAL(HAL_LCD_MISO_PORT, HAL_LCD_MISO_PIN);


  /* Set SPI speed to 1 MHz (the values assume system clk of 32MHz)
   * Confirm on board that this results in 1MHz spi clk.
   */
  baud_exponent = 15;
  baud_mantissa =  0;

  /* Configure SPI */
  U1UCR  = 0x80;      /* Flush and goto IDLE state. 8-N-1. */
  U1CSR  = 0x00;      /* SPI mode, master. */
  U1GCR  = HAL_SPI_TRANSFER_MSB_FIRST | HAL_SPI_CLOCK_PHA_0 | HAL_SPI_CLOCK_POL_LO | baud_exponent;
  U1BAUD = baud_mantissa;
}

/**************************************************************************************************
初始化，已根据lcd12864手册更改
 **************************************************************************************************/
void HalLcd_HW_Init(void)
{
  /* Initialize LCD IO lines */
  halLcd_ConfigIO();

  /* Initialize SPI */
  halLcd_ConfigSPI();

	HalLcd_HW_Control(0xe2);	//软复位
	HalLcd_HW_WaitUs(15000); // 15 us
	HalLcd_HW_Control(0x2c);	//升压步聚1
	HalLcd_HW_WaitUs(15); // 15 us
	HalLcd_HW_Control(0x2e);	//升压步聚2
	HalLcd_HW_WaitUs(15); // 15 us
	HalLcd_HW_Control(0x2f);	//升压步聚3
	HalLcd_HW_WaitUs(150); // 15 us
	HalLcd_HW_Control(0x23);	//粗调对比度，可设置范围0x20～0x27
	HalLcd_HW_WaitUs(15); // 15 us
	HalLcd_HW_Control(0x81);	//微调对比度
	HalLcd_HW_WaitUs(15); // 15 us
	HalLcd_HW_Control(0x28);	//0x1a,微调对比度的值，可设置范围0x00～0x3f
	HalLcd_HW_WaitUs(15); // 15 us
	
	HalLcd_HW_Control(0xa2);	// 1/9偏压比（bias）
	HalLcd_HW_WaitUs(15); // 15 us
	HalLcd_HW_Control(0xa0);	//行扫描顺序：从上到下
	HalLcd_HW_WaitUs(15); // 15 us
	HalLcd_HW_Control(0xc8);	//列扫描顺序：从左到右
	HalLcd_HW_WaitUs(15); // 15 us
	HalLcd_HW_Control(0x40);	//起始行：第一行开始
	HalLcd_HW_WaitUs(15); // 15 us
	HalLcd_HW_Control(0xaf);	//打开显示
	HalLcd_HW_WaitUs(15); // 15 us
    HalLcd_HW_Control(0xa4);	
	HalLcd_HW_WaitUs(15); // 15 us
        
	HalLcd_HW_Clear();
        //display_string_5x7(1,1,"abcdefghijklmnopqrstuvwxyz");
        //display_string_5x7(8,1,"uvwxyz0123456789");
        //while(1);


}

/**************************************************************************************************
z-stack代码,SPI总线写命令
 **************************************************************************************************/
void HalLcd_HW_Control(uint8 cmd)
{
#ifdef CC2530DK_V1
  //do nothiing
#else
  //保存原来MISO引脚的设置
  uint8 dir = P1DIR;
  uint8 sel = P1SEL;
  P1DIR |=BV(7);
  /*
  0为GPIO,1为外设，
  这里用到的是MISO引脚的GPIO功能，
  因此相应引脚设为0?  */
  P1SEL &=~(BV(7));
#endif
  LCD_SPI_BEGIN();
  LCD_DO_CONTROL();
  LCD_SPI_TX(cmd);
  LCD_SPI_WAIT_RXRDY();
  LCD_SPI_END();

#ifdef CC2530DK_V1
  //do nothiing
#else
  P1DIR =dir;
  P1SEL =sel;
#endif
}

/**************************************************************************************************
z-stack代码,SPI总线写数据
 **************************************************************************************************/
void HalLcd_HW_Write(uint8 data)
{
#ifdef CC2530DK_V1
  //do nothiing
#else
  //保存原来MISO引脚的设置，方向和功能选择
  uint8 dir = P1DIR;
  uint8 sel = P1SEL;
  /*
  0为input 1为output
  这里要设为输出，控制lcd的A0(命令或数据选择)
  */
  P1DIR |=BV(7);
  /*
  0为GPIO,1为外设，
  这里用到的是MISO引脚的GPIO功能，
  因此相应引脚设为0?  */
  P1SEL &=~(BV(7));
#endif
  LCD_SPI_BEGIN();
  LCD_DO_WRITE();
  LCD_SPI_TX(data);
  LCD_SPI_WAIT_RXRDY();
  LCD_SPI_END();
#ifdef CC2530DK_V1
  //do nothiing
#else
  P1DIR =dir;
  P1SEL =sel;
#endif
}

/**************************************************************************************************
z-stack代码
 **************************************************************************************************/
void HalLcd_HW_SetContrast(uint8 value)
{

}

/**************************************************************************************************
作用    清屏
 **************************************************************************************************/
void HalLcd_HW_Clear(void)
{

  int i,j;
  for(i=0;i<8;i++){
        set_ddram_line_col(i,0);
	for(j=0;j<128;j++){
		HalLcd_HW_Write(0x00);
	}
  }
}

/**************************************************************************************************
z-stack代码
 **************************************************************************************************/
void HalLcd_HW_ClearAllSpecChars(void)
{

}

/**************************************************************************************************
作用    向指定的行和列写入一个字符
参数1   line，范围1~8,带显示的行,注意这里的范围是1~8,而不是0~7,目的是兼容上层的代码
参数2   col，范围1~LCD_MAX_LINE_LENGTH,待显示的列,注意，这里将128等分成LCD_MAX_LINE_LENGTH个区域，每个区域显示一个字符
参数3   text，需要显示的ascii字符
 **************************************************************************************************/
void HalLcd_HW_WriteChar(uint8 line, uint8 col, char text)
{
	uint8 column = 1+col*6;
	uint8 page = line-1;
	if(col > LCD_MAX_LINE_LENGTH)/*超出部分不显示*/
		return;
	DisplayByte_5x7(page,column,(unsigned char)text);
}

/**************************************************************************************************
作用    向指定行写入一串字符串
参数1   line。范围1~8
参数2   pText。待显示的字符串
 **************************************************************************************************/
void HalLcd_HW_WriteLine(uint8 line, const char *pText)
{
  uint8 count;
  uint8 totalLength = (uint8)osal_strlen( (char *)pText );
  /* Write the content first */
  for (count=0; count<totalLength; count++)
  {
    HalLcd_HW_WriteChar(line, count, (*(pText++)));
  }

  /* Write blank spaces to rest of the line */
  for(count=totalLength; count<LCD_MAX_LINE_LENGTH;count++)
  {
    HalLcd_HW_WriteChar(line, count, 0x00);
  }
}

/**************************************************************************************************
 * @fn      HalLcd_HW_WaitUs
 *
 * @brief   wait for x us. @ 32MHz MCU clock it takes 32 "nop"s for 1 us delay.
 *
 * @param   x us. range[0-65536]
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_WaitUs(uint16 microSecs)
{
  while(microSecs--)
  {
    /* 32 NOPs == 1 usecs */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
}

#if 1
/*全体ASCII 列表:5x7点阵库*/
const uint8  ascii_table_5x7[95][5]={
0x00,0x00,0x00,0x00,0x00,//space
0x00,0x00,0x4f,0x00,0x00,//!
0x00,0x07,0x00,0x07,0x00,//"
0x14,0x7f,0x14,0x7f,0x14,//#
0x24,0x2a,0x7f,0x2a,0x12,//$
0x23,0x13,0x08,0x64,0x62,//%
0x36,0x49,0x55,0x22,0x50,//&
0x00,0x05,0x07,0x00,0x00,//]
0x00,0x1c,0x22,0x41,0x00,//(
0x00,0x41,0x22,0x1c,0x00,//)
0x14,0x08,0x3e,0x08,0x14,//*
0x08,0x08,0x3e,0x08,0x08,//+
0x00,0x50,0x30,0x00,0x00,//,
0x08,0x08,0x08,0x08,0x08,//-
0x00,0x60,0x60,0x00,0x00,//.
0x20,0x10,0x08,0x04,0x02,///
0x3e,0x51,0x49,0x45,0x3e,//0
0x00,0x42,0x7f,0x40,0x00,//1
0x42,0x61,0x51,0x49,0x46,//2
0x21,0x41,0x45,0x4b,0x31,//3
0x18,0x14,0x12,0x7f,0x10,//4
0x27,0x45,0x45,0x45,0x39,//5
0x3c,0x4a,0x49,0x49,0x30,//6
0x01,0x71,0x09,0x05,0x03,//7
0x36,0x49,0x49,0x49,0x36,//8
0x06,0x49,0x49,0x29,0x1e,//9
0x00,0x36,0x36,0x00,0x00,//:
0x00,0x56,0x36,0x00,0x00,//;
0x08,0x14,0x22,0x41,0x00,//<
0x14,0x14,0x14,0x14,0x14,//=
0x00,0x41,0x22,0x14,0x08,//>
0x02,0x01,0x51,0x09,0x06,//?
0x32,0x49,0x79,0x41,0x3e,//@
0x7e,0x11,0x11,0x11,0x7e,//A
0x7f,0x49,0x49,0x49,0x36,//B
0x3e,0x41,0x41,0x41,0x22,//C
0x7f,0x41,0x41,0x22,0x1c,//D
0x7f,0x49,0x49,0x49,0x41,//E
0x7f,0x09,0x09,0x09,0x01,//F
0x3e,0x41,0x49,0x49,0x7a,//G
0x7f,0x08,0x08,0x08,0x7f,//H
0x00,0x41,0x7f,0x41,0x00,//I
0x20,0x40,0x41,0x3f,0x01,//J
0x7f,0x08,0x14,0x22,0x41,//K
0x7f,0x40,0x40,0x40,0x40,//L
0x7f,0x02,0x0c,0x02,0x7f,//M
0x7f,0x04,0x08,0x10,0x7f,//N
0x3e,0x41,0x41,0x41,0x3e,//O
0x7f,0x09,0x09,0x09,0x06,//P
0x3e,0x41,0x51,0x21,0x5e,//Q
0x7f,0x09,0x19,0x29,0x46,//R
0x46,0x49,0x49,0x49,0x31,//S
0x01,0x01,0x7f,0x01,0x01,//T
0x3f,0x40,0x40,0x40,0x3f,//U
0x1f,0x20,0x40,0x20,0x1f,//V
0x3f,0x40,0x38,0x40,0x3f,//W
0x63,0x14,0x08,0x14,0x63,//X
0x07,0x08,0x70,0x08,0x07,//Y
0x61,0x51,0x49,0x45,0x43,//Z
0x00,0x7f,0x41,0x41,0x00,//[
0x02,0x04,0x08,0x10,0x20,// 斜杠
0x00,0x41,0x41,0x7f,0x00,//]
0x04,0x02,0x01,0x02,0x04,//^
0x40,0x40,0x40,0x40,0x40,//_
0x01,0x02,0x04,0x00,0x00,//`
0x20,0x54,0x54,0x54,0x78,//a
0x7f,0x48,0x48,0x48,0x30,//b
0x38,0x44,0x44,0x44,0x44,//c
0x30,0x48,0x48,0x48,0x7f,//d
0x38,0x54,0x54,0x54,0x58,//e
0x00,0x08,0x7e,0x09,0x02,//f
0x48,0x54,0x54,0x54,0x3c,//g
0x7f,0x08,0x08,0x08,0x70,//h
0x00,0x00,0x7a,0x00,0x00,//i
0x20,0x40,0x40,0x3d,0x00,//j
0x7f,0x20,0x28,0x44,0x00,//k
0x00,0x41,0x7f,0x40,0x00,//l
0x7c,0x04,0x38,0x04,0x7c,//m
0x7c,0x08,0x04,0x04,0x78,//n
0x38,0x44,0x44,0x44,0x38,//o
0x7c,0x14,0x14,0x14,0x08,//p
0x08,0x14,0x14,0x14,0x7c,//q
0x7c,0x08,0x04,0x04,0x08,//r
0x48,0x54,0x54,0x54,0x24,//s
0x04,0x04,0x3f,0x44,0x24,//t
0x3c,0x40,0x40,0x40,0x3c,//u
0x1c,0x20,0x40,0x20,0x1c,//v
0x3c,0x40,0x30,0x40,0x3c,//w
0x44,0x28,0x10,0x28,0x44,//x
0x04,0x48,0x30,0x08,0x04,//y
0x44,0x64,0x54,0x4c,0x44,//z
0x08,0x36,0x41,0x41,0x00,//{
0x00,0x00,0x77,0x00,0x00,//|
0x00,0x41,0x41,0x36,0x08,//}
0x04,0x02,0x02,0x02,0x01,//~
};
#endif

#endif


/**************************************************************************************************
**************************************************************************************************/



