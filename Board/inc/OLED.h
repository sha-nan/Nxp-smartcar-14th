#ifndef _OLED_H
#define _OLED_H
#include"oled.h"

#define byte uint8
#define word uint16
#define GPIO_PIN_MASK      0x1Fu    //0x1f=31,����λ��Ϊ0--31��Ч
#define GPIO_PIN(x)        (((1)<<(x & GPIO_PIN_MASK)))  //�ѵ�ǰλ��1

#define OLED_DC   PTA15_OUT //B17 
#define OLED_RST  PTA16_OUT //B18
#define OLED_D1   PTA13_OUT //B19 
#define OLED_D0   PTA14_OUT //B20

#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����

 extern byte longqiu96x64[768];
void OLED_Init(void);
 void LCD_CLS(void);
 void LCD_Set_Pos(byte x, byte y);
 void LCD_WrDat(byte data);
 void LCD_P6x8Str(byte x,byte y,byte ch[]);
 void LCD_single_P6x8Str(byte x,byte y,byte ch[]);
 void LCD_P8x16Str(byte x,byte y,byte ch[]);
 void LCD_single_P8x16Str(byte x,byte y,byte ch[]);
 void LCD_P14x16Str(byte x,byte y,byte ch[]);
 void LCD_Print(byte x, byte y, byte ch[]);
 void LCD_PutPixel(byte x,byte y);
 void LCD_Rectangle(byte x1,byte y1,byte x2,byte y2,byte gif);
 void Draw_LQLogo(void);
 void Draw_LibLogo(void);
 void Draw_BMP(byte x0,byte y0,byte x1,byte y1,byte bmp[]); 
 void LCD_Fill(byte dat);
 extern void LCD_Refresh_Gram(void);
 void LCD_WR_Byte(uint8 dat,uint8 cmd);
 extern void LCD_DrawPoint(uint8 x,uint8 y,uint8 t);
 
 

#endif

