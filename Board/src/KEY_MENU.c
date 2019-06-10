#include "common.h"
#include "MK60_port.h"
#include "MK60_gpio.h"
#include"KEY_MENU.h"
#include"OLED.h"
#include "MK60_uart.h"

//extern PID_SPEED    Speed_PID;
float Speed_Set_Goal = 70.f;
int MidLineExcursion = 0;


uint8 S01[9]; 
//uint8_t AA[4]={0,0,0,0};
int currentFocusMenu = 0-1;
uint8 key_flag=1;
/* 将菜单的属性和操作"封装"在一起*/ 
typedef struct tagSysMenu 
{ 
  uint8 *text[8];                        /* 菜单的文本*/ 
  uint8 xPos[8];                         /* 菜单在LCD上的x坐标*/ 
  uint16 yPos[8];                         /* 菜单在LCD上的y坐标*/ 
  uint8  YMAX;                           /* 菜单在LCD上的y坐标最大个数*/ 
  float *a[3];                          /* 菜单在LCD上的数的地址，直接改地址中的数*/ 
  uint16_t  *d;
  signed char *b;                    /* 菜单在LCD上的数的地址，直接改地址中的数*/ 
  int  *c[3];                    /* 菜单在LCD上的数的地址，直接改地址中的数*/ 
  void (*onAddFun)();                   /* 在该菜单上按下up键的处理函数指针*/
  void (*onSubFun)();                   /* 在该菜单上按下down键的处理函数指针*/
  void (*onLEFTFun)();                  /* 在该菜单上按下ok键的处理函数指针*/ 
  void (*onRightFun)();                 /* 在该菜单上按下cancel键的处理函数指针*/ 
  void (*OLED_SHOW_MENU)();             /* 在该菜单初始化*/ 
}SysMenu, *LPSysMenu; 

/* 定义菜单初始化*/
static SysMenu menu[3] = 
{ {    {"EPerC:","Error:","S_S_G:", "DJPWM:","StableN:","MidEx:","Excur:","ROAD:",},  /* 菜单的文本*/ 
         {75,75,75,75,75,75,75,75},                                /* 菜单在LCD上的x坐标*/ 
       {1,20,40,60,80,100,120,140},   //  {130,155,180,205,230,255,280,305},                /* 菜单在LCD�  y系膟坐标*/ 
          7,                                               /* 菜单在LCD上的x,y坐标最大个数*/ 
        {&EPerCount,&Error ,&Speed_Set_Goal},                               /* 菜单在LCD上的数的地址*/ 
       &Error1,
        {&RoadType}, 
         {&StableNumbers,&MidLineExcursion,&Excursion},
         menuAdd, menuSub,                        /* 加减函数*/ 
         menuLEFT, menuRIGHT ,                    /* 移位函数*/ 
         OLED_SHOW_SMENU} ,                        /* 菜单框架*/ 
  
  
  
};

static int fabss(int n)
{
    if(n<0) return (-1)*n;
    else return n;
}
void Funcation_key(void)
{
    currentFocusMenu++;
    if(currentFocusMenu>5)
    currentFocusMenu=0;    
    menu[ currentFocusMenu].OLED_SHOW_MENU() ;    
}

void  OLED_SHOW_SMENU(void)
{
  uint8 i=0;
  gpio_turn(PTA19);
 // LCD_Fill(0x00);  //初始清屏
  for(i=0;i<=menu[currentFocusMenu].YMAX;i++)
  {
   
   Gui_DrawFont_GBK24(5,menu[currentFocusMenu].yPos[i],BLUE,GRAY0,menu[currentFocusMenu].text[i]);
  if(currentFocusMenu==0&&i<=3)
  {
      if(i!=3)
      {
     sprintf((int8*)S01,"%d",(uint32)(*menu[currentFocusMenu].a[i]*1000));   
     Gui_DrawFont_GBK24(menu[currentFocusMenu].xPos[i],menu[currentFocusMenu].yPos[i],RED,GRAY0,S01);
      }
      else
        {
     sprintf((int8*)S01,"%d",(*menu[currentFocusMenu].d));   
     Gui_DrawFont_GBK24(menu[currentFocusMenu].xPos[i],menu[currentFocusMenu].yPos[i],RED,GRAY0,S01);
      } 
  }else
  if(currentFocusMenu==0&&i<=6)
    {
         sprintf((int8*)S01,"%d",(fabss(*menu[currentFocusMenu].c[i-4])));   
         Gui_DrawFont_GBK24(menu[currentFocusMenu].xPos[i],menu[currentFocusMenu].yPos[i],RED,GRAY0,S01);
 
    }
  else
  if(currentFocusMenu==0&&i==7)
    {
         sprintf((int8*)S01,"%d",(*menu[currentFocusMenu].b));   
         Gui_DrawFont_GBK24(menu[currentFocusMenu].xPos[i],menu[currentFocusMenu].yPos[i],RED,GRAY0,S01);
 
    }/**//**/
  }
 
}

/* 按下ADD键*/ 
void onAddKey(void) 
{ //gpio_turn(PTB21) ;
  menu[currentFocusMenu].onAddFun(); 
} 
/* 按下SUB键*/ 
void onSubKey(void) 
{ 
  menu[currentFocusMenu].onSubFun(); 
} 

///*******************************上下移位建*********************************/

///*******************************按下左键*********************************/
void onLEFTKey(void) 
{
  key_flag--;
  if( key_flag<1|| key_flag>menu[currentFocusMenu].YMAX)
  key_flag=menu[currentFocusMenu].YMAX;
  
  menu[currentFocusMenu].onLEFTFun(); 
  
} 

void menuLEFT(void)
{
    gpio_turn(PTA19) ;
    LCD_P8x16Str(120,menu[currentFocusMenu].yPos[ key_flag],"1");
}

 
/*******************************按下右键*********************************/
void onRightKey(void) 
{ 
  key_flag++;
  if(key_flag<1|| key_flag>menu[currentFocusMenu].YMAX)
  key_flag=1;
  
  menu[currentFocusMenu].onRightFun(); 
  
} 

void menuRIGHT(void)
{
    gpio_turn(PTD15) ;
    LCD_P8x16Str(120,menu[currentFocusMenu].yPos[ key_flag],"0");
   
}


/*******************************加减建*********************************/

void menuAdd(void)	/*加键*/
{
   gpio_turn(PTA19) ;
   *menu[currentFocusMenu].a[key_flag]+=0.005;
   sprintf((int8*)S01,"%d",(uint32)(*menu[currentFocusMenu].a[key_flag]*1000));   
   LCD_P8x16Str(menu[currentFocusMenu].xPos[ key_flag],menu[currentFocusMenu].yPos[ key_flag],S01 );
  //  sprintf((int8*)S01,"%d",(uint32)( Speed_PID.KP*1000));//检验是否成功改写源地址中的数
   //uart_putstr(UART1,S01);
}
void menuSub(void)
{
   gpio_turn(PTD15) ;
   *menu[currentFocusMenu].a[key_flag]-=0.005;
   sprintf((int8*)S01,"%d",(uint32)( *menu[currentFocusMenu].a[key_flag]*1000));
   LCD_P8x16Str(menu[currentFocusMenu].xPos[ key_flag],menu[currentFocusMenu].yPos[ key_flag],S01 ); 
   //sprintf((int8*)S01,"%d",(uint32)( Speed_PID.KP*1000));
   //uart_putstr(UART1,S01);
}




void PORTC_IRQHandler(void)
{



    uint8  n = 0  ;    //引脚号
   
   for(n=15;n<20;n++)  //判断哪个引脚的中断发生，i表示哪个引脚发生的中断
   {
	  if(PORTC_ISFR & (1 << n))break;
   }
   PORTC_ISFR  = (1 << n);        //写1清中断标志位
    switch(n)
      { /*这里是引脚发生中断所做的事,需要哪个引脚就写上那个引脚*/
	 
	  case 15:     onSubKey()  ;   break;  
	  case 16:     Funcation_key()  ;    break;  
	  case 17:     onLEFTKey()  ;     break;  
	  case 18:     onRightKey()   ;      break;
	  case 19:     onAddKey()  ;     break;  
	  default : break;
	}
  
             
   
   

}


void key(void)
{   
    // port_init(PTC15, ALT1 | IRQ_FALLING | PULLUP );          //初始化 PTD7 管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻
     //port_init(PTC16, ALT1 | IRQ_FALLING | PULLUP );          //初始化 PTD7 管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻
    // port_init(PTC18, ALT1 | IRQ_FALLING | PULLUP );          //初始化 PTD7 管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻
    // port_init(PTC17, ALT1 | IRQ_FALLING | PULLUP );          //初始化 PTD7 管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻
    // port_init(PTC19, ALT1 | IRQ_FALLING | PULLUP );          //初始化 PTD7 管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻

  //  set_vector_handler(PORTC_VECTORn ,PORTC_IRQHandler);    //设置PORTE的中断复位函数为 PORTE_IRQHandler
  //  enable_irq (PORTC_IRQn);   //使能PORTE中断 
    
   currentFocusMenu=0;
    OLED_SHOW_SMENU();
    //   Speed_PID.KP =11.1111;
     //  Speed_PID.KI = 12.1111;
     //  Speed_PID.KD =13.1111;
  
   
}








