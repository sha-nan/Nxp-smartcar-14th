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
/* ½«²Ëµ¥µÄÊôĞÔºÍ²Ù×÷"·â×°"ÔÚÒ»Æğ*/ 
typedef struct tagSysMenu 
{ 
  uint8 *text[8];                        /* ²Ëµ¥µÄÎÄ±¾*/ 
  uint8 xPos[8];                         /* ²Ëµ¥ÔÚLCDÉÏµÄx×ø±ê*/ 
  uint16 yPos[8];                         /* ²Ëµ¥ÔÚLCDÉÏµÄy×ø±ê*/ 
  uint8  YMAX;                           /* ²Ëµ¥ÔÚLCDÉÏµÄy×ø±ê×î´ó¸öÊı*/ 
  float *a[3];                          /* ²Ëµ¥ÔÚLCDÉÏµÄÊıµÄµØÖ·£¬Ö±½Ó¸ÄµØÖ·ÖĞµÄÊı*/ 
  uint16_t  *d;
  signed char *b;                    /* ²Ëµ¥ÔÚLCDÉÏµÄÊıµÄµØÖ·£¬Ö±½Ó¸ÄµØÖ·ÖĞµÄÊı*/ 
  int  *c[3];                    /* ²Ëµ¥ÔÚLCDÉÏµÄÊıµÄµØÖ·£¬Ö±½Ó¸ÄµØÖ·ÖĞµÄÊı*/ 
  void (*onAddFun)();                   /* ÔÚ¸Ã²Ëµ¥ÉÏ°´ÏÂup¼üµÄ´¦Àíº¯ÊıÖ¸Õë*/
  void (*onSubFun)();                   /* ÔÚ¸Ã²Ëµ¥ÉÏ°´ÏÂdown¼üµÄ´¦Àíº¯ÊıÖ¸Õë*/
  void (*onLEFTFun)();                  /* ÔÚ¸Ã²Ëµ¥ÉÏ°´ÏÂok¼üµÄ´¦Àíº¯ÊıÖ¸Õë*/ 
  void (*onRightFun)();                 /* ÔÚ¸Ã²Ëµ¥ÉÏ°´ÏÂcancel¼üµÄ´¦Àíº¯ÊıÖ¸Õë*/ 
  void (*OLED_SHOW_MENU)();             /* ÔÚ¸Ã²Ëµ¥³õÊ¼»¯*/ 
}SysMenu, *LPSysMenu; 

/* ¶¨Òå²Ëµ¥³õÊ¼»¯*/
static SysMenu menu[3] = 
{ {    {"EPerC:","Error:","S_S_G:", "DJPWM:","StableN:","MidEx:","Excur:","ROAD:",},  /* ²Ëµ¥µÄÎÄ±¾*/ 
         {75,75,75,75,75,75,75,75},                                /* ²Ëµ¥ÔÚLCDÉÏµÄx×ø±ê*/ 
       {1,20,40,60,80,100,120,140},   //  {130,155,180,205,230,255,280,305},                /* ²Ëµ¥ÔÚLCDÉ  yÏµÄy×ø±ê*/ 
          7,                                               /* ²Ëµ¥ÔÚLCDÉÏµÄx,y×ø±ê×î´ó¸öÊı*/ 
        {&EPerCount,&Error ,&Speed_Set_Goal},                               /* ²Ëµ¥ÔÚLCDÉÏµÄÊıµÄµØÖ·*/ 
       &Error1,
        {&RoadType}, 
         {&StableNumbers,&MidLineExcursion,&Excursion},
         menuAdd, menuSub,                        /* ¼Ó¼õº¯Êı*/ 
         menuLEFT, menuRIGHT ,                    /* ÒÆÎ»º¯Êı*/ 
         OLED_SHOW_SMENU} ,                        /* ²Ëµ¥¿ò¼Ü*/ 
  
  
  
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
 // LCD_Fill(0x00);  //³õÊ¼ÇåÆÁ
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

/* °´ÏÂADD¼ü*/ 
void onAddKey(void) 
{ //gpio_turn(PTB21) ;
  menu[currentFocusMenu].onAddFun(); 
} 
/* °´ÏÂSUB¼ü*/ 
void onSubKey(void) 
{ 
  menu[currentFocusMenu].onSubFun(); 
} 

///*******************************ÉÏÏÂÒÆÎ»½¨*********************************/

///*******************************°´ÏÂ×ó¼ü*********************************/
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

 
/*******************************°´ÏÂÓÒ¼ü*********************************/
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


/*******************************¼Ó¼õ½¨*********************************/

void menuAdd(void)	/*¼Ó¼ü*/
{
   gpio_turn(PTA19) ;
   *menu[currentFocusMenu].a[key_flag]+=0.005;
   sprintf((int8*)S01,"%d",(uint32)(*menu[currentFocusMenu].a[key_flag]*1000));   
   LCD_P8x16Str(menu[currentFocusMenu].xPos[ key_flag],menu[currentFocusMenu].yPos[ key_flag],S01 );
  //  sprintf((int8*)S01,"%d",(uint32)( Speed_PID.KP*1000));//¼ìÑéÊÇ·ñ³É¹¦¸ÄĞ´Ô´µØÖ·ÖĞµÄÊı
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



    uint8  n = 0  ;    //Òı½ÅºÅ
   
   for(n=15;n<20;n++)  //ÅĞ¶ÏÄÄ¸öÒı½ÅµÄÖĞ¶Ï·¢Éú£¬i±íÊ¾ÄÄ¸öÒı½Å·¢ÉúµÄÖĞ¶Ï
   {
	  if(PORTC_ISFR & (1 << n))break;
   }
   PORTC_ISFR  = (1 << n);        //Ğ´1ÇåÖĞ¶Ï±êÖ¾Î»
    switch(n)
      { /*ÕâÀïÊÇÒı½Å·¢ÉúÖĞ¶ÏËù×öµÄÊÂ,ĞèÒªÄÄ¸öÒı½Å¾ÍĞ´ÉÏÄÇ¸öÒı½Å*/
	 
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
    // port_init(PTC15, ALT1 | IRQ_FALLING | PULLUP );          //³õÊ¼»¯ PTD7 ¹Ü½Å£¬¸´ÓÃ¹¦ÄÜÎªGPIO £¬ÏÂ½µÑØ´¥·¢ÖĞ¶Ï£¬ÉÏÀ­µç×è
     //port_init(PTC16, ALT1 | IRQ_FALLING | PULLUP );          //³õÊ¼»¯ PTD7 ¹Ü½Å£¬¸´ÓÃ¹¦ÄÜÎªGPIO £¬ÏÂ½µÑØ´¥·¢ÖĞ¶Ï£¬ÉÏÀ­µç×è
    // port_init(PTC18, ALT1 | IRQ_FALLING | PULLUP );          //³õÊ¼»¯ PTD7 ¹Ü½Å£¬¸´ÓÃ¹¦ÄÜÎªGPIO £¬ÏÂ½µÑØ´¥·¢ÖĞ¶Ï£¬ÉÏÀ­µç×è
    // port_init(PTC17, ALT1 | IRQ_FALLING | PULLUP );          //³õÊ¼»¯ PTD7 ¹Ü½Å£¬¸´ÓÃ¹¦ÄÜÎªGPIO £¬ÏÂ½µÑØ´¥·¢ÖĞ¶Ï£¬ÉÏÀ­µç×è
    // port_init(PTC19, ALT1 | IRQ_FALLING | PULLUP );          //³õÊ¼»¯ PTD7 ¹Ü½Å£¬¸´ÓÃ¹¦ÄÜÎªGPIO £¬ÏÂ½µÑØ´¥·¢ÖĞ¶Ï£¬ÉÏÀ­µç×è

  //  set_vector_handler(PORTC_VECTORn ,PORTC_IRQHandler);    //ÉèÖÃPORTEµÄÖĞ¶Ï¸´Î»º¯ÊıÎª PORTE_IRQHandler
  //  enable_irq (PORTC_IRQn);   //Ê¹ÄÜPORTEÖĞ¶Ï 
    
   currentFocusMenu=0;
    OLED_SHOW_SMENU();
    //   Speed_PID.KP =11.1111;
     //  Speed_PID.KI = 12.1111;
     //  Speed_PID.KD =13.1111;
  
   
}








