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
/* ���˵������ԺͲ���"��װ"��һ��*/ 
typedef struct tagSysMenu 
{ 
  uint8 *text[8];                        /* �˵����ı�*/ 
  uint8 xPos[8];                         /* �˵���LCD�ϵ�x����*/ 
  uint16 yPos[8];                         /* �˵���LCD�ϵ�y����*/ 
  uint8  YMAX;                           /* �˵���LCD�ϵ�y����������*/ 
  float *a[3];                          /* �˵���LCD�ϵ����ĵ�ַ��ֱ�Ӹĵ�ַ�е���*/ 
  uint16_t  *d;
  signed char *b;                    /* �˵���LCD�ϵ����ĵ�ַ��ֱ�Ӹĵ�ַ�е���*/ 
  int  *c[3];                    /* �˵���LCD�ϵ����ĵ�ַ��ֱ�Ӹĵ�ַ�е���*/ 
  void (*onAddFun)();                   /* �ڸò˵��ϰ���up���Ĵ�����ָ��*/
  void (*onSubFun)();                   /* �ڸò˵��ϰ���down���Ĵ�����ָ��*/
  void (*onLEFTFun)();                  /* �ڸò˵��ϰ���ok���Ĵ�����ָ��*/ 
  void (*onRightFun)();                 /* �ڸò˵��ϰ���cancel���Ĵ�����ָ��*/ 
  void (*OLED_SHOW_MENU)();             /* �ڸò˵���ʼ��*/ 
}SysMenu, *LPSysMenu; 

/* ����˵���ʼ��*/
static SysMenu menu[3] = 
{ {    {"EPerC:","Error:","S_S_G:", "DJPWM:","StableN:","MidEx:","Excur:","ROAD:",},  /* �˵����ı�*/ 
         {75,75,75,75,75,75,75,75},                                /* �˵���LCD�ϵ�x����*/ 
       {1,20,40,60,80,100,120,140},   //  {130,155,180,205,230,255,280,305},                /* �˵���LCD�  yϵ�y����*/ 
          7,                                               /* �˵���LCD�ϵ�x,y����������*/ 
        {&EPerCount,&Error ,&Speed_Set_Goal},                               /* �˵���LCD�ϵ����ĵ�ַ*/ 
       &Error1,
        {&RoadType}, 
         {&StableNumbers,&MidLineExcursion,&Excursion},
         menuAdd, menuSub,                        /* �Ӽ�����*/ 
         menuLEFT, menuRIGHT ,                    /* ��λ����*/ 
         OLED_SHOW_SMENU} ,                        /* �˵����*/ 
  
  
  
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
 // LCD_Fill(0x00);  //��ʼ����
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

/* ����ADD��*/ 
void onAddKey(void) 
{ //gpio_turn(PTB21) ;
  menu[currentFocusMenu].onAddFun(); 
} 
/* ����SUB��*/ 
void onSubKey(void) 
{ 
  menu[currentFocusMenu].onSubFun(); 
} 

///*******************************������λ��*********************************/

///*******************************�������*********************************/
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

 
/*******************************�����Ҽ�*********************************/
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


/*******************************�Ӽ���*********************************/

void menuAdd(void)	/*�Ӽ�*/
{
   gpio_turn(PTA19) ;
   *menu[currentFocusMenu].a[key_flag]+=0.005;
   sprintf((int8*)S01,"%d",(uint32)(*menu[currentFocusMenu].a[key_flag]*1000));   
   LCD_P8x16Str(menu[currentFocusMenu].xPos[ key_flag],menu[currentFocusMenu].yPos[ key_flag],S01 );
  //  sprintf((int8*)S01,"%d",(uint32)( Speed_PID.KP*1000));//�����Ƿ�ɹ���дԴ��ַ�е���
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



    uint8  n = 0  ;    //���ź�
   
   for(n=15;n<20;n++)  //�ж��ĸ����ŵ��жϷ�����i��ʾ�ĸ����ŷ������ж�
   {
	  if(PORTC_ISFR & (1 << n))break;
   }
   PORTC_ISFR  = (1 << n);        //д1���жϱ�־λ
    switch(n)
      { /*���������ŷ����ж���������,��Ҫ�ĸ����ž�д���Ǹ�����*/
	 
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
    // port_init(PTC15, ALT1 | IRQ_FALLING | PULLUP );          //��ʼ�� PTD7 �ܽţ����ù���ΪGPIO ���½��ش����жϣ���������
     //port_init(PTC16, ALT1 | IRQ_FALLING | PULLUP );          //��ʼ�� PTD7 �ܽţ����ù���ΪGPIO ���½��ش����жϣ���������
    // port_init(PTC18, ALT1 | IRQ_FALLING | PULLUP );          //��ʼ�� PTD7 �ܽţ����ù���ΪGPIO ���½��ش����жϣ���������
    // port_init(PTC17, ALT1 | IRQ_FALLING | PULLUP );          //��ʼ�� PTD7 �ܽţ����ù���ΪGPIO ���½��ش����жϣ���������
    // port_init(PTC19, ALT1 | IRQ_FALLING | PULLUP );          //��ʼ�� PTD7 �ܽţ����ù���ΪGPIO ���½��ش����жϣ���������

  //  set_vector_handler(PORTC_VECTORn ,PORTC_IRQHandler);    //����PORTE���жϸ�λ����Ϊ PORTE_IRQHandler
  //  enable_irq (PORTC_IRQn);   //ʹ��PORTE�ж� 
    
   currentFocusMenu=0;
    OLED_SHOW_SMENU();
    //   Speed_PID.KP =11.1111;
     //  Speed_PID.KI = 12.1111;
     //  Speed_PID.KD =13.1111;
  
   
}








