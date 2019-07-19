#include "common.h"
#include "Public.h"
#include "MK60_PIT.h"
#include "MK60_uart.h"
#include "stdlib.h"

/*定义变量*/
int Zhangai_Flag=0;//全局变量，障碍检测标志
uint8 PoDao_Flag=0;//坡道检测标志 
int RightWheel_Count,LeftWheel_Count;
extern uint16 s1,s2,s3,s4;
extern uint16 sensor1,sensor2,sensor3,sensor4;
extern uint16 sensor5;//,sensor6,sensor7,sensor8;//中间电感
extern float turn_error,pre_turn_error;
extern uint8 Run_Flag;
uint8 error_flag=1;
extern int16 turn_out;//舵机输出
int k=0;//障碍进一次标志
/*函数声明*/

extern void PORTB_IRQHandler();//PORTB中断服务函数
void zhangaichuli();//障碍处理函数
/*****************************************************************************
                         测试使用
******************************************************************************/
#if 0

/*!
 *  @brief      main函数
 *  @since      v5.2
 *  @note       测试查询接收多个字符串函数
 */
void main()
{   

}
#endif

/*****************************************************************************
                         主函数（四电感）(无环，有障碍)
******************************************************************************/
#if 1
int RightWheel_Count,LeftWheel_Count;
extern int Real_Speed,Out_Speed,Speed_Error,k;
extern int Speed_Flag;
void main()
{
    wildWolf_init();//初始化
    while(1)
    {        
           uint8 S1[8],S2[8],S3[8],S4[8];
           sprintf((uint8*)S1," S1:%4d S2:%4d",sensor1,sensor2);
            LCD_single_P8x16Str(0,0,S1);
           sprintf((uint8*)S2," S3:%4d S4:%4d",sensor3,sensor4);
            LCD_single_P8x16Str(0,2,S2);
           sprintf((uint8*)S4," Error:%4d ",turn_error);
            LCD_single_P8x16Str(0,6,S4);      

        printf("s1=%4d\n ",s1);
        printf("s2=%4d\n ",s2);
        printf("s3=%4d\n ",s3);
        printf("s4=%4d\n ",s4);            
      
//           uint8 L1[8]={0},L2[8]={0},Re[8]={0};
//           sprintf((char*)L1,"L:%5d",LeftWheel_Count);
//            LCD_single_P8x16Str(0,0,L1);
//           sprintf((char*)L2,"R:%5d",RightWheel_Count);
//            LCD_single_P8x16Str(0,2,L2);
//           sprintf((char*)Re,"Re:%5d",Real_Speed);
//            LCD_single_P8x16Str(0,4,Re);

    }
}

void zhangaichuli()//路障处理
{  
/*260速度*/
  if(1==Zhangai_Flag&&1==BOMA)//260速度
    {
     FTM_PWM_Duty(FTM1, FTM_CH0,1010);
     pit_delay_ms(PIT2,304);
//     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
//     pit_delay_ms(PIT2,200);
     FTM_PWM_Duty(FTM1, FTM_CH0,1160);
     pit_delay_ms(PIT2,480);
     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
     pit_delay_ms(PIT2,480);    
     Zhangai_Flag=0;//清除标志位
    }
/*270速度*/    
  else if(1==Zhangai_Flag&&2==BOMA)//270速度
    {
     FTM_PWM_Duty(FTM1, FTM_CH0,1010);
     pit_delay_ms(PIT2,292);
//     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
//     pit_delay_ms(PIT2,200);
     FTM_PWM_Duty(FTM1, FTM_CH0,1160);
     pit_delay_ms(PIT2,462);
     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
     pit_delay_ms(PIT2,462);   
     Zhangai_Flag=0;//清除标志位
    }
/*280速度*/
  else if(1==Zhangai_Flag&&4==BOMA)
    {
     FTM_PWM_Duty(FTM1, FTM_CH0,1010);
     pit_delay_ms(PIT2,282);
//     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
//     pit_delay_ms(PIT2,200);
     FTM_PWM_Duty(FTM1, FTM_CH0,1160);
     pit_delay_ms(PIT2,446);
     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
     pit_delay_ms(PIT2,446);   
     Zhangai_Flag=0;//清除标志位
    }
/*250速度*/  
  else if(1==Zhangai_Flag)//250速度
    {
     FTM_PWM_Duty(FTM1, FTM_CH0,1010);//舵机向右打角，右打死1005
     pit_delay_ms(PIT2,316);
//     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
//     pit_delay_ms(PIT2,200);
     FTM_PWM_Duty(FTM1, FTM_CH0,1163);//舵机向左打角，左打死1163
     pit_delay_ms(PIT2,500);
     FTM_PWM_Duty(FTM1, FTM_CH0,1063);//舵机中值1082，舵机稍向右打角
     pit_delay_ms(PIT2,500);    
     Zhangai_Flag=0;//清除标志位
    }    
  else
    {
      FTM_PWM_Duty(FTM1, FTM_CH0, turn_out_cal());           
    }              
}
/*!
 *  @brief      PORTC中断服务函数
 *  @since      v5.0
 */
void PORTB_IRQHandler()
{
   PORTB_ISFR  = ~0; //清中断标志位
   uint8  n = 0;    //引脚号
   n = 21;   //PTB21触发中断，低电平触发
   if(PORTB_ISFR & (1 << n))
    {
       /*     用户任务       */
/*只识别障碍，坡道不识别*/
#if 1
      k=k++;
      if(1==k)
       {
         Zhangai_Flag=1;
         zhangaichuli();
       }
      else
        FTM_PWM_Duty(FTM1, FTM_CH0, turn_out_cal());
#endif   
/*先障碍，后坡*/
#if 0     
      k=k++;
      if(1==k)
       {
         Zhangai_Flag=1;
         zhangaichuli();
       }
      else if(2==k)
       {
         PoDao_Flag=1;
         PTC5_OUT=1;
       }
      else
        FTM_PWM_Duty(FTM1, FTM_CH0, turn_out_cal());
#endif   
    }
}

/*
 *  @brief      FTM中断服务函数(编码器计数)
 *  @since      v5.0
 */
void FTM2_INPUT_IRQHandler(void)
{
     uint8 s = FTM2_STATUS;          //读取捕捉和比较状态
     uint8 CHn;
     FTM2_STATUS = 0x00;             //清中断标志

     CHn = 0;
     if( s & (1 << CHn) )
      {
        if(gpio_get(PTB17)) LeftWheel_Count++;
        else  LeftWheel_Count--;
      } 
     
     CHn = 1;
     if( s & (1 << CHn) )
      {
        if(gpio_get(PTB16)) RightWheel_Count++;
        else  RightWheel_Count--;
      }
}

#endif