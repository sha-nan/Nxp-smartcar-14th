#include "common.h"
#include "Public.h"
#include "MK60_PIT.h"
#include "MK60_uart.h"
#include "stdlib.h"

/*定义变量*/
int Zhangai_Flag;//全局变量，障碍检测标志
int RightWheel_Count,LeftWheel_Count;
extern uint8 Vol;//电池电压
extern uint16 sensor1,sensor2,sensor3,sensor4;
extern uint16 sensor5;//,sensor6,sensor7,sensor8;//中间电感
extern float turn_error,pre_turn_error;
extern uint8 Run_Flag;
uint8 error_flag=1;
extern int16 turn_out;//舵机输出
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
 // gpio_init(PTA19,GPO,0);//蜂鸣器
  while(1)
  {
    //PTA19_OUT=0;
   // pit_delay_ms(PIT3,2000);//延时10ms
        PTA19_OUT=1;
      // pit_delay_ms(PIT3,1000);//延时10ms


      
  }
}
#endif

/*****************************************************************************
                         主函数（四电感）
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
//      if(0==Zhangai_Flag)
//        {
//           FTM_PWM_Duty(FTM1, FTM_CH0, turn_out_cal());           
//        }
//      else
//       {
//         zhangaichuli();
//       }
//          
           uint8 S1[8],S2[8],S3[8],S4[8];
           sprintf((uint8*)S1," S1:%4d S2:%4d",sensor1,sensor2);
            LCD_single_P8x16Str(0,0,S1);
           sprintf((uint8*)S2," S3:%4d S4:%4d",sensor3,sensor4);
            LCD_single_P8x16Str(0,2,S2);
           sprintf((uint8*)S3," S5:%4d Vol:%3.1f",sensor5,Vol);
            LCD_single_P8x16Str(0,4,S3);
           sprintf((uint8*)S4," Error:%4d ",turn_error);
            LCD_single_P8x16Str(0,6,S4);      

//            printf("s1=%4d\n ",sensor1);
//            printf("terror:%4.4f\n",turn_error);



            
//            if(1==error_flag)
//           {
//             printf("Error=%4.2f\n ec=%4.2f\n",turn_error,turn_error - pre_turn_error);
//             error_flag=0;
//           } 
//           else
//           {             
//             printf("turn_out=%4d\n",turn_out);
//             error_flag=1;
//           }

      
//           uint8 L1[8]={0},L2[8]={0},Re[8]={0};
//           sprintf((char*)L1,"L:%5d",LeftWheel_Count);
//            LCD_single_P8x16Str(0,0,L1);
//           sprintf((char*)L2,"R:%5d",RightWheel_Count);
//            LCD_single_P8x16Str(0,2,L2);
//           sprintf((char*)Re,"Re:%5d",Real_Speed);
//            LCD_single_P8x16Str(0,4,Re);

//      if(!Run_Flag)
//      {
//        Motor_output1(0);
//        Motor_output2(0);
//      }
//      else
//      {
//        Motor_output1(1500);
//        Motor_output2(1500);
//      }

   #if 0//拨码盘
        if(1==BOMA)
        {
           uint8 S1[8],S2[8],S3[8],S4[8];
           sprintf((uint8*)S1," S1:%4d S2:%4d",sensor1,sensor2);
            LCD_single_P8x16Str(0,0,S1);
           sprintf((uint8*)S2," S3:%4d S4:%4d",sensor3,sensor4);
            LCD_single_P8x16Str(0,2,S2);
           sprintf((uint8*)S3," S5:%4d Vol:%3.1f",sensor5,Vol);
            LCD_single_P8x16Str(0,4,S3);
           sprintf((uint8*)S4," Error:%4d ",(sensor1-sensor4));
            LCD_single_P8x16Str(0,6,S4);
        }
        else if(2==BOMA)
        {
           uint8 L1[8]={0},L2[8]={0},Re[8]={0};
           sprintf((char*)L1,"L:%5d",LeftWheel_Count);
            LCD_single_P8x16Str(0,0,L1);
           sprintf((char*)L2,"R:%5d",RightWheel_Count);
            LCD_single_P8x16Str(0,2,L2);
           sprintf((char*)Re,"Re:%5d",Real_Speed);
            LCD_single_P8x16Str(0,4,Re);              
        }
        else if(4==BOMA)
        {

        }
        
  #endif

    }
}

void zhangaichuli()//路障处理
{
    if(1==Zhangai_Flag&&1==Speed_Flag)
    {
     FTM_PWM_Duty(FTM1, FTM_CH0,1010);
     pit_delay_ms(PIT2,480);//延时1000ms
//     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
//     pit_delay_ms(PIT2,200);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1160);
     pit_delay_ms(PIT2,650);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
     pit_delay_ms(PIT2,600);//延时1000ms    
     Zhangai_Flag=0;//清除标志位
    }
    else if(1==Zhangai_Flag&&2==Speed_Flag)
    {
     FTM_PWM_Duty(FTM1, FTM_CH0,1010);
     pit_delay_ms(PIT2,480);//延时1000ms
//     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
//     pit_delay_ms(PIT2,200);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1160);
     pit_delay_ms(PIT2,650);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
     pit_delay_ms(PIT2,600);//延时1000ms    
     Zhangai_Flag=0;//清除标志位
    }
    else if(1==Zhangai_Flag&&2==Speed_Flag)
    {
     FTM_PWM_Duty(FTM1, FTM_CH0,1010);
     pit_delay_ms(PIT2,480);//延时1000ms
//     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
//     pit_delay_ms(PIT2,200);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1160);
     pit_delay_ms(PIT2,650);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
     pit_delay_ms(PIT2,600);//延时1000ms    
     Zhangai_Flag=0;//清除标志位
    }
    else if(1==Zhangai_Flag&&4==Speed_Flag)
    {
     FTM_PWM_Duty(FTM1, FTM_CH0,1010);
     pit_delay_ms(PIT2,480);//延时1000ms
//     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
//     pit_delay_ms(PIT2,200);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1160);
     pit_delay_ms(PIT2,650);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
     pit_delay_ms(PIT2,600);//延时1000ms    
     Zhangai_Flag=0;//清除标志位
    }
    else if(1==Zhangai_Flag&&5==Speed_Flag)
    {
     FTM_PWM_Duty(FTM1, FTM_CH0,1010);
     pit_delay_ms(PIT2,480);//延时1000ms
//     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
//     pit_delay_ms(PIT2,200);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1160);
     pit_delay_ms(PIT2,650);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
     pit_delay_ms(PIT2,600);//延时1000ms    
     Zhangai_Flag=0;//清除标志位
    }
    else if(1==Zhangai_Flag&&6==Speed_Flag)
    {
     FTM_PWM_Duty(FTM1, FTM_CH0,1010);
     pit_delay_ms(PIT2,480);//延时1000ms
//     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
//     pit_delay_ms(PIT2,200);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1160);
     pit_delay_ms(PIT2,650);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
     pit_delay_ms(PIT2,600);//延时1000ms    
     Zhangai_Flag=0;//清除标志位
    }
    else if(1==Zhangai_Flag&&7==Speed_Flag)
    {
     FTM_PWM_Duty(FTM1, FTM_CH0,1010);
     pit_delay_ms(PIT2,480);//延时1000ms
//     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
//     pit_delay_ms(PIT2,200);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1160);
     pit_delay_ms(PIT2,650);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
     pit_delay_ms(PIT2,600);//延时1000ms    
     Zhangai_Flag=0;//清除标志位
    }
    else if(1==Zhangai_Flag&&8==Speed_Flag)
    {
     FTM_PWM_Duty(FTM1, FTM_CH0,1010);
     pit_delay_ms(PIT2,480);//延时1000ms
//     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
//     pit_delay_ms(PIT2,200);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1160);
     pit_delay_ms(PIT2,650);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
     pit_delay_ms(PIT2,600);//延时1000ms    
     Zhangai_Flag=0;//清除标志位
    }
    else if(1==Zhangai_Flag&&9==Speed_Flag)
    {
     FTM_PWM_Duty(FTM1, FTM_CH0,1010);
     pit_delay_ms(PIT2,480);//延时1000ms
//     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
//     pit_delay_ms(PIT2,200);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1160);
     pit_delay_ms(PIT2,650);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
     pit_delay_ms(PIT2,600);//延时1000ms    
     Zhangai_Flag=0;//清除标志位
    }   
    else if(1==Zhangai_Flag&&10==Speed_Flag)
    {
     FTM_PWM_Duty(FTM1, FTM_CH0,1010);
     pit_delay_ms(PIT2,480);//延时1000ms
//     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
//     pit_delay_ms(PIT2,200);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1160);
     pit_delay_ms(PIT2,650);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
     pit_delay_ms(PIT2,600);//延时1000ms    
     Zhangai_Flag=0;//清除标志位
    }
    else if(1==Zhangai_Flag&&11==Speed_Flag)
    {
     FTM_PWM_Duty(FTM1, FTM_CH0,1010);
     pit_delay_ms(PIT2,480);//延时1000ms
//     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
//     pit_delay_ms(PIT2,200);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1160);
     pit_delay_ms(PIT2,650);//延时1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
     pit_delay_ms(PIT2,600);//延时1000ms    
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
      Zhangai_Flag=1;
      zhangaichuli();
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