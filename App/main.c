#include "common.h"
#include "Public.h"
#include "MK60_PIT.h"
#include "MK60_uart.h"
#include "stdlib.h"

/*�������*/
int Zhangai_Flag;//ȫ�ֱ������ϰ�����־
int RightWheel_Count,LeftWheel_Count;
extern uint8 Vol;//��ص�ѹ
extern uint16 sensor1,sensor2,sensor3,sensor4;
extern uint16 sensor5;//,sensor6,sensor7,sensor8;//�м���
extern float turn_error,pre_turn_error;
extern uint8 Run_Flag;
uint8 error_flag=1;
extern int16 turn_out;//������
int c=0;
/*��������*/

extern void PORTB_IRQHandler();//PORTB�жϷ�����
void zhangaichuli();//�ϰ�������
/*****************************************************************************
                         ����ʹ��
******************************************************************************/
#if 0

/*!
 *  @brief      main����
 *  @since      v5.2
 *  @note       ���Բ�ѯ���ն���ַ�������
 */
void main()
{   
  gpio_init(PTB20,GPO,0);//������
  while(1)
  {
    PTB20_OUT=0;
  }
}
#endif

/*****************************************************************************
                         ���������ĵ�У�
******************************************************************************/
#if 1
int RightWheel_Count,LeftWheel_Count;
extern int Real_Speed,Out_Speed,Speed_Error,k;

void main()
{
    DisableInterrupts;//��ֹȫ���ж�
    wildWolf_init();//��ʼ��
    EnableInterrupts;//ʹ��ȫ���ж�

    while(1)
    {
//      AD_Collect();//��вɼ�
//      AD_Date_analyse();
//      Run_Control();
//      FTM_PWM_Duty(FTM1, FTM_CH0, turn_out_cal());

//      if(0==Zhangai_Flag)
//        {
//           FTM_PWM_Duty(FTM1, FTM_CH0, turn_out_cal());           
//        }
//      else
//       {
//         zhangaichuli();
//       }
//          
//           uint8 S1[8],S2[8],S3[8],S4[8];
//           sprintf((uint8*)S1," S1:%4d S2:%4d",sensor1,sensor2);
//            LCD_single_P8x16Str(0,0,S1);
//           sprintf((uint8*)S2," S3:%4d S4:%4d",sensor3,sensor4);
//            LCD_single_P8x16Str(0,2,S2);
//           sprintf((uint8*)S3," S5:%4d Vol:%3.1f",sensor5,Vol);
//            LCD_single_P8x16Str(0,4,S3);
//           sprintf((uint8*)S4," Error:%4d ",(sensor1-sensor4));
//            LCD_single_P8x16Str(0,6,S4);      

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

      
           uint8 L1[8]={0},L2[8]={0},Re[8]={0};
           sprintf((char*)L1,"L:%5d",LeftWheel_Count);
            LCD_single_P8x16Str(0,0,L1);
           sprintf((char*)L2,"R:%5d",RightWheel_Count);
            LCD_single_P8x16Str(0,2,L2);
           sprintf((char*)Re,"Re:%5d",Real_Speed);
            LCD_single_P8x16Str(0,4,Re);

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

   #if 0//������
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

void zhangaichuli()//·�ϴ���
{
    if(1==Zhangai_Flag&&1 == c)
    {
     FTM_PWM_Duty(FTM1, FTM_CH0,1010);
     pit_delay_ms(PIT2,500);//��ʱ1000ms
//     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
//     pit_delay_ms(PIT2,200);//��ʱ1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1160);
     pit_delay_ms(PIT2,700);//��ʱ1000ms
     FTM_PWM_Duty(FTM1, FTM_CH0,1082);
     pit_delay_ms(PIT2,600);//��ʱ1000ms    
     Zhangai_Flag=0;//�����־λ
    }
    else
    {
      FTM_PWM_Duty(FTM1, FTM_CH0, turn_out_cal());           
    }      
    
    

}
/*!
 *  @brief      PORTC�жϷ�����
 *  @since      v5.0
 */
void PORTB_IRQHandler()
{
    PORTB_ISFR  = ~0; //���жϱ�־λ
    uint8  n = 0;    //���ź�
    n = 21;   //PTB21�����жϣ��͵�ƽ����
    if(PORTB_ISFR & (1 << n))
    {
       /*     �û�����       */
      c=c++;
      Zhangai_Flag=1;
      zhangaichuli();
    }
}

/*
 *  @brief      FTM�жϷ�����(����������)
 *  @since      v5.0
 */
void FTM2_INPUT_IRQHandler(void)
{
     uint8 s = FTM2_STATUS;          //��ȡ��׽�ͱȽ�״̬
     uint8 CHn;
     FTM2_STATUS = 0x00;             //���жϱ�־

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