#include "common.h"
#include "Control.h"
#include "Public.h"
#include  "MK60_FTM.h"
#include  "MK60_PIT.h"     //�����жϼ�ʱ��
#include "MK60_adc.h"

int16 Turn_Out;
int16 Error;
int16 Car_Sudu;
int16 zhuangshu,zuo_zhuanshu,you_zhuanshu,zuo_error,you_error;//������ת������ֵ
float Turn_KP,Turn_KD;
extern uint16 sensor1,sensor2;
extern float turn_error,turn_lasterror;//ת�����

/*��������*/
extern void get_speed();
extern void Speed_Control();
extern int16 Speed_PID(int16 Goal,int16 Input);
/********************��вɼ��ж�*******************************/
void PIT0_IRQHandler(void)
{
    PIT_TFLG(PIT0)&=~(PIT_TCTRL_TEN_MASK|PIT_TCTRL_TIE_MASK);//�ض�ʱ�����ж�
    AD_Date_analyse();
    Run_Control();
    FTM_PWM_Duty(FTM1, FTM_CH0, turn_out_cal());
    PIT_Flag_Clear(PIT0); //���жϱ�־λ
    PIT_TFLG(PIT0)|=(PIT_TCTRL_TEN_MASK|PIT_TCTRL_TIE_MASK);//����ʱ�����жϣ����ο��ֲ�976ҳ
}
/******************************�ٶȼ����ж�*********************************/
int Speed_Set,Speed_Int,Real_Speed,Speed_Control_Out;
int Left_Speed,Right_Speed;
uint8 time_counter=0;//ʱ���־λ
void PIT1_IRQHandler(void)//�ٶȼ����жϷ�����
{
    PIT_TFLG(PIT1)&=~(PIT_TCTRL_TEN_MASK|PIT_TCTRL_TIE_MASK);//�ض�ʱ�����ж�
    time_counter++;//ʱ���־λ
    if(50==time_counter)    //50ms��һ���ٶ�
    {
      time_counter=0;  //��־λ����
      get_speed();     //���ٺ���
      Speed_Control(); //�ٶȿ���
      Left_Speed=0;
      Right_Speed=0;      
    }
    PIT_Flag_Clear(PIT1);   //���жϱ�־λ
    PIT_TFLG(PIT1)|=(PIT_TCTRL_TEN_MASK|PIT_TCTRL_TIE_MASK);//����ʱ�����ж�
}

/********************�ٶȿ���********************************/
float Speed_P,Speed_I,Speed_D;
extern int RightWheel_Count,LeftWheel_Count;
void get_speed()//�ٶȼ���
{

      Real_Speed = (LeftWheel_Count+RightWheel_Count)/2; //�����ٶ� 
      Left_Speed = LeftWheel_Count;//�����ٶ�
      Right_Speed = RightWheel_Count;//�ҵ���ٶ�
      LeftWheel_Count=0;
      RightWheel_Count=0;//����������ݣ�������ÿ10ms����һ��
//      printf("Left_Speed=%4d\n",Left_Speed);
//     uint8 L1[8]={0},L2[8]={0},Re[8]={0};
//      sprintf((char*)L1,"L:%5d",LeftWheel_Count);
//     LCD_single_P8x16Str(0,0,L1);
//      sprintf((char*)L2,"R:%5d",RightWheel_Count);
//     LCD_single_P8x16Str(0,2,L2);
//      sprintf((char*)Re,"Re:%5d",Real_Speed);
//     LCD_single_P8x16Str(0,4,Re);
}

extern float e,ec;
void Turn_PD(float Input)
{ 
  float error,lasterror;
  lasterror = error;
  error = Input * Turn_KP;             
  Turn_Out = error + (error-lasterror)*Turn_KD; 
}

extern uint8 Speed_Flag;
void Speed_Control()
{
//    if(1==Speed_Flag)
//    {
      Speed_P=0.0;
      Speed_I=0.1;
      Speed_D=0.0;
      Car_Sudu=240;
      Turn_KP=2.0;
      Turn_KD=0.8;      
//    }
//   else if(2==Speed_Flag)
//    {
//      Speed_P=0.0;
//      Speed_I=0.1;
//      Speed_D=0.0;
//      Car_Sudu=200;
//      Turn_KP=4.0;
//      Turn_KD=0.8;      
//    }
//   else if(3==Speed_Flag)
//    {
//      Speed_P=0.0;
//      Speed_I=0.1;
//      Speed_D=0.0;
//      Car_Sudu=200;
//      Turn_KP=4.0;
//      Turn_KD=0.8;      
//    }
//    else if(4==Speed_Flag)
//    {
//      Speed_P=0.0;
//      Speed_I=0.1;
//      Speed_D=0.0;
//      Car_Sudu=200;
//      Turn_KP=4.0;
//      Turn_KD=0.8;      
//    }
//   else if(5==Speed_Flag)
//    {
//      Speed_P=0.0;
//      Speed_I=0.1;
//      Speed_D=0.0;
//      Car_Sudu=200;
//      Turn_KP=4.0;
//      Turn_KD=0.8;      
//    }
//   else if(6==Speed_Flag)
//    {
//      Speed_P=0.0;
//      Speed_I=0.1;
//      Speed_D=0.0;
//      Car_Sudu=170;
//      Turn_KP=4.0;
//      Turn_KD=0.8;      
//    }  
//   else if(7==Speed_Flag)
//    {
//      Speed_P=0.0;
//      Speed_I=0.1;
//      Speed_D=0.0;
//      Car_Sudu=160;
//      Turn_KP=4.0;
//      Turn_KD=0.8;      
//    }  
//   else if(8==Speed_Flag)
//    {
//      Speed_P=0.0;
//      Speed_I=0.1;
//      Speed_D=0.0;
//      Car_Sudu=150;
//      Turn_KP=4.0;
//      Turn_KD=0.8;      
//    }  
//   else if(9==Speed_Flag)
//    {
//      Speed_P=0.0;
//      Speed_I=0.1;
//      Speed_D=0.0;
//      Car_Sudu=140;
//      Turn_KP=4.0;
//      Turn_KD=0.8;      
//    }        
//    else if(10==Speed_Flag)
//    {
//      Speed_P=0.0;
//      Speed_I=0.1;
//      Speed_D=0.0;
//      Car_Sudu=150;
//      Turn_KP=4.0;
//      Turn_KD=0.8;      
//    }
//   else if(11==Speed_Flag)
//    {
//      Speed_P=0.0;
//      Speed_I=0.1;
//      Speed_D=0.0;
//      Car_Sudu=120;
//      Turn_KP=14.0;
//      Turn_KD=0.8;      
//    }
//   else
//    {
//      Speed_P=0.0;
//      Speed_I=0.1;
//      Speed_D=0.0;
//      Car_Sudu=200;
//      Turn_KP=15.0;
//      Turn_KD=0.8;
//    }      
//    Speed_Flag=0;
    
    Turn_PD(e);   //����ת��ƫ��
    zuo_zhuanshu = Car_Sudu-Turn_Out;//  7-9
    you_zhuanshu = Car_Sudu+Turn_Out;//  7+9
    
    if(!Run_Flag)
      {
        Motor_output1(0);
        Motor_output2(0);
      }
    else
      {
        Motor_output1(Speed_PID(zuo_zhuanshu,Left_Speed));
        Motor_output2(Speed_PID(you_zhuanshu,Right_Speed)); 
      }
}
