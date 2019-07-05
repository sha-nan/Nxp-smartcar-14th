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
    AD_Date_analyse();//��вɼ������ݴ���
    StopCar();//ͣ�����   
    Run_Control();//����״̬
    PIT_Flag_Clear(PIT0); //���жϱ�־λ
    PIT_TFLG(PIT0)|=(PIT_TCTRL_TEN_MASK|PIT_TCTRL_TIE_MASK);//����ʱ�����жϣ����ο��ֲ�976ҳ
}
/******************************�ٶȼ����ж�*********************************/
int Real_Speed=0;
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
      Left_Speed = LeftWheel_Count;//�����ٶ�
      Right_Speed = RightWheel_Count;//�ҵ���ٶ�
      Real_Speed = (Left_Speed+Right_Speed)/2; //�����ٶ� 
      
//      if(0!=Real_Speed&&0!=Left_Speed)
//      printf("times:%4d\n Re=%4d\n" ,k,Real_Speed);
      
      LeftWheel_Count=0;
      RightWheel_Count=0;//����������ݣ�������ÿ50ms����һ��
      
      
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
    if(11==Speed_Flag)
    {
      Car_Sudu=250;//����Ŀ���ٶ�
      Speed_P=1.0;//�ٶ�Kp
      Speed_I=0.2;//�ٶ�Ki
      Speed_D=0.0;//�ٶ�Kd
      Turn_KP=1.0;//����Kp
      Turn_KD=0.0;//����Kd  
    }
   else if(10==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0; 
    }    
   else if(9==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0; 
    }
   else if(8==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;   
    }
    else if(7==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;      
    }
   else if(6==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;     
    }
   else if(5==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;    
    }  
   else if(4==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;     
    }  
   else if(3==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;    
    }  
   else if(2==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;       
    }        
    else if(1==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;     
    }
   else
    {
      Car_Sudu=250;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0; 
    }      
    Speed_Flag=0;
    
    Turn_PD(e);   //����ת��ƫ��
    zuo_zhuanshu = Car_Sudu-Turn_Out;//����
    you_zhuanshu = Car_Sudu+Turn_Out;
    
    if(!Run_Flag)
      {
        Motor_output1(0);
        Motor_output2(0);
        BEEP_OFF;//������        
      }
    else
      {
        Motor_output1(Speed_PID(zuo_zhuanshu,Left_Speed));
        Motor_output2(Speed_PID(you_zhuanshu,Right_Speed)); 
      }
}

