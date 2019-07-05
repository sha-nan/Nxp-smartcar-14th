#include "common.h"
#include "Public.h"
#include "Init.h"
#include  "OLED.h"
#include "MK60_uart.h"
#include "Boma.h"
#include "misc.h"
#include "MK60_adc.h"
#include "Motor.h"
#include  "MK60_PIT.h" 

void Motor_init(void)
{
     FTM_PWM_init(FTM0, FTM_CH0,13000, 0); 
     FTM_PWM_init(FTM0, FTM_CH1,13000, 0);
     FTM_PWM_init(FTM0, FTM_CH2,13000, 0); 
     FTM_PWM_init(FTM0, FTM_CH3,13000, 0);
}


void Motor_output1(int16 duty)//����
{
  //���ȶ�duty������ֵ  ����Ϊ10000  ��ֵΪ10000
  if(duty>=4500)
  {
    duty=4500;
  }
  else  if(duty<=-4500)
  {
    duty=-4500;
  }
  //Ȼ���������
  if(duty>=0)
  {

  FTM_PWM_Duty(FTM0, FTM_CH0,duty);
  FTM_PWM_Duty(FTM0, FTM_CH1,0);
  }
  else if(duty<0)
  {
  duty=-duty;//�Ը���ȡ��   FTM���ֻ��Ϊ��ֵ
  FTM_PWM_Duty(FTM0, FTM_CH0,0);
  FTM_PWM_Duty(FTM0, FTM_CH1,duty);
  } 
}

void Motor_output2(int16 duty)//����
{
  //���ȶ�duty������ֵ  ����Ϊ10000  ��ֵΪ10000
  if(duty>=4500)
  {
    duty=4500;
  }
  else  if(duty<=-4500)
  {
    duty=-4500;
  }
  //Ȼ���������
  if(duty>=0)
  {

  FTM_PWM_Duty(FTM0, FTM_CH2,duty);
  FTM_PWM_Duty(FTM0, FTM_CH3,0);
  }
  else if(duty<0)
  {
  duty=-duty;//�Ը���ȡ��   FTM���ֻ��Ϊ��ֵ
  FTM_PWM_Duty(FTM0, FTM_CH2,0);
  FTM_PWM_Duty(FTM0, FTM_CH3,duty);
  } 
}