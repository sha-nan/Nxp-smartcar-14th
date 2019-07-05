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


void Motor_output1(int16 duty)//左轮
{
  //首先对duty进行限值  精度为10000  限值为10000
  if(duty>=4500)
  {
    duty=4500;
  }
  else  if(duty<=-4500)
  {
    duty=-4500;
  }
  //然后给电机输出
  if(duty>=0)
  {

  FTM_PWM_Duty(FTM0, FTM_CH0,duty);
  FTM_PWM_Duty(FTM0, FTM_CH1,0);
  }
  else if(duty<0)
  {
  duty=-duty;//对负数取反   FTM输出只能为正值
  FTM_PWM_Duty(FTM0, FTM_CH0,0);
  FTM_PWM_Duty(FTM0, FTM_CH1,duty);
  } 
}

void Motor_output2(int16 duty)//右轮
{
  //首先对duty进行限值  精度为10000  限值为10000
  if(duty>=4500)
  {
    duty=4500;
  }
  else  if(duty<=-4500)
  {
    duty=-4500;
  }
  //然后给电机输出
  if(duty>=0)
  {

  FTM_PWM_Duty(FTM0, FTM_CH2,duty);
  FTM_PWM_Duty(FTM0, FTM_CH3,0);
  }
  else if(duty<0)
  {
  duty=-duty;//对负数取反   FTM输出只能为正值
  FTM_PWM_Duty(FTM0, FTM_CH2,0);
  FTM_PWM_Duty(FTM0, FTM_CH3,duty);
  } 
}