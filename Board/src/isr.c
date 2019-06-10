/******************** (C) COPYRIGHT 2011 蓝宙电子工作室 ********************
 * 文件名       ：isr.c
 * 描述         ：中断处理例程
 *
 * 实验平台     ：野火kinetis开发板
 * 库版本       ：
 * 嵌入系统     ：
 *
 * 作者         ：蓝宙电子工作室
 * 淘宝店       ：http://landzo.taobao.com/
**********************************************************************************/



/******************** (C) COPYRIGHT 2011 蓝宙电子工作室 ********************
 * 文件名       ：isr.c
 * 描述         ：中断处理例程
 *
 * 实验平台     ：野火kinetis开发板
 * 库版本       ：
 * 嵌入系统     ：
 *
 * 作者         ：蓝宙电子工作室
 * 淘宝店       ：http://landzo.taobao.com/
**********************************************************************************/



#include "common.h"
#include  "isr.h"


uint8 TIME0flag_5ms  = 0 ;
uint8 TIME0flag_10ms = 0 ;
uint8 TIME0flag_15ms = 0 ;
uint8 TIME0flag_20ms = 0 ;
uint8 TIME0flag_80ms = 0 ; 
uint8 TIME1flag_20ms = 0 ;
uint8 TIME1flag_1ms  = 0 ;
uint8  TimeCount = 0 ;



/*************************************************************************
*                             蓝宙嵌入式开发工作室
*
*  函数名称：PIT1_IRQHandler
*  功能说明：PIT1定时中断服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：2012-9-18    已测试
*  备    注：
*************************************************************************/

void PIT1_IRQHandler(void)
{
  PIT_TFLG(PIT1)&=~(PIT_TCTRL_TEN_MASK|PIT_TCTRL_TIE_MASK);//关定时器及中断  
   PIT_Flag_Clear(PIT1);       //清中断标志位
    //disable_irq(PIT0_IRQn);//////////失能点击控制程序，避免采集不及时
    
  static unsigned char TimerCnt20ms = 0;
  TIME1flag_1ms = 1 ;
  TimerCnt20ms++;
  if(TimerCnt20ms ==1)
  {
    StartIntegration();
  }

  if(TimerCnt20ms >=10) {
    TimerCnt20ms = 0;
    TIME1flag_20ms = 1;
  }


  //enable_irq (PIT0_IRQn); 
PIT_TFLG(PIT1)|=(PIT_TCTRL_TEN_MASK|PIT_TCTRL_TIE_MASK);//开定时器及中断，见参考手册976页
   
}

/*************************************************************************
*                             蓝宙电子工作室
*
*  函数名称：PIT0_IRQHandler
*  功能说明：PIT0 定时中断服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-18    已测试
*  备    注：
*************************************************************************/


/*
void PIT0_IRQHandler(void)
{
  PIT_TFLG(PIT0)&=~(PIT_TCTRL_TEN_MASK|PIT_TCTRL_TIE_MASK);//关定时器及中断
   PIT_Flag_Clear(PIT0);       //清中断标志位
   
   TimeCount ++ ;

  if(TimeCount%1 == 0 ){
     TIME0flag_5ms = 1;

 } 
  if(TimeCount%2 == 0 ){
     TIME0flag_10ms = 1;

  } 
  if(TimeCount%3 == 0 ){
     TIME0flag_15ms = 1;
  }
  if(TimeCount%4 == 0 ){
     TIME0flag_20ms = 1;
  }
  if(TimeCount%64 == 0 ){
     TIME0flag_80ms = 1;
  }
  
  if(TimeCount == 192)
  {
    TimeCount = 0 ;
  }
PIT_TFLG(PIT0)|=(PIT_TCTRL_TEN_MASK|PIT_TCTRL_TIE_MASK);//开定时器及中断，见参考手册976页
   
}

*/

