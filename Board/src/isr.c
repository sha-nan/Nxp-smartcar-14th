/******************** (C) COPYRIGHT 2011 ������ӹ����� ********************
 * �ļ���       ��isr.c
 * ����         ���жϴ�������
 *
 * ʵ��ƽ̨     ��Ұ��kinetis������
 * ��汾       ��
 * Ƕ��ϵͳ     ��
 *
 * ����         ��������ӹ�����
 * �Ա���       ��http://landzo.taobao.com/
**********************************************************************************/



/******************** (C) COPYRIGHT 2011 ������ӹ����� ********************
 * �ļ���       ��isr.c
 * ����         ���жϴ�������
 *
 * ʵ��ƽ̨     ��Ұ��kinetis������
 * ��汾       ��
 * Ƕ��ϵͳ     ��
 *
 * ����         ��������ӹ�����
 * �Ա���       ��http://landzo.taobao.com/
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
*                             ����Ƕ��ʽ����������
*
*  �������ƣ�PIT1_IRQHandler
*  ����˵����PIT1��ʱ�жϷ�����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-9-18    �Ѳ���
*  ��    ע��
*************************************************************************/

void PIT1_IRQHandler(void)
{
  PIT_TFLG(PIT1)&=~(PIT_TCTRL_TEN_MASK|PIT_TCTRL_TIE_MASK);//�ض�ʱ�����ж�  
   PIT_Flag_Clear(PIT1);       //���жϱ�־λ
    //disable_irq(PIT0_IRQn);//////////ʧ�ܵ�����Ƴ��򣬱���ɼ�����ʱ
    
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
PIT_TFLG(PIT1)|=(PIT_TCTRL_TEN_MASK|PIT_TCTRL_TIE_MASK);//����ʱ�����жϣ����ο��ֲ�976ҳ
   
}

/*************************************************************************
*                             ������ӹ�����
*
*  �������ƣ�PIT0_IRQHandler
*  ����˵����PIT0 ��ʱ�жϷ�����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-2-18    �Ѳ���
*  ��    ע��
*************************************************************************/


/*
void PIT0_IRQHandler(void)
{
  PIT_TFLG(PIT0)&=~(PIT_TCTRL_TEN_MASK|PIT_TCTRL_TIE_MASK);//�ض�ʱ�����ж�
   PIT_Flag_Clear(PIT0);       //���жϱ�־λ
   
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
PIT_TFLG(PIT0)|=(PIT_TCTRL_TEN_MASK|PIT_TCTRL_TIE_MASK);//����ʱ�����жϣ����ο��ֲ�976ҳ
   
}

*/

