#include "common.h"
#include  "OLED.h"
#include "Init.h"
#include "Sensor.h"
#include "MK60_uart.h"
#include "Boma.h"
#include "misc.h"
#include "MK60_adc.h"
#include "Motor.h"
#include "Public.h"
#include  "MK60_PIT.h"//���ڶ�ʱ���ж�
#include "MK60_port.h"

extern void PORTB_IRQHandler();//PORTB�жϷ�����
extern void PIT1_IRQHandler(void);//1ms�жϷ�����
extern void PIT0_IRQHandler(void);//10ms�жϷ�����
extern void FTM2_INPUT_IRQHandler();//FTM2�жϷ�����
void Ftm_input_init();//��ʼ��FTM���벶׽ģʽ

void wildWolf_init(void)//��ʼ��
{
       OLED_Init(); //OLED��ʼ��
       gpio_init(PTC5,GPO,0);
       gpio_init(PTC18,GPO,0);//LED��ʼ��
       gpio_init(PTA19,GPO,0);//��λLED
       gpio_init(PTB20,GPO,0);//������
       gpio_init(PTB21,GPI,0);//����ģ��
      /*��ʼ������IO��������ȡ��ת����*/
       gpio_init(PTB17,GPI,1);//������PhaseA2,�������ţ���
       gpio_init(PTB16,GPI,1);//������PhaseB2,�������ţ��ң�
       Boma_init();//�����̳�ʼ��
       Boma_read();//�����̶�ȡ
       AD_Init();//AD��ʼ��
       SC_black_Init();//���ֵ����
       uart_init(UART4,9600);//���ڳ�ʼ��
       FTM_PWM_init(FTM1, FTM_CH0,50, 1082);//�����ʼ��
       FTM_PWM_Duty(FTM1, FTM_CH0,1082);
       Motor_init();//�����ʼ��
       Ftm_input_init();//��ʼ��FTM���벶׽ģʽ��������
       port_init (PTB21, IRQ_ZERO | PF | ALT1 | PULLUP );
   /************************* �����ж����ȼ�  ************************/    
       NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);
       NVIC_SetPriority(PORTB_IRQn,0);          //��һ����: �������ж�
       NVIC_SetPriority(PIT0_IRQn, 1);          //��������: 10ms���ƶ���ж�
       NVIC_SetPriority(FTM2_IRQn, 2);          //��һ����: ���벶׽���½��ش����ж�
       NVIC_SetPriority(PIT1_IRQn, 3);          //�ڶ�����: 1ms��ʱ���жϣ����Ƶ����
       
       set_vector_handler(PORTB_VECTORn ,PORTB_IRQHandler);//����PORTB���жϷ�����ΪPORTB_IRQHandler
       enable_irq(PORTB_IRQn);
       
       pit_init_ms(PIT0,5);  //10ms��ʱ�жϣ�AD�ɼ�
       set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);//���� PIT0 ���жϸ�λ����Ϊ PIT0_IRQHandler
       enable_irq(PIT0_IRQn);
       
       pit_init_ms(PIT1,1);  //1ms��ʱ�ж�,�ٶȼ���
       set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);//���� PIT1 ���жϸ�λ����Ϊ PIT0_IRQHandler
       enable_irq(PIT1_IRQn);

       set_vector_handler(FTM2_VECTORn ,FTM2_INPUT_IRQHandler);    //����FTM2���жϷ�����Ϊ FTM2_INPUT_IRQHandler
       enable_irq (FTM2_IRQn);                                     //ʹ��FTM2�ж�
}

void Ftm_input_init()//��ʼ��FTM���벶׽ģʽ
{
   ftm_input_init(FTM2, FTM_CH0 , FTM_Falling, FTM_PS_2 );    //��ʼ��FTM���벶׽ģʽ�������ز�׽�����ڵ�һ·���٣�
   ftm_input_init(FTM2, FTM_CH1 , FTM_Falling, FTM_PS_2 );    //��ʼ��FTM���벶׽ģʽ�������ز�׽�����ڵڶ�·���٣�   
}