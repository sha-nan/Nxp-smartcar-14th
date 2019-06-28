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
#include  "MK60_PIT.h"//周期定时器中断
#include "MK60_port.h"

extern void PORTB_IRQHandler();//PORTB中断服务函数
extern void PIT1_IRQHandler(void);//1ms中断服务函数
extern void PIT0_IRQHandler(void);//10ms中断服务函数
extern void FTM2_INPUT_IRQHandler();//FTM2中断服务函数
void Ftm_input_init();//初始化FTM输入捕捉模式

void wildWolf_init(void)//初始化
{
       OLED_Init(); //OLED初始化
       gpio_init(PTC5,GPO,0);
       gpio_init(PTC18,GPO,0);//LED初始化
       gpio_init(PTA19,GPO,0);//复位LED
       gpio_init(PTB20,GPO,0);//蜂鸣器
       gpio_init(PTB21,GPI,0);//激光模块
      /*初始化两个IO口用来读取旋转方向*/
       gpio_init(PTB17,GPI,1);//编码器PhaseA2,方向引脚（左）
       gpio_init(PTB16,GPI,1);//编码器PhaseB2,方向引脚（右）
       Boma_init();//拨码盘初始化
       Boma_read();//拨码盘读取
       AD_Init();//AD初始化
       SC_black_Init();//最大值采样
       uart_init(UART4,9600);//串口初始化
       FTM_PWM_init(FTM1, FTM_CH0,50, 1082);//舵机初始化
       FTM_PWM_Duty(FTM1, FTM_CH0,1082);
       Motor_init();//电机初始化
       Ftm_input_init();//初始化FTM输入捕捉模式，编码器
       port_init (PTB21, IRQ_ZERO | PF | ALT1 | PULLUP );
   /************************* 配置中断优先级  ************************/    
       NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);
       NVIC_SetPriority(PORTB_IRQn,0);          //第一优先: 红外检测中断
       NVIC_SetPriority(PIT0_IRQn, 1);          //第三优先: 10ms控制舵机中断
       NVIC_SetPriority(FTM2_IRQn, 2);          //第一优先: 输入捕捉，下降沿触发中断
       NVIC_SetPriority(PIT1_IRQn, 3);          //第二优先: 1ms定时器中断（控制电机）
       
       set_vector_handler(PORTB_VECTORn ,PORTB_IRQHandler);//设置PORTB的中断服务函数为PORTB_IRQHandler
       enable_irq(PORTB_IRQn);
       
       pit_init_ms(PIT0,5);  //10ms定时中断，AD采集
       set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);//设置 PIT0 的中断复位函数为 PIT0_IRQHandler
       enable_irq(PIT0_IRQn);
       
       pit_init_ms(PIT1,1);  //1ms定时中断,速度计算
       set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);//设置 PIT1 的中断复位函数为 PIT0_IRQHandler
       enable_irq(PIT1_IRQn);

       set_vector_handler(FTM2_VECTORn ,FTM2_INPUT_IRQHandler);    //设置FTM2的中断服务函数为 FTM2_INPUT_IRQHandler
       enable_irq (FTM2_IRQn);                                     //使能FTM2中断
}

void Ftm_input_init()//初始化FTM输入捕捉模式
{
   ftm_input_init(FTM2, FTM_CH0 , FTM_Falling, FTM_PS_2 );    //初始化FTM输入捕捉模式，上升沿捕捉（用于第一路测速）
   ftm_input_init(FTM2, FTM_CH1 , FTM_Falling, FTM_PS_2 );    //初始化FTM输入捕捉模式，上升沿捕捉（用于第二路测速）   
}