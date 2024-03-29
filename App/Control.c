#include "common.h"
#include "Control.h"
#include "Public.h"
#include  "MK60_FTM.h"
#include  "MK60_PIT.h"     //周期中断计时器
#include "MK60_adc.h"
/*速度选择*/
#define Speed_0 250
#define Speed_1 260
#define Speed_2 270
#define Speed_3 280
#define Speed_PoDao 300

int i=0,j=0;//坡道电机速度延时

int16 Turn_Out;
int16 Error;
int16 Car_Sudu;
int16 zhuangshu,zuo_zhuanshu,you_zhuanshu,zuo_error,you_error;//编码器转数期望值
float Turn_KP,Turn_KD;

int PoDao_Speed=0;//坡道速度

extern uint16 sensor1,sensor2;
extern float turn_error,turn_lasterror;//转向误差
extern uint8 Go_Ring_Flag;//进环减速标志
extern uint8 Out_Ring_Flag;//出环检测标志
extern uint8 PoDao_Flag; 
/*函数声明*/
extern void get_speed();
extern void Speed_Control();
extern int16 Speed_PID(int16 Goal,int16 Input);
/********************电感采集中断*******************************/
void PIT0_IRQHandler(void)
{
    PIT_TFLG(PIT0)&=~(PIT_TCTRL_TEN_MASK|PIT_TCTRL_TIE_MASK);//关定时器及中断
    AD_Date_analyse();//电感采集，数据处理
    StopCar();//停车检测   
    Run_Control();//车辆状态
    PIT_Flag_Clear(PIT0); //清中断标志位
    PIT_TFLG(PIT0)|=(PIT_TCTRL_TEN_MASK|PIT_TCTRL_TIE_MASK);//开定时器及中断，见参考手册976页
}
/******************************速度计算中断*********************************/
int Real_Speed=0;
int Left_Speed,Right_Speed;
uint8 time_counter=0;//时间标志位
void PIT1_IRQHandler(void)//速度计算中断服务函数
{
    PIT_TFLG(PIT1)&=~(PIT_TCTRL_TEN_MASK|PIT_TCTRL_TIE_MASK);//关定时器及中断
    time_counter++;//时间标志位
    if(50==time_counter)    //50ms测一次速度
    {
      time_counter=0;  //标志位清零
      get_speed();     //测速函数
      Speed_Control(); //速度控制
      Left_Speed=0;
      Right_Speed=0;      
    }
    PIT_Flag_Clear(PIT1);   //清中断标志位
    PIT_TFLG(PIT1)|=(PIT_TCTRL_TEN_MASK|PIT_TCTRL_TIE_MASK);//开定时器及中断
}

/********************速度控制********************************/
float Speed_P,Speed_I,Speed_D;
extern int RightWheel_Count,LeftWheel_Count;
void get_speed()//速度计算
{
      Left_Speed = LeftWheel_Count;//左电机速度
      Right_Speed = RightWheel_Count;//右电机速度
      Real_Speed = (Left_Speed+Right_Speed)/2; //车身速度 
      
//      if(0!=Real_Speed&&0!=Left_Speed)
//      printf("times:%4d\n Re=%4d\n" ,k,Real_Speed);
      
      LeftWheel_Count=0;
      RightWheel_Count=0;//清零测速数据，等于是每50ms计数一次
      
      
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

extern int Speed_Choose;
extern uint8 Speed_Flag;
void Speed_Control()
{
  /********************速度控制，一档速度260********************************/
  if(1==Speed_Choose)
  {
    if(11==Speed_Flag)
    {
      Car_Sudu=Speed_1;//设置目标速度
      Speed_P=1.0;//速度Kp
      Speed_I=0.2;//速度Ki
      Speed_D=0.0;//速度Kd
      Turn_KP=1.0;//差速Kp
      Turn_KD=0.0;//差速Kd  
    }
   else if(10==Speed_Flag)
    {
      Car_Sudu=Speed_1;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0; 
    }    
   else if(9==Speed_Flag)
    {
      Car_Sudu=Speed_1;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0; 
    }
   else if(8==Speed_Flag)
    {
      Car_Sudu=Speed_1;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;   
    }
    else if(7==Speed_Flag)
    {
      Car_Sudu=Speed_1;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;      
    }
   else if(6==Speed_Flag)
    {
      Car_Sudu=Speed_1;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;     
    }
   else if(5==Speed_Flag)
    {
      Car_Sudu=Speed_1;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;    
    }  
   else if(4==Speed_Flag)
    {
      Car_Sudu=Speed_1;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;     
    }  
   else if(3==Speed_Flag)
    {
      Car_Sudu=Speed_1;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;    
    }  
   else if(2==Speed_Flag)
    {
      Car_Sudu=Speed_1;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;       
    }        
    else if(1==Speed_Flag)
    {
      Car_Sudu=Speed_1;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;     
    }
   else
    {
      Car_Sudu=Speed_1;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0; 
    }      
    Speed_Flag=0;
  }
 /********************二档，速度270********************************/ 
  else if(2==Speed_Choose)
  {
    if(11==Speed_Flag)
    {
      Car_Sudu=Speed_2;//设置目标速度
      Speed_P=1.0;//速度Kp
      Speed_I=0.2;//速度Ki
      Speed_D=0.0;//速度Kd
      Turn_KP=1.0;//差速Kp
      Turn_KD=0.0;//差速Kd  
    }
   else if(10==Speed_Flag)
    {
      Car_Sudu=Speed_2;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0; 
    }    
   else if(9==Speed_Flag)
    {
      Car_Sudu=Speed_2;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0; 
    }
   else if(8==Speed_Flag)
    {
      Car_Sudu=Speed_2;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;   
    }
    else if(7==Speed_Flag)
    {
      Car_Sudu=Speed_2;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;      
    }
   else if(6==Speed_Flag)
    {
      Car_Sudu=Speed_2;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;     
    }
   else if(5==Speed_Flag)
    {
      Car_Sudu=Speed_2;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;    
    }  
   else if(4==Speed_Flag)
    {
      Car_Sudu=Speed_2;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;     
    }  
   else if(3==Speed_Flag)
    {
      Car_Sudu=Speed_2;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;    
    }  
   else if(2==Speed_Flag)
    {
      Car_Sudu=Speed_2;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;       
    }        
    else if(1==Speed_Flag)
    {
      Car_Sudu=Speed_2;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;     
    }
   else
    {
      Car_Sudu=Speed_2;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0; 
    }      
    Speed_Flag=0;
  }
  /********************三档，速度280********************************/
 else if(3==Speed_Choose)
  {
    if(11==Speed_Flag)
    {
      Car_Sudu=270;//设置目标速度
      Speed_P=1.0;//速度Kp
      Speed_I=0.2;//速度Ki
      Speed_D=0.0;//速度Kd
      Turn_KP=1.0;//差速Kp
      Turn_KD=0.0;//差速Kd  
    }
   else if(10==Speed_Flag)
    {
      Car_Sudu=270;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0; 
    }    
   else if(9==Speed_Flag)
    {
      Car_Sudu=270;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0; 
    }
   else if(8==Speed_Flag)
    {
      Car_Sudu=270;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;   
    }
    else if(7==Speed_Flag)
    {
      Car_Sudu=270;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;      
    }
   else if(6==Speed_Flag)
    {
      Car_Sudu=270;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;     
    }
   else if(5==Speed_Flag)
    {
      Car_Sudu=280;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;    
    }  
   else if(4==Speed_Flag)
    {
      Car_Sudu=280;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;     
    }  
   else if(3==Speed_Flag)
    {
      Car_Sudu=280;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;    
    }  
   else if(2==Speed_Flag)
    {
      Car_Sudu=280;
      Speed_P=1.0;
      Speed_I=0.2;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;       
    }        
    else if(1==Speed_Flag)
    {
      Car_Sudu=280;
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
  }
/********************其他，速度250********************************/
 else 
  {
    if(1==PoDao_Flag)//坡道标志&&turn_error<=40
    {
//      Car_Sudu=Speed_PoDao;//设置目标速度
      Car_Sudu=600;//设置目标速度      
      Speed_P=0.0;//速度Kp
      Speed_I=1.0;//速度Ki
      Speed_D=0.05;//速度Kd
      Turn_KP=1.0;//差速Kp
      Turn_KD=0.0;//差速Kd
      PoDao_Flag=0; //清除坡道标志
      PoDao_Speed=1;//坡道速度标志
    }    
    else if(1==Go_Ring_Flag)//进环减速标志
    {
      Car_Sudu=240;//设置目标速度
      Speed_P=0.0;//速度Kp
      Speed_I=1.1;//速度Ki
      Speed_D=0.01;//速度Kd
      Turn_KP=1.0;//差速Kp
      Turn_KD=0.0;//差速Kd
      Go_Ring_Flag=0;//清除进环减速标志
    }
    else if(1==Out_Ring_Flag)
    {
      Car_Sudu=200;//设置目标速度
      Speed_P=2.0;//速度Kp
      Speed_I=5.0;//速度Ki
      Speed_D=0.0;//速度Kd
      Turn_KP=1.0;//差速Kp
      Turn_KD=0.0;//差速Kd
      Out_Ring_Flag=0;//出环减速标志
    }   
   else if(11==Speed_Flag)
    {
      Car_Sudu=250;//设置目标速度
      Speed_P=0.0;//速度Kp
      Speed_I=0.4;//速度Ki
      Speed_D=0.0;//速度Kd
      Turn_KP=1.0;//差速Kp
      Turn_KD=0.0;//差速Kd  
    }
   else if(10==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=0.0;
      Speed_I=0.6;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0; 
    }    
   else if(9==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=0.0;
      Speed_I=0.6;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;  
    }
   else if(8==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=0.0;
      Speed_I=0.6;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;    
    }
    else if(7==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=0.0;
      Speed_I=0.6;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;      
    }
   else if(6==Speed_Flag)
    {
      Car_Sudu=Speed_0;
      Speed_P=0.0;
      Speed_I=0.6;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;      
    }
   else if(5==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=0.0;
      Speed_I=0.6;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;    
    }  
   else if(4==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=0.0;
      Speed_I=0.6;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;     
    }  
   else if(3==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=0.0;
      Speed_I=0.6;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;    
    }  
   else if(2==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=0.0;
      Speed_I=0.6;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;        
    }        
    else if(1==Speed_Flag)
    {
      Car_Sudu=250;
      Speed_P=0.0;
      Speed_I=0.6;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;      
    }
   else
    {
      Car_Sudu=250;
      Speed_P=0.0;
      Speed_I=0.6;
      Speed_D=0.0;
      Turn_KP=1.0;
      Turn_KD=0.0;  
    }      
    Speed_Flag=0;
  }    
  
    Turn_PD(e);   //计算转向偏差
    zuo_zhuanshu = Car_Sudu-Turn_Out;//差速
    you_zhuanshu = Car_Sudu+Turn_Out;
    
    if(!Run_Flag)
      {
        Motor_output1(0);
        Motor_output2(0);
        BEEP_OFF;//蜂鸣器        
      }
    else if(1==PoDao_Speed)
      {

         Motor_output1(Speed_PID(zuo_zhuanshu,Left_Speed));
         Motor_output2(Speed_PID(you_zhuanshu,Right_Speed));

//        Motor_output1(3000);
//        Motor_output2(3000);        
        PTC5_OUT=0;
        PoDao_Speed=0;
      }      
    else
      {
        Motor_output1(Speed_PID(zuo_zhuanshu,Left_Speed));
        Motor_output2(Speed_PID(you_zhuanshu,Right_Speed)); 
      }
}

