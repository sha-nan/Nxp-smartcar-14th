 /*************************************************************************
 *  文件名   : sensor.c
 *  描述     ：电感数据采集与分析
 *  作者     ：
 *************************************************************************/
#include "common.h"
#include "math.h"
#include "MK60_FTM.h"
#include "MK60_adc.h"
#include "MK60_PIT.h"
#include "Public.h"
#include "Motor.h"
#include "Sensor.h"
#include "Fuzzy.h"
#include  "MK60_flash.h"
/*函数声明*/
void StopCar();//停车函数声明
/*变量声明*/
float  AD_V[5][SENSOR_NUMBER];
float  AD_value[SENSOR_NUMBER],AD_sum[SENSOR_NUMBER];
float  max_v[SENSOR_NUMBER],min_v[SENSOR_NUMBER]; //电感标定采集值
float  ad_value[SENSOR_NUMBER][SENSOR_NUMBER],ad_value1[SENSOR_NUMBER],ad_sum[SENSOR_NUMBER];

float  sensor_middle=0;//中间电感，用于检测坡道
uint16 s1,s2,s3,s4;
uint16 sensor1=0,sensor2=0,sensor3=0,sensor4=0;//从左向右，电感依次为1，2，3，4
uint16 sensor5=0;//,sensor6,sensor7,sensor8;//中间电感
uint8 Vol=0.00;//电池电压

float Slope_AD=0;//获取中间电感值，归一化
float AD[5]={0},sensor[5]={0},sensor_to_one[5]={0};//获取的电感值,归一化
float turn_error=0.00,pre_turn_error=0.00;
float e=0.00;
float ec=0.00;//误差和误差变化率
uint32 turn_out=0;//舵机输出
uint16 zhidao_flag,wandao_flag,stop_flag;//赛道类型标志

int Speed_Choose=0;
int Stop_Car=0,TC=0,R_stopcar=0;//停车变量

/*圆环变量*/
int  out;
int  rightcount=0,leftcount=0;
int  right_flag=0,Right_Count=0,Left_Count=0,left_flag=0;
  /*************************************************************************
  *  函数名称:  AD_Init
  *  功能说明： AD初始化
  *  参数说明：
  *  函数返回： 无
  *  修改时间：
  *  备    注：
  *************************************************************************/
void AD_Init(void)
{
  adc_init(AD_Vol);//采集电压
  /*右边电磁*/
  adc_init(AD1);
  adc_init(AD2);
  adc_init(AD3);
  adc_init(AD4);
  /*左边电磁*/
  adc_init(AD5);
  adc_init(AD6);
  adc_init(AD7);
  adc_init(AD8);
}
  /*************************************************************************
  *  函数名称   AD_Ave
  *  功能说明： 电感值采样
  *  参数说明： 采集 N次求平均
  *  函数返回： 无
  *  修改时间：
  *  备    注：
  *************************************************************************/
uint16 AD_Ave(ADCn_Ch_e adcn_ch, ADC_nbit bit,uint8 N)
{
  uint32 temp = 0;
  uint8 i;
  for(i=0;i<N;i++)temp+=adc_once(adcn_ch,bit);
  temp = temp/N;
  return temp;
}
  /*************************************************************************
  *  函数名称   SC_black_Init
  *  功能说明： 最大值采样
  *  参数说明：
  *  函数返回： 无
  *  修改时间：
  *  备    注：
  *************************************************************************/
void SC_black_Init(void)
{
  uint16 i,j;
  float  sensor_1=0,sensor_2=0,sensor_3=0,sensor_4=0,sensor_5=0;
  max_v[0] = max_v[1] = max_v[2] = max_v[3]= max_v[4]=0;
  min_v[0] = min_v[1] = min_v[2] = min_v[3]= min_v[4]=5;
  if(8==BOMA)//加入拨码盘判断（拨码盘4）
  {
     uint8 C1[8],C2[8];
     sprintf((uint8*)C1,"Collecting");
     LCD_single_P8x16Str(0,0,C1);
     sprintf((uint8*)C2,"Samples...");
     LCD_single_P8x16Str(0,2,C2);
     for(i=0;i<1400;i++)//采集执行的时间
     {
      AD_value[0] = AD_Ave(AD5, ADC_12bit,20);//水平左
      AD_value[1] = AD_Ave(AD6, ADC_12bit,20);//垂直左电感
      AD_value[2] = AD_Ave(AD7, ADC_12bit,20);//垂直右电感
      AD_value[3] = AD_Ave(AD8, ADC_12bit,20);//水平右
      AD_value[4] = AD_Ave(AD1, ADC_12bit,20);//中间电感

      for(j=0;j<SENSOR_NUMBER;j++)
      {
        if(AD_value[j]>max_v[j])
       {
          max_v[j] = AD_value[j];//扫描最大值
        }
      }
      pit_delay_ms(PIT3,1);//延时1ms
    }
    flash_erase_sector(SECTOR_AD);       //擦除254扇区
    for(i=0; i<SENSOR_NUMBER;i++)                   //电感标定的最大值写入扇区
      {
         flash_write(SECTOR_AD,i*4,max_v[i]);
      }
  }

  else
  {
    for(i=0;i<4;i++)
     {
       for(j=0;j<SENSOR_NUMBER;j++)   //读取5个电感的采样标定的最大值
        {
            max_v[j] = flash_read(SECTOR_AD,j*4,int16);
        }
     }
       uint8 C1[8],C2[8];
       sprintf((uint8*)C1,"Reading");
       LCD_single_P8x16Str(0,0,C1);
       sprintf((uint8*)C2,"Samples...");
       LCD_single_P8x16Str(0,2,C2);       
       pit_delay_ms(PIT3,200);//延时10ms
  }
  OLED_Init(); //OLED初始化
}
 /*************************************************************************
 *  函数名称:  AD_Collect
 *  功能说明： AD采集
 *  参数说明：
 *  函数返回： 无
 *  修改时间：
 *  备    注：
 *************************************************************************/
void AD_Collect()//AD采集
{
//    sensor1 =(AD_Ave(ADC0_SE14, ADC_12bit,5)>>2)*0.1;//水平左
//    sensor2 = (AD_Ave(ADC1_SE15, ADC_12bit,5)>>2)*0.1;//垂直左电感
//    sensor3 = (AD_Ave(ADC0_SE13, ADC_12bit,5)>>2)*0.1;//垂直右电感
//    sensor4 =( AD_Ave(ADC0_SE12, ADC_12bit,5)>>2)*0.1;//水平右
//  sensor4 = AD_Ave(ADC0_DP0, ADC_12bit,5);
//  sensor3 = AD_Ave(ADC0_DM0, ADC_12bit,5);
//  sensor2 = AD_Ave(ADC1_DP0, ADC_12bit,5);
//  sensor1 = AD_Ave(ADC1_DM0, ADC_12bit,5);
      //采集5次

//       ad_value[0] = AD_Ave(AD5, ADC_12bit,10);//水平左电感
//       ad_value[1] = AD_Ave(AD6, ADC_12bit,10); //垂直左电感
//       ad_value[2] = AD_Ave(AD7, ADC_12bit,10);//垂直右电
//       ad_value[3] = AD_Ave(AD8, ADC_12bit,10); //水平右电感
//       ad_value[4] = AD_Ave(AD1, ADC_12bit,10);//中间电感
#if 1
    //采集5次
    int16 i,j,k,temp;
    for(i = 0; i <5; i++)
     {
       ad_value[0][i] = AD_Ave(AD5, ADC_12bit,20);//水平左电感
       ad_value[1][i] = AD_Ave(AD6, ADC_12bit,20); //垂直左电感
       ad_value[2][i] = AD_Ave(AD7, ADC_12bit,20);//垂直右电
       ad_value[3][i] = AD_Ave(AD8, ADC_12bit,20); //水平右电感
       ad_value[4][i] = AD_Ave(AD1, ADC_12bit,20);//中间电感
     }
    /*=========================冒泡排序升序==========================*///舍弃最大值和最小值
     for(i=0;i<SENSOR_NUMBER;i++)//5个电感
     {
       for(j=0;j<4;j++)//5个数据排序
       { 
         for(k=0;k<4-j;k++)
         {
           if(ad_value[i][k]>ad_value[i][k+1]) //前面的比后面的大则进行交换
           {
             temp=ad_value[i][k+1];
             ad_value[i][k+1]=ad_value[i][k];
             ad_value[i][k]= temp;
           }
         }
       }
     }

     /*===========================中值滤波=================================*/
     for(i=0;i<SENSOR_NUMBER;i++)    //求中间三项的和
     {
        ad_sum[i] =ad_value[i][1]+ad_value[i][2]+ad_value[i][3]; //舍去最大最小取中间三项
        ad_value1[i]=ad_sum[i]/3; //求平均值
     }
/*测试使用*/
//     sensor1=ad_value1[0];
//     sensor2=ad_value1[1];
//     sensor3=ad_value1[2];
//     sensor4=ad_value1[3];
//     sensor5=ad_value1[4];//最大值1023，最小值0

     /*滑动平均滤波*/
      for(i = 0;i < SENSOR_NUMBER-1;i ++)
      {
          AD_V[0][i] = AD_V[0][i + 1];
          AD_V[1][i] = AD_V[1][i + 1];
          AD_V[2][i] = AD_V[2][i + 1];
          AD_V[3][i] = AD_V[3][i + 1];
          AD_V[4][i] = AD_V[4][i + 1];
      }
      for(i=0;i<5;i++)
      {
          AD_V[i][SENSOR_NUMBER-1] =  ad_value1[i];
      }

      for(i = 0;i < SENSOR_NUMBER;i ++)
      {
          AD_sum[0] += AD_V[0][i];
          AD_sum[1] += AD_V[1][i];
          AD_sum[2] += AD_V[2][i];
          AD_sum[3] += AD_V[3][i];
          AD_sum[4] += AD_V[4][i];
      }
      for(i=0;i<5;i++)  //求平均
      {
          AD[i] = AD_sum[i] / SENSOR_NUMBER;
          AD_sum[i] = 0;
      }

     /*测试使用*/
//     sensor1=AD[0];
//     sensor2=AD[1];
//     sensor3=AD[2];
//     sensor4=AD[3];
//     sensor5=AD[4];//最大值1023，最小值0
#endif
}
/*************************************************************************
 *  函数名称   AD_Date_analyse
 *  功能说明： 电感数据分析
 *  参数说明：
 *  函数返回： 无
 *  修改时间：
 *  备    注：
 *************************************************************************/
void AD_Date_analyse()
{
  int16  i;
  AD_Collect();//电感数据采集
  /*********************归一化处理********************/
  for(i=0;i<SENSOR_NUMBER;i++)
  {
    sensor_to_one[i] = (float)(AD[i] - min_v[i])/(float)(max_v[i] - min_v[i]);
    if(sensor_to_one[i]<=0.0)  sensor_to_one[i]=0.001;
    if(sensor_to_one[i]>1.0)   sensor_to_one[i]=1.0;
    sensor[i] = 100.00 * sensor_to_one[i];     //sensor[i]为归一化后的值,范围为0-100.00
   }

    sensor[0] = (sensor[0] < 1? 1:sensor[0]);	//电感值限幅
    sensor[1] = (sensor[1] < 1? 1:sensor[1]);
    sensor[2] = (sensor[2] < 1? 1:sensor[2]);
    sensor[3] = (sensor[3] < 1? 1:sensor[3]);
  
    sensor1=sensor[0];
    sensor2=sensor[1];
    sensor3=sensor[2];
    sensor4=sensor[3];
   /*******中间电感特殊归一化，用于坡道检测********/
   sensor_middle = (float)(ad_value1[4] - min_v[4])/(float)(max_v[4] - min_v[4]);
   if(sensor_middle <= 0.0)  sensor_middle = 0.001;
   Slope_AD = 100.00 * sensor_middle;
   sensor5=Slope_AD;//坡道检测电感值    
}

/********************开方公式*****************/
uint16 m_sqrt(uint16 x)
{
      uint8 ans=0,p=0x80;
      while(p!=0)
      {
          ans+=p;
          if(ans*ans>x)
          ans-=p;
          p=(uint8)(p/2);
      }
      return(ans);
}

/*************************************************************************
 *  函数名称   Run_Control  
 *  功能说明： 起跑状态
 *  参数说明：
 *  函数返回： 无
 *  修改时间：
 *  备    注：
 *************************************************************************/
#define normal_Run   0     
#define Wait_Ring    1 
#define Running      2 
#define Stop         3

uint8 Run_state=0;
uint8 Run_Flag;
float Dis_Run = 0.0;
extern int k;
void Run_Control(void)
{      
  turn_error=100.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);
  switch(Run_state)
  { 
    case normal_Run://开机正常起步
        Run_Flag = 1;
        Run_state = Wait_Ring;
        if(1==Stop_Car)  Run_state = Stop;//冲出赛道保护,停车检测sensor1==1&&sensor2==1&&sensor3==1&&sensor4==1||
     break;
    
    case Wait_Ring://等待环岛
        Run_Flag = 1;
        Run_state = Running;
        if(1==Stop_Car)  Run_state = Stop;//冲出赛道保护,停车检测sensor1==1&&sensor2==1&&sensor3==1&&sensor4==1||
     break;
    
    case Running://正常跑
        Run_Flag = 1;
//        Ring_Control();//环岛检测
        if(1==Stop_Car)  Run_state = Stop;//冲出赛道保护,停车检测sensor1==1&&sensor2==1&&sensor3==1&&sensor4==1||
     break;
    
    case Stop://停止
        {
          Run_Flag = 0;
        }
     break;		
	
    default :break;    
  }
    FTM_PWM_Duty(FTM1, FTM_CH0, turn_out_cal());//转向控制  
}
/*************************************************************************
 *  函数名称   Ring_Control  
 *  功能说明： 环岛检测
 *  参数说明：
 *  函数返回： 无
 *  修改时间：
 *  备    注：
 *************************************************************************/
#define No_Ring            0
#define Find_Right_Ring    1
#define Find_Left_Ring     2
#define Ready_Right_Ring   3
#define Ready_Left_Ring    4
#define Start_Right_Ring   5
#define Start_Left_Ring    6
#define In_Ring            7
#define Out_Ring           8
#define Real_Out_Ring      9


uint8  Ring_Flag = 0;
uint8 Speed_Flag = 0;
uint8 Ring_state;
uint8 Go_Ring_Flag=0,Out_Ring_Flag=0;
void Ring_Control(void)
{
  switch(Ring_state)
  {
    case No_Ring://第一步：判断是否出现环
      turn_error=100.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
//      if(sensor2>70&&50<sensor3&&sensor3<80&&sensor1>80&&sensor4>80)//(右环)
      if(sensor2>20&&30<sensor3&&sensor3<80&&sensor1>80&&sensor4>80)//(右环)      
      {
        Ring_state  = Find_Right_Ring;
      }
      else if(sensor3>20&&30<sensor2&&sensor2<80&&sensor1>80&&sensor4>80)//(左环)
//      else if(sensor3>20&&30<sensor2&&sensor2<80&&sensor1>80&&sensor4>80)//(左环)
      {
        Ring_state  = Find_Left_Ring;
      }
      else
        Ring_state  = No_Ring;
    break;
      
    case Find_Right_Ring://第二步：判断进环点(右环)
      turn_error=100.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
//      if(sensor2>70&&sensor3>70)
      if(sensor2>20&&sensor3>40)
      {
        Ring_state  = Ready_Right_Ring;
      }
    break;
      
    case Find_Left_Ring://第二步：判断进环点(左环)
      turn_error=100.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
      if(sensor2>40&&sensor3>20)
      {
        Ring_state  = Ready_Left_Ring;
      }
    break;  
    
    case Ready_Right_Ring://第三步：准备进环点 （右环）
      turn_error=200.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
//      if(30<sensor2&&sensor2<50&&sensor3>80)
      if(20<sensor2&&sensor2<40&&sensor3>80)
      {
        Go_Ring_Flag=1;//进环减速标志
        Ring_state  = Start_Right_Ring;
      }
    break;
    
    case Ready_Left_Ring://第三步：准备进环点  （左环）
      turn_error=200.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
      if(20<sensor3&&sensor3<40&&sensor2>80)
      {
        Go_Ring_Flag=1;//进环减速标志
        Ring_state  = Start_Left_Ring;
      }
    break; 
    
    case Start_Right_Ring://第四步：开始进环，以固定偏差走一段距离(右环)
      turn_error=200.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
      if(sensor3>86&&sensor2<32)
      {
        if(1==BOMA)//速度260
        {
            FTM_PWM_Duty(FTM1, FTM_CH0, 1010);//舵机向右打死,右环前一段死值打角处理
            pit_delay_ms(PIT3,340);//延时                
        }
        else if(2==BOMA)//速度270
        {
            FTM_PWM_Duty(FTM1, FTM_CH0, 1010);//舵机向右打死,右环前一段死值打角处理
            pit_delay_ms(PIT3,340);//延时                
        } 
        else if(4==BOMA)//速度280
        {
            FTM_PWM_Duty(FTM1, FTM_CH0, 1010);//舵机向右打死,右环前一段死值打角处理
            pit_delay_ms(PIT3,340);//延时                
        }
        else//速度250（默认）
        {
            FTM_PWM_Duty(FTM1, FTM_CH0, 1008);//舵机向右打死,右环前一段死值打角处理
            pit_delay_ms(PIT3,300);//延时                
        }
        if(sensor1<60&&sensor2<10&&sensor3>80&&sensor4>80)
        {
           Out_Ring_Flag=1;//出环减速标志
           Ring_state = In_Ring;
        }
      }
    break;

    case Start_Left_Ring://第四步：开始进环，以固定偏差走一段距离（左环）
      turn_error=300.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
      if(sensor2>87&&sensor3<32)
      {
        if(1==BOMA)//速度260
        {
            FTM_PWM_Duty(FTM1, FTM_CH0, 1160);//舵机向右打死,右环前一段死值打角处理
            pit_delay_ms(PIT3,340);//延时                
        }
        else if(2==BOMA)//速度270
        {
            FTM_PWM_Duty(FTM1, FTM_CH0, 1160);//舵机向右打死,右环前一段死值打角处理
            pit_delay_ms(PIT3,340);//延时                
        } 
        else if(4==BOMA)//速度280
        {
            FTM_PWM_Duty(FTM1, FTM_CH0, 1160);//舵机向右打死,右环前一段死值打角处理
            pit_delay_ms(PIT3,340);//延时                
        }
        else//速度250（默认）
        {
            FTM_PWM_Duty(FTM1, FTM_CH0, 1160);//舵机向右打死,右环前一段死值打角处理
            pit_delay_ms(PIT3,335);//延时                
        }
        if(sensor1<60&&sensor2<10&&sensor3>80&&sensor4>80)
        {
           Out_Ring_Flag=1;//出环减速标志
           Ring_state = In_Ring;
        }
        
//        if(sensor1>80&&sensor2>80&&sensor3<10&&sensor4<80)
//        {
////          BEEP_ON;//蜂鸣器
//          Ring_state = In_Ring;
//        }
      }
    break;
      
    case In_Ring://第五步：环岛中
      turn_error=300.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
//      if(sensor1>40&&sensor2<20)
      if((sensor2>20&&20<sensor3&&sensor3<70)||(sensor3>20&&20<sensor2&&sensor2<70))
      {      
        Ring_state = Out_Ring;
      }
    break;
    
    case Out_Ring://第六步：出环  两边的电感一定会出现极大值
      turn_error=400.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
      if(sensor1>20&&sensor4>80||sensor2>80||sensor2>0&&sensor3>80||sensor1>80&&sensor4>20)
      {
        Ring_state  = Real_Out_Ring;
      }
    break;
    
    case Real_Out_Ring://第七步：出环  四个电感值都减小后  回到无环状态
      turn_error=400.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
      if(abs(sensor1-sensor2)<40 && (sensor3-sensor4)<40) //出环条件
      {
        Ring_state  = No_Ring;
      }
      break;
  default:break;		
  }
}
/********************舵机控制(位置式PD)*****************/
int16 turn_out_cal()//舵机控制(位置式PD);float kp,float kd
{  
//       turn_error=20*(sensor1-sensor4)/(sensor1+sensor4);//+0.5*(sensor2-sensor3)/(sensor2+sensor3);
   if(sensor1>90&&sensor4>90&&((sensor2>95&&sensor3<5)||(sensor3>95&&sensor2<5)))
      {
        turn_error=100.00*(sensor1-sensor4)/(sensor1+sensor4);
        float kp,kd;
        kp=40.00;
        kd=1.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=1;
      }
   else
   {
    switch(BOMA)
    {
     /*一档，速度260*/     
    case 1:
      Speed_Choose=1;
      if(abs(turn_error)<=10)//直道
      {
        float kp,kd;
        kp=1.0;
        kd=2.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=2;        
      }     
      else if(abs(turn_error)<=20)//直道
      {
        float kp,kd;
        kp=1.0;
        kd=2.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=3;        
      }      
      else if(abs(turn_error)<=30)
      {
        float kp,kd;
        kp=1.00;
        kd=3.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=4;
      }     
      else if(abs(turn_error)<=40)
      {
        float kp,kd;
        kp=1.00;
        kd=4.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=5;        
      }
      else if(abs(turn_error)<=50)//小s弯
      {
        
        float kp,kd;
        kp=1.0;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd; 
        Speed_Flag=6;
      }
      else if(abs(turn_error)<=60)
      {
        float kp,kd;
        kp=1.0;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=7;        
      }
      else if(abs(turn_error)<=70)
      {
        float kp,kd;
        kp=1.00;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=8;        
      } 
      else if(abs(turn_error)<=80)
      {
        float kp,kd;
        kp=1.00;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=9;        
      } 
      else if(abs(turn_error)<=90)
      {
        float kp,kd;
        kp=1.0;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=10;          
      }       
      else 
      {
        float kp,kd;
        kp=1.00;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=11;        
      }
      break;
    /*二档，速度270*/ 
      
    case 2:
      Speed_Choose=2;      
      if(abs(turn_error)<=10)//直道
      {
        float kp,kd;
        kp=1.0;
        kd=2.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=2;        
      }     
      else if(abs(turn_error)<=20)//直道
      {
        float kp,kd;
        kp=1.0;
        kd=2.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=3;        
      }      
      else if(abs(turn_error)<=30)
      {
        float kp,kd;
        kp=1.00;
        kd=3.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=4;
      }     
      else if(abs(turn_error)<=40)
      {
        float kp,kd;
        kp=1.00;
        kd=4.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=5;        
      }
      else if(abs(turn_error)<=50)//小s弯
      {
        
        float kp,kd;
        kp=1.0;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd; 
        Speed_Flag=6;
      }
      else if(abs(turn_error)<=60)
      {
        float kp,kd;
        kp=1.0;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=7;        
      }
      else if(abs(turn_error)<=70)
      {
        float kp,kd;
        kp=1.00;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=8;        
      } 
      else if(abs(turn_error)<=80)
      {
        float kp,kd;
        kp=1.00;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=9;        
      } 
      else if(abs(turn_error)<=90)
      {
        float kp,kd;
        kp=1.0;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=10;          
      }       
      else 
      {
        float kp,kd;
        kp=1.00;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=11;        
      }
      break;
      
     /*三档，速度280*/        
    case 4:
      Speed_Choose=3;      
      if(abs(turn_error)<=10)//直道
      {
        float kp,kd;
        kp=1.0;
        kd=2.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=2;        
      }     
      else if(abs(turn_error)<=20)//直道
      {
        float kp,kd;
        kp=1.0;
        kd=2.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=3;        
      }      
      else if(abs(turn_error)<=30)
      {
        float kp,kd;
        kp=1.00;
        kd=3.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=4;
      }     
      else if(abs(turn_error)<=40)
      {
        float kp,kd;
        kp=1.00;
        kd=4.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=5;        
      }
      else if(abs(turn_error)<=50)//小s弯
      {
        
        float kp,kd;
        kp=1.0;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd; 
        Speed_Flag=6;
      }
      else if(abs(turn_error)<=60)
      {
        float kp,kd;
        kp=1.0;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=7;        
      }
      else if(abs(turn_error)<=70)
      {
        float kp,kd;
        kp=1.00;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=8;        
      } 
      else if(abs(turn_error)<=80)
      {
        float kp,kd;
        kp=1.00;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=9;        
      } 
      else if(abs(turn_error)<=90)
      {
        float kp,kd;
        kp=1.0;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=10;          
      }       
      else 
      {
        float kp,kd;
        kp=1.00;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=11;        
      }
      break;
      
     /*默认，速度250*/        
    default:
      if(abs(turn_error)<=10)//直道
      {
        float kp,kd;
        kp=1.0;
        kd=2.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=2;        
      }     
      else if(abs(turn_error)<=20)//直道
      {
        float kp,kd;
        kp=1.0;
        kd=2.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=3;        
      }      
      else if(abs(turn_error)<=30)
      {
        float kp,kd;
        kp=1.00;
        kd=2.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=4;
      }     
      else if(abs(turn_error)<=40)
      {
        float kp,kd;
        kp=1.00;
        kd=2.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=5;        
      }
      else if(abs(turn_error)<=50)//小s弯
      {
        
        float kp,kd;
        kp=1.0;
        kd=21.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd; 
        Speed_Flag=6;
      }
      else if(abs(turn_error)<=60)
      {
        float kp,kd;
        kp=1.0;
        kd=22.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=7;        
      }
      else if(abs(turn_error)<=70)
      {
        float kp,kd;
        kp=1.00;
        kd=23.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=8;        
      } 
      else if(abs(turn_error)<=80)
      {
        float kp,kd;
        kp=1.00;
        kd=24.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=9;        
      } 
      else if(abs(turn_error)<=90)
      {
        float kp,kd;
        kp=1.0;
        kd=25.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=10;          
      }       
      else 
      {
        float kp,kd;
        kp=1.00;
        kd=26.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=11;        
      }
      break;      
    } 
   } 
      e = turn_error;//当前偏差        
      ec= turn_error-pre_turn_error;//偏差变化率（当前偏差和上次偏差的差值）    
      pre_turn_error=turn_error;     
      if(turn_out<=DirectRight) turn_out = DirectRight;//对舵机打角进行限幅
      else if(turn_out>=DirectLeft) turn_out = DirectLeft;
//      printf("turn_out=%4d\n",turn_out);
      return turn_out;      
}


/******环岛检测判断*****/
void turn_control()
{
#if 0
    /*非环岛执行*/
//      if(sensor1>30&&sensor4>30&&abs(sensor1-sensor4)<50)//else
//      {
//        FTM_PWM_Duty(FTM1, FTM_CH0, turn_out_cal());//260,0
//        Motor_output1(2000);
//        Motor_output2(2000);
//      }
//
//      else if (sensor1==0&&sensor2==0&&sensor3==0&&sensor4==0&&sensor5==0)
//      {
//        FTM_PWM_Duty(FTM1, FTM_CH0,1082);
//        Motor_output1(0);
//        Motor_output2(0);
//      }
//      else
//      {
//        FTM_PWM_Duty(FTM1, FTM_CH0, turn_out_cal());//950,0
//        Motor_output1(2000);
//        Motor_output2(2000);
//      }
}
#endif
void StopCar()
{
       if(gpio_get(PTB22)==0)//第一次过磁铁(第一块板)  
//       if(gpio_get(PTC13)==0)//第一次过磁铁(第二块板)
       {
          R_stopcar=1;
       }
       if(gpio_get(PTB22)==1&&R_stopcar==1) //(第一块板)        
//       if(gpio_get(PTC13)==1&&R_stopcar==1) //(第二块板) 
       {
          R_stopcar=0;
          TC=1;
       }
       if(gpio_get(PTB22)==0&&TC==1)//第二次过磁铁,停车       
//       if(gpio_get(PTC13)==0&&TC==1)//第二次过磁铁,停车
       {  
         Stop_Car=1;
       }  
}