 /*************************************************************************
 *  �ļ���   : sensor.c
 *  ����     ��������ݲɼ������
 *  ����     ��
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
/*��������*/
void StopCar();//ͣ����������
/*��������*/
float  AD_V[5][SENSOR_NUMBER];
float  AD_value[SENSOR_NUMBER],AD_sum[SENSOR_NUMBER];
float  max_v[SENSOR_NUMBER],min_v[SENSOR_NUMBER]; //��б궨�ɼ�ֵ
float  ad_value[SENSOR_NUMBER][SENSOR_NUMBER],ad_value1[SENSOR_NUMBER],ad_sum[SENSOR_NUMBER];

float  sensor_middle=0;//�м��У����ڼ���µ�
uint16 s1,s2,s3,s4;
uint16 sensor1=0,sensor2=0,sensor3=0,sensor4=0;//�������ң��������Ϊ1��2��3��4
uint16 sensor5=0;//,sensor6,sensor7,sensor8;//�м���
uint8 Vol=0.00;//��ص�ѹ

float Slope_AD=0;//��ȡ�м���ֵ����һ��
float AD[5]={0},sensor[5]={0},sensor_to_one[5]={0};//��ȡ�ĵ��ֵ,��һ��
float turn_error=0.00,pre_turn_error=0.00;
float e=0.00;
float ec=0.00;//�������仯��
uint32 turn_out=0;//������
uint16 zhidao_flag,wandao_flag,stop_flag;//�������ͱ�־

int Speed_Choose=0;
int Stop_Car=0,TC=0,R_stopcar=0;//ͣ������

/*Բ������*/
int  out;
int  rightcount=0,leftcount=0;
int  right_flag=0,Right_Count=0,Left_Count=0,left_flag=0;
  /*************************************************************************
  *  ��������:  AD_Init
  *  ����˵���� AD��ʼ��
  *  ����˵����
  *  �������أ� ��
  *  �޸�ʱ�䣺
  *  ��    ע��
  *************************************************************************/
void AD_Init(void)
{
  adc_init(AD_Vol);//�ɼ���ѹ
  /*�ұߵ��*/
  adc_init(AD1);
  adc_init(AD2);
  adc_init(AD3);
  adc_init(AD4);
  /*��ߵ��*/
  adc_init(AD5);
  adc_init(AD6);
  adc_init(AD7);
  adc_init(AD8);
}
  /*************************************************************************
  *  ��������   AD_Ave
  *  ����˵���� ���ֵ����
  *  ����˵���� �ɼ� N����ƽ��
  *  �������أ� ��
  *  �޸�ʱ�䣺
  *  ��    ע��
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
  *  ��������   SC_black_Init
  *  ����˵���� ���ֵ����
  *  ����˵����
  *  �������أ� ��
  *  �޸�ʱ�䣺
  *  ��    ע��
  *************************************************************************/
void SC_black_Init(void)
{
  uint16 i,j;
  float  sensor_1=0,sensor_2=0,sensor_3=0,sensor_4=0,sensor_5=0;
  max_v[0] = max_v[1] = max_v[2] = max_v[3]= max_v[4]=0;
  min_v[0] = min_v[1] = min_v[2] = min_v[3]= min_v[4]=5;
  if(8==BOMA)//���벦�����жϣ�������4��
  {
     uint8 C1[8],C2[8];
     sprintf((uint8*)C1,"Collecting");
     LCD_single_P8x16Str(0,0,C1);
     sprintf((uint8*)C2,"Samples...");
     LCD_single_P8x16Str(0,2,C2);
     for(i=0;i<1400;i++)//�ɼ�ִ�е�ʱ��
     {
      AD_value[0] = AD_Ave(AD5, ADC_12bit,20);//ˮƽ��
      AD_value[1] = AD_Ave(AD6, ADC_12bit,20);//��ֱ����
      AD_value[2] = AD_Ave(AD7, ADC_12bit,20);//��ֱ�ҵ��
      AD_value[3] = AD_Ave(AD8, ADC_12bit,20);//ˮƽ��
      AD_value[4] = AD_Ave(AD1, ADC_12bit,20);//�м���

      for(j=0;j<SENSOR_NUMBER;j++)
      {
        if(AD_value[j]>max_v[j])
       {
          max_v[j] = AD_value[j];//ɨ�����ֵ
        }
      }
      pit_delay_ms(PIT3,1);//��ʱ1ms
    }
    flash_erase_sector(SECTOR_AD);       //����254����
    for(i=0; i<SENSOR_NUMBER;i++)                   //��б궨�����ֵд������
      {
         flash_write(SECTOR_AD,i*4,max_v[i]);
      }
  }

  else
  {
    for(i=0;i<4;i++)
     {
       for(j=0;j<SENSOR_NUMBER;j++)   //��ȡ5����еĲ����궨�����ֵ
        {
            max_v[j] = flash_read(SECTOR_AD,j*4,int16);
        }
     }
       uint8 C1[8],C2[8];
       sprintf((uint8*)C1,"Reading");
       LCD_single_P8x16Str(0,0,C1);
       sprintf((uint8*)C2,"Samples...");
       LCD_single_P8x16Str(0,2,C2);       
       pit_delay_ms(PIT3,200);//��ʱ10ms
  }
  OLED_Init(); //OLED��ʼ��
}
 /*************************************************************************
 *  ��������:  AD_Collect
 *  ����˵���� AD�ɼ�
 *  ����˵����
 *  �������أ� ��
 *  �޸�ʱ�䣺
 *  ��    ע��
 *************************************************************************/
void AD_Collect()//AD�ɼ�
{
//    sensor1 =(AD_Ave(ADC0_SE14, ADC_12bit,5)>>2)*0.1;//ˮƽ��
//    sensor2 = (AD_Ave(ADC1_SE15, ADC_12bit,5)>>2)*0.1;//��ֱ����
//    sensor3 = (AD_Ave(ADC0_SE13, ADC_12bit,5)>>2)*0.1;//��ֱ�ҵ��
//    sensor4 =( AD_Ave(ADC0_SE12, ADC_12bit,5)>>2)*0.1;//ˮƽ��
//  sensor4 = AD_Ave(ADC0_DP0, ADC_12bit,5);
//  sensor3 = AD_Ave(ADC0_DM0, ADC_12bit,5);
//  sensor2 = AD_Ave(ADC1_DP0, ADC_12bit,5);
//  sensor1 = AD_Ave(ADC1_DM0, ADC_12bit,5);
      //�ɼ�5��

//       ad_value[0] = AD_Ave(AD5, ADC_12bit,10);//ˮƽ����
//       ad_value[1] = AD_Ave(AD6, ADC_12bit,10); //��ֱ����
//       ad_value[2] = AD_Ave(AD7, ADC_12bit,10);//��ֱ�ҵ�
//       ad_value[3] = AD_Ave(AD8, ADC_12bit,10); //ˮƽ�ҵ��
//       ad_value[4] = AD_Ave(AD1, ADC_12bit,10);//�м���
#if 1
    //�ɼ�5��
    int16 i,j,k,temp;
    for(i = 0; i <5; i++)
     {
       ad_value[0][i] = AD_Ave(AD5, ADC_12bit,20);//ˮƽ����
       ad_value[1][i] = AD_Ave(AD6, ADC_12bit,20); //��ֱ����
       ad_value[2][i] = AD_Ave(AD7, ADC_12bit,20);//��ֱ�ҵ�
       ad_value[3][i] = AD_Ave(AD8, ADC_12bit,20); //ˮƽ�ҵ��
       ad_value[4][i] = AD_Ave(AD1, ADC_12bit,20);//�м���
     }
    /*=========================ð����������==========================*///�������ֵ����Сֵ
     for(i=0;i<SENSOR_NUMBER;i++)//5�����
     {
       for(j=0;j<4;j++)//5����������
       { 
         for(k=0;k<4-j;k++)
         {
           if(ad_value[i][k]>ad_value[i][k+1]) //ǰ��ıȺ���Ĵ�����н���
           {
             temp=ad_value[i][k+1];
             ad_value[i][k+1]=ad_value[i][k];
             ad_value[i][k]= temp;
           }
         }
       }
     }

     /*===========================��ֵ�˲�=================================*/
     for(i=0;i<SENSOR_NUMBER;i++)    //���м�����ĺ�
     {
        ad_sum[i] =ad_value[i][1]+ad_value[i][2]+ad_value[i][3]; //��ȥ�����Сȡ�м�����
        ad_value1[i]=ad_sum[i]/3; //��ƽ��ֵ
     }
/*����ʹ��*/
//     sensor1=ad_value1[0];
//     sensor2=ad_value1[1];
//     sensor3=ad_value1[2];
//     sensor4=ad_value1[3];
//     sensor5=ad_value1[4];//���ֵ1023����Сֵ0

     /*����ƽ���˲�*/
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
      for(i=0;i<5;i++)  //��ƽ��
      {
          AD[i] = AD_sum[i] / SENSOR_NUMBER;
          AD_sum[i] = 0;
      }

     /*����ʹ��*/
//     sensor1=AD[0];
//     sensor2=AD[1];
//     sensor3=AD[2];
//     sensor4=AD[3];
//     sensor5=AD[4];//���ֵ1023����Сֵ0
#endif
}
/*************************************************************************
 *  ��������   AD_Date_analyse
 *  ����˵���� ������ݷ���
 *  ����˵����
 *  �������أ� ��
 *  �޸�ʱ�䣺
 *  ��    ע��
 *************************************************************************/
void AD_Date_analyse()
{
  int16  i;
  AD_Collect();//������ݲɼ�
  /*********************��һ������********************/
  for(i=0;i<SENSOR_NUMBER;i++)
  {
    sensor_to_one[i] = (float)(AD[i] - min_v[i])/(float)(max_v[i] - min_v[i]);
    if(sensor_to_one[i]<=0.0)  sensor_to_one[i]=0.001;
    if(sensor_to_one[i]>1.0)   sensor_to_one[i]=1.0;
    sensor[i] = 100.00 * sensor_to_one[i];     //sensor[i]Ϊ��һ�����ֵ,��ΧΪ0-100.00
   }

    sensor[0] = (sensor[0] < 1? 1:sensor[0]);	//���ֵ�޷�
    sensor[1] = (sensor[1] < 1? 1:sensor[1]);
    sensor[2] = (sensor[2] < 1? 1:sensor[2]);
    sensor[3] = (sensor[3] < 1? 1:sensor[3]);
  
    sensor1=sensor[0];
    sensor2=sensor[1];
    sensor3=sensor[2];
    sensor4=sensor[3];
   /*******�м��������һ���������µ����********/
   sensor_middle = (float)(ad_value1[4] - min_v[4])/(float)(max_v[4] - min_v[4]);
   if(sensor_middle <= 0.0)  sensor_middle = 0.001;
   Slope_AD = 100.00 * sensor_middle;
   sensor5=Slope_AD;//�µ������ֵ    
}

/********************������ʽ*****************/
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
 *  ��������   Run_Control  
 *  ����˵���� ����״̬
 *  ����˵����
 *  �������أ� ��
 *  �޸�ʱ�䣺
 *  ��    ע��
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
    case normal_Run://����������
        Run_Flag = 1;
        Run_state = Wait_Ring;
        if(1==Stop_Car)  Run_state = Stop;//�����������,ͣ�����sensor1==1&&sensor2==1&&sensor3==1&&sensor4==1||
     break;
    
    case Wait_Ring://�ȴ�����
        Run_Flag = 1;
        Run_state = Running;
        if(1==Stop_Car)  Run_state = Stop;//�����������,ͣ�����sensor1==1&&sensor2==1&&sensor3==1&&sensor4==1||
     break;
    
    case Running://������
        Run_Flag = 1;
//        Ring_Control();//�������
        if(1==Stop_Car)  Run_state = Stop;//�����������,ͣ�����sensor1==1&&sensor2==1&&sensor3==1&&sensor4==1||
     break;
    
    case Stop://ֹͣ
        {
          Run_Flag = 0;
        }
     break;		
	
    default :break;    
  }
    FTM_PWM_Duty(FTM1, FTM_CH0, turn_out_cal());//ת�����  
}
/*************************************************************************
 *  ��������   Ring_Control  
 *  ����˵���� �������
 *  ����˵����
 *  �������أ� ��
 *  �޸�ʱ�䣺
 *  ��    ע��
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
    case No_Ring://��һ�����ж��Ƿ���ֻ�
      turn_error=100.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
//      if(sensor2>70&&50<sensor3&&sensor3<80&&sensor1>80&&sensor4>80)//(�һ�)
      if(sensor2>20&&30<sensor3&&sensor3<80&&sensor1>80&&sensor4>80)//(�һ�)      
      {
        Ring_state  = Find_Right_Ring;
      }
      else if(sensor3>20&&30<sensor2&&sensor2<80&&sensor1>80&&sensor4>80)//(��)
//      else if(sensor3>20&&30<sensor2&&sensor2<80&&sensor1>80&&sensor4>80)//(��)
      {
        Ring_state  = Find_Left_Ring;
      }
      else
        Ring_state  = No_Ring;
    break;
      
    case Find_Right_Ring://�ڶ������жϽ�����(�һ�)
      turn_error=100.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
//      if(sensor2>70&&sensor3>70)
      if(sensor2>20&&sensor3>40)
      {
        Ring_state  = Ready_Right_Ring;
      }
    break;
      
    case Find_Left_Ring://�ڶ������жϽ�����(��)
      turn_error=100.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
      if(sensor2>40&&sensor3>20)
      {
        Ring_state  = Ready_Left_Ring;
      }
    break;  
    
    case Ready_Right_Ring://��������׼�������� ���һ���
      turn_error=200.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
//      if(30<sensor2&&sensor2<50&&sensor3>80)
      if(20<sensor2&&sensor2<40&&sensor3>80)
      {
        Go_Ring_Flag=1;//�������ٱ�־
        Ring_state  = Start_Right_Ring;
      }
    break;
    
    case Ready_Left_Ring://��������׼��������  ���󻷣�
      turn_error=200.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
      if(20<sensor3&&sensor3<40&&sensor2>80)
      {
        Go_Ring_Flag=1;//�������ٱ�־
        Ring_state  = Start_Left_Ring;
      }
    break; 
    
    case Start_Right_Ring://���Ĳ�����ʼ�������Թ̶�ƫ����һ�ξ���(�һ�)
      turn_error=200.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
      if(sensor3>86&&sensor2<32)
      {
        if(1==BOMA)//�ٶ�260
        {
            FTM_PWM_Duty(FTM1, FTM_CH0, 1010);//������Ҵ���,�һ�ǰһ����ֵ��Ǵ���
            pit_delay_ms(PIT3,340);//��ʱ                
        }
        else if(2==BOMA)//�ٶ�270
        {
            FTM_PWM_Duty(FTM1, FTM_CH0, 1010);//������Ҵ���,�һ�ǰһ����ֵ��Ǵ���
            pit_delay_ms(PIT3,340);//��ʱ                
        } 
        else if(4==BOMA)//�ٶ�280
        {
            FTM_PWM_Duty(FTM1, FTM_CH0, 1010);//������Ҵ���,�һ�ǰһ����ֵ��Ǵ���
            pit_delay_ms(PIT3,340);//��ʱ                
        }
        else//�ٶ�250��Ĭ�ϣ�
        {
            FTM_PWM_Duty(FTM1, FTM_CH0, 1008);//������Ҵ���,�һ�ǰһ����ֵ��Ǵ���
            pit_delay_ms(PIT3,300);//��ʱ                
        }
        if(sensor1<60&&sensor2<10&&sensor3>80&&sensor4>80)
        {
           Out_Ring_Flag=1;//�������ٱ�־
           Ring_state = In_Ring;
        }
      }
    break;

    case Start_Left_Ring://���Ĳ�����ʼ�������Թ̶�ƫ����һ�ξ��루�󻷣�
      turn_error=300.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
      if(sensor2>87&&sensor3<32)
      {
        if(1==BOMA)//�ٶ�260
        {
            FTM_PWM_Duty(FTM1, FTM_CH0, 1160);//������Ҵ���,�һ�ǰһ����ֵ��Ǵ���
            pit_delay_ms(PIT3,340);//��ʱ                
        }
        else if(2==BOMA)//�ٶ�270
        {
            FTM_PWM_Duty(FTM1, FTM_CH0, 1160);//������Ҵ���,�һ�ǰһ����ֵ��Ǵ���
            pit_delay_ms(PIT3,340);//��ʱ                
        } 
        else if(4==BOMA)//�ٶ�280
        {
            FTM_PWM_Duty(FTM1, FTM_CH0, 1160);//������Ҵ���,�һ�ǰһ����ֵ��Ǵ���
            pit_delay_ms(PIT3,340);//��ʱ                
        }
        else//�ٶ�250��Ĭ�ϣ�
        {
            FTM_PWM_Duty(FTM1, FTM_CH0, 1160);//������Ҵ���,�һ�ǰһ����ֵ��Ǵ���
            pit_delay_ms(PIT3,335);//��ʱ                
        }
        if(sensor1<60&&sensor2<10&&sensor3>80&&sensor4>80)
        {
           Out_Ring_Flag=1;//�������ٱ�־
           Ring_state = In_Ring;
        }
        
//        if(sensor1>80&&sensor2>80&&sensor3<10&&sensor4<80)
//        {
////          BEEP_ON;//������
//          Ring_state = In_Ring;
//        }
      }
    break;
      
    case In_Ring://���岽��������
      turn_error=300.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
//      if(sensor1>40&&sensor2<20)
      if((sensor2>20&&20<sensor3&&sensor3<70)||(sensor3>20&&20<sensor2&&sensor2<70))
      {      
        Ring_state = Out_Ring;
      }
    break;
    
    case Out_Ring://������������  ���ߵĵ��һ������ּ���ֵ
      turn_error=400.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
      if(sensor1>20&&sensor4>80||sensor2>80||sensor2>0&&sensor3>80||sensor1>80&&sensor4>20)
      {
        Ring_state  = Real_Out_Ring;
      }
    break;
    
    case Real_Out_Ring://���߲�������  �ĸ����ֵ����С��  �ص��޻�״̬
      turn_error=400.0*((sensor1+sensor2)-(sensor4+sensor3))/(sensor1+sensor4+sensor2+sensor3);     
      if(abs(sensor1-sensor2)<40 && (sensor3-sensor4)<40) //��������
      {
        Ring_state  = No_Ring;
      }
      break;
  default:break;		
  }
}
/********************�������(λ��ʽPD)*****************/
int16 turn_out_cal()//�������(λ��ʽPD);float kp,float kd
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
     /*һ�����ٶ�260*/     
    case 1:
      Speed_Choose=1;
      if(abs(turn_error)<=10)//ֱ��
      {
        float kp,kd;
        kp=1.0;
        kd=2.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=2;        
      }     
      else if(abs(turn_error)<=20)//ֱ��
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
      else if(abs(turn_error)<=50)//Сs��
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
    /*�������ٶ�270*/ 
      
    case 2:
      Speed_Choose=2;      
      if(abs(turn_error)<=10)//ֱ��
      {
        float kp,kd;
        kp=1.0;
        kd=2.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=2;        
      }     
      else if(abs(turn_error)<=20)//ֱ��
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
      else if(abs(turn_error)<=50)//Сs��
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
      
     /*�������ٶ�280*/        
    case 4:
      Speed_Choose=3;      
      if(abs(turn_error)<=10)//ֱ��
      {
        float kp,kd;
        kp=1.0;
        kd=2.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=2;        
      }     
      else if(abs(turn_error)<=20)//ֱ��
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
      else if(abs(turn_error)<=50)//Сs��
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
      
     /*Ĭ�ϣ��ٶ�250*/        
    default:
      if(abs(turn_error)<=10)//ֱ��
      {
        float kp,kd;
        kp=1.0;
        kd=2.00;
        turn_out = DirectMiddle + (float)kp*turn_error+(turn_error-pre_turn_error)*(float)kd;
        Speed_Flag=2;        
      }     
      else if(abs(turn_error)<=20)//ֱ��
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
      else if(abs(turn_error)<=50)//Сs��
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
      e = turn_error;//��ǰƫ��        
      ec= turn_error-pre_turn_error;//ƫ��仯�ʣ���ǰƫ����ϴ�ƫ��Ĳ�ֵ��    
      pre_turn_error=turn_error;     
      if(turn_out<=DirectRight) turn_out = DirectRight;//�Զ����ǽ����޷�
      else if(turn_out>=DirectLeft) turn_out = DirectLeft;
//      printf("turn_out=%4d\n",turn_out);
      return turn_out;      
}


/******��������ж�*****/
void turn_control()
{
#if 0
    /*�ǻ���ִ��*/
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
       if(gpio_get(PTB22)==0)//��һ�ι�����(��һ���)  
//       if(gpio_get(PTC13)==0)//��һ�ι�����(�ڶ����)
       {
          R_stopcar=1;
       }
       if(gpio_get(PTB22)==1&&R_stopcar==1) //(��һ���)        
//       if(gpio_get(PTC13)==1&&R_stopcar==1) //(�ڶ����) 
       {
          R_stopcar=0;
          TC=1;
       }
       if(gpio_get(PTB22)==0&&TC==1)//�ڶ��ι�����,ͣ��       
//       if(gpio_get(PTC13)==0&&TC==1)//�ڶ��ι�����,ͣ��
       {  
         Stop_Car=1;
       }  
}