#include "common.h"
#include "Init.h"
#include "Public.h"
#include "Huan.h"

extern int World_X[60][80];
extern unsigned char ZD_len[60];
extern unsigned char Mid_cor[60];
extern uint16_t SteerPidCal(float excursion);
extern int fabss(int n);
/***************************************************************************/
float LOOP_Error2=0.0;
int LOOP_Error=0;

float Get_LOOP_Error2(unsigned char start,unsigned char end,float midpos)
{
	unsigned char i=0;
	int  Black_Sum=0;
	int weightSum = 0;
        int X,Y;
	float TemError = 0.0;
        for(i =30; i <=59; i++) 
	{	
                Y=i;
                X=midLine[i];
                if(midLine[i]!=255) 
		{
                   Black_Sum += World_X[Y][X];//*LineWeight[i];
		   weightSum += 1;//LineWeight[i];
                 //  printf("i=%d,x=%d\n",i,World_X[Y][X]);
                }

	}
	TemError = Black_Sum*1.0/weightSum - midpos;
	
	return TemError;
}

/*************************************�������************************************/
void CommonRectificate(int16 data[],unsigned char begin,unsigned char end)
{
	unsigned char MidPos = 0;
	if (end > CAMERA_H-1)  //��λ
	{
		end = CAMERA_H-1;
	}
	if (begin == end)
	{
		data[begin] = (data[begin-1]+data[begin+1])/2;
		midLine[begin] =  leftLine[begin] + (rightLine[begin]-leftLine[begin])/2;
	}
	else if(begin < end)
	{
		MidPos = (begin+end)/2;	

		data[MidPos] = (data[begin]+data[end])/2;  //���ң����м��λ��

		midLine[MidPos] =  leftLine[MidPos] + (rightLine[MidPos]-leftLine[MidPos])/2;	//�����м��λ��
	
		if (begin+1 < MidPos)
		{
			CommonRectificate(data,begin,MidPos);
		}
		if (MidPos+1 < end)
		{
			CommonRectificate(data,MidPos,end);
		}
	}
}
////////
void CommonRectificate_2(int16 data[],unsigned char begin,unsigned char end)
{
	unsigned char MidPos = 0;
	if (end > CAMERA_H-1)  //��λ
	{
		end = CAMERA_H-1;
	}
	if (begin == end)
	{
		data[begin] = (data[begin-1]+data[begin+1])/2;
		//midLine[begin] =  leftLine[begin] + (rightLine[begin]-leftLine[begin])/2;
	}
	else if(begin < end)
	{
		MidPos = (begin+end)/2;	

		data[MidPos] = (data[begin]+data[end])/2;  //���ң����м��λ��

		//midLine[MidPos] =  leftLine[MidPos] + (rightLine[MidPos]-leftLine[MidPos])/2;	//�����м��λ��
	
		if (begin+1 < MidPos)
		{
			CommonRectificate(data,begin,MidPos);
		}
		if (MidPos+1 < end)
		{
			CommonRectificate(data,MidPos,end);
		}
	}
}
/*************************Բ���ж�***********************************/
//��Բ����־
int Loop_Flag_1=0;//1��2��
int RLoop_Flag_1=0;
int LLoop_Flag_1=0;
int RLoop_Flag_2=0;
int LLoop_Flag_2=0;

unsigned char Huan_Max=0; //Բ���м��

unsigned char Ricount=0;
unsigned char Licount=0;
unsigned char Ristart=0;
unsigned char Listart=0;
unsigned char Riend=0;
unsigned char Liend=0;
//����ֱ���м���
signed char Var_R=0;
signed char Var_L=0;
float Aver_R=0;
float Aver_L=0;
unsigned char R_count=0;
unsigned char L_count=0;
uint8 Loop_Judge(void)
{ 
    short i;
    //����ȫ�׼���
    Ricount=0;
    Licount=0;
    Ristart=0;
    Listart=0;
    Riend=0;
    Liend=0;
    //����ֱ���м���
    Var_R=0;
    Var_L=0;
    Aver_R=0;
    Aver_L=0;
    R_count=0;
    L_count=0;
    
    Loop_Flag_1=0;
    RLoop_Flag_1=0;
    LLoop_Flag_1=0;
    RLoop_Flag_2=0;
    LLoop_Flag_2=0;
    
    if(MidNumbers>=49)//��֤��Ұ������������߶���
    {
        for(i=CAMERA_H-1;i>=CAMERA_H-MidNumbers&&i>10;i--)//���������ȫ����
        {
             if(leftLine[i]==0)
             {   
                 Listart=i;
                 while(leftLine[i]<=1)  
                 {
                    Licount++;
                    Liend=i;
                    i--;
                 }
                 break;
             }
        }
        for(i=CAMERA_H-1;i>=CAMERA_H-MidNumbers&&i>10;i--)//���ұ�����ȫ����
        {
             if(rightLine[i]==79)
             {  
                 Ristart=i;
                 while(rightLine[i]>=78)
                 {
                     Ricount++;
                     Riend=i;  
                     i--;
                 }
                 break;
             }
        }
    }
    else
       Loop_Flag_1=0;
    
    if(Licount>=20&&Licount<=35)//�˵����߶��߹���
    {
         for(i=CAMERA_H-1;i>=CAMERA_H-MidNumbers&&i>10;i--)//������Ұ
         {
             if((rightLine[i]>=rightLine[i-1])&&(rightLine[i]-rightLine[i-1])<=2&&rightLine[i]!=0) 
             {   
                   Var_R=Var_R+(rightLine[i]- rightLine[i-1]);//�ұ�Ϊֱ��
                   R_count++;
             }
             else
                  R_count=0;
         }
         Aver_R=Var_R*1.0/R_count;
         
         if(R_count>=45&&Aver_R<=1.0)
            LLoop_Flag_1=1;
         else 
            LLoop_Flag_1=0;
    }
    else if(Ricount>=20&&Ricount<=35)//�˵����߶��߹���
    {   
         for(i=CAMERA_H-1;i>=CAMERA_H-MidNumbers&&i>10;i--)//������Ұ
         {
             if((leftLine[i-1]>=leftLine[i])&&(leftLine[i-1]-leftLine[i])<=2&&leftLine[i]!=0)
             {  
                 Var_L=Var_L+(leftLine[i-1]-leftLine[i]);//���Ϊֱ��
                 L_count++;
             }  
             else 
                 L_count=0;
         }
         Aver_L=Var_L*1.0/L_count;
         
         if(L_count>=45&&Aver_L<=1)
           RLoop_Flag_1=1;
         else 
           RLoop_Flag_1=0;
    }
    else
      Loop_Flag_1=0;
    //�ڲ�С��
    int Huan_Start=0;//���ο�ʼ
    int Huan_End=0;//���ν���
    int Huan_up_count=0;//�ϰ벿�ּ���
    int Huan_down_count=0;//�°벿�ּ���
    int Huan_Var_D_up=0;//�°�Բ�ϰ벿��ƫ���
    int Huan_Var_D_down=0;//�°�Բ�°벿��ƫ���
    float Huan_aver_D_up=0.0;//�°�Բ�ϰ벿��ƽ��ֵ
    float Huan_aver_D_down=0.0;//�°�Բ�°벿��ƽ��ֵ
    int Mid=0;//�°벿�ּ����е�
    
    if(LLoop_Flag_1)
    {
          Huan_Start= Liend;//�ж�ΪС����������С����ֹȫ�׺����ֱ��
          if((leftLine[Huan_Start-1]-leftLine[Huan_Start]<=12)&&(leftLine[Huan_Start-2]-leftLine[Huan_Start-1]<=9))
          {
                  i=Huan_Start-1;
                  while(leftLine[i]>=leftLine[i+1])//Ҫ������
                  {
                      Huan_Max=i;
                      Huan_down_count++;   
                      i--;
                  }
                  //�°�Բ����������ƫ��ƽ��ֵ
                  Mid=(int)(Huan_down_count/2);
                  //down
                  for(i=Huan_Start-2;i>=(Huan_Max+Mid);i--)
                  {
                       Huan_Var_D_down=Huan_Var_D_down+(leftLine[i]-leftLine[i+1]);   
                  }
               
                  Huan_aver_D_down=Huan_Var_D_down*1.0/(Huan_Start-1-(Huan_Max+Mid));
                  //up
                  for(i=(Huan_Max+Mid-1);i>=Huan_Max;i--)
                  {
                      Huan_Var_D_up=Huan_Var_D_up+(leftLine[i]-leftLine[i+1]);
                  }
                  
                  Huan_aver_D_up=Huan_Var_D_up*1.0/((Huan_Max+Mid)-Huan_Max);
                  
                  //�ϰ벿��Բ
                  if(leftLine[Huan_Max]==leftLine[Huan_Max+1]==leftLine[Huan_Max+2]==leftLine[Huan_Max+3])
                    Huan_Max=Huan_Max+3;
                  else if(leftLine[Huan_Max]==leftLine[Huan_Max+1]==leftLine[Huan_Max+2])
                    Huan_Max=Huan_Max+2;
                  else if(leftLine[Huan_Max]==leftLine[Huan_Max+1])
                    Huan_Max=Huan_Max+1;
          }
          i=Huan_Max;
          while((leftLine[i]>=leftLine[i-1])&&leftLine[i]!=0) 
          { 
            Huan_up_count++;
            i--; 
          }
          if(Huan_aver_D_down>Huan_aver_D_up&&Huan_aver_D_down>=1.2&&Huan_down_count<=25&&Huan_up_count>=3)
          {
            LLoop_Flag_2=1;
            Loop_Flag_1=1;
          }
          else
            Loop_Flag_1=0;
    }
    else if(RLoop_Flag_1)
    {
          Huan_Start= Riend;
          if((rightLine[Huan_Start]-rightLine[Huan_Start-1]<=12)&&(rightLine[Huan_Start-1]-rightLine[Huan_Start-2]<=9))
          {
                  i=Huan_Start-1;
                  while(rightLine[i]<=rightLine[i+1])
                  {
                      Huan_Max=i;
                      Huan_down_count++;
                      i--;
                  }
                  Mid=Huan_down_count/2;
                  for(i=Huan_Start-2;i>=(Huan_Max+Mid);i--)
                  {
                       Huan_Var_D_down=Huan_Var_D_down+(rightLine[i+1]-rightLine[i]);   
                  }
                  Huan_aver_D_down=Huan_Var_D_down*1.0/(Huan_Start-1-(Huan_Max+Mid));
                  
                  for(i=(Huan_Max+Mid-1);i>=Huan_Max;i--)
                  {
                      Huan_Var_D_up=Huan_Var_D_up+(rightLine[i+1]-rightLine[i]);
                  }
                  Huan_aver_D_up=Huan_Var_D_up*1.0/((Huan_Max+Mid)-Huan_Max);
                  
                  if(rightLine[Huan_Max]==rightLine[Huan_Max+1]==rightLine[Huan_Max+2]==rightLine[Huan_Max+3])
                    Huan_Max=Huan_Max+3;
                  else if(rightLine[Huan_Max]==rightLine[Huan_Max+1]==rightLine[Huan_Max+2])
                    Huan_Max=Huan_Max+2;
                  else if(rightLine[Huan_Max]==rightLine[Huan_Max+1])
                    Huan_Max=Huan_Max+1;
          }
          i=Huan_Max;
          while((rightLine[i]<=rightLine[i-1])&&rightLine[i]!=0)
          {
               Huan_up_count++;
               i--; 
          }     
          if(Huan_aver_D_down>Huan_aver_D_up&&Huan_aver_D_down>=1.2&&Huan_down_count<=25&&Huan_up_count>=3)
          {
              RLoop_Flag_2=1;
              Loop_Flag_1=2;
          }
          else
            Loop_Flag_1=0;
    }
    else
      Loop_Flag_1=0;
    if(BOMA==4)
    {         uint8 S1[8],S2[8],S3[8],S4[8],S5[8],S6[8],S7[8],S8[8],S9[8];   
             
               sprintf((char*)S1,"%d",(int)LLoop_Flag_1);
               LCD_single_P8x16Str(1,0,S1);
  
               sprintf((char*)S2,"%d",(int)RLoop_Flag_1);
               LCD_single_P8x16Str(1,2,S2);
  
               sprintf((char*)S3,"%f",(float)Huan_aver_D_up);
               LCD_single_P8x16Str(1,4,S3);
               
               sprintf((char*)S4,"%f",(float)Huan_aver_D_down);
               LCD_single_P8x16Str(1,6,S4);
               
               sprintf((char*)S5,"%d",(int)Liend);
               LCD_single_P8x16Str(24,0,S5);
               
               sprintf((char*)S6,"%d",(int)Riend);
               LCD_single_P8x16Str(24,2,S6);
               
               sprintf((char*)S7,"%d",(int)Loop_Flag_1);
               LCD_single_P8x16Str(96,4,S7);
               
               sprintf((char*)S8,"%d",(int)Huan_down_count);
               LCD_single_P8x16Str(56,2,S8);
               
               sprintf((char*)S9,"%d",(int)Huan_up_count);
               LCD_single_P8x16Str(56,0,S9);
    }
    return Loop_Flag_1;  
}

/***************************Բ������**********************************/
int Loop_f_R_count=0;//����־����
int Loop_f_L_count=0;
int Loop_C=0;//������
unsigned char Corner_L=0;//��һ�β��ߣ��·��ĵ�
unsigned char Corner_R=0;
int White_up_count=0;//�ڲ�С������ȫ����
int White_down_count=0;
int down_line=0;//�ڻ��·���ʼ��
int up_line=0;//�ڻ��Ϸ���ֹ��
int Black_Huan_count_U=0,Black_Huan_count_D=0;//�ڻ����²��м���
unsigned char Huan_Mid=0;//�ڻ��е�
unsigned char Top_Point=0;//�ڶ��β����Ϸ������
int Huan_Black_R=0;//��������
int Huan_Black_L=0;
int Huan_White_R=0;//�����߽�(����ȫ��)�ж�
int Huan_White_L=0;

int Loop_Flag_2=0;//�ڶ����뻷��־
int Loop_in_Start=0;//��ʼ����
int Loop_in=0;//���뻷
//int little_F=0;//С����־
//int little_H=0;//С��
unsigned char D_W_F=0;//ʶ�𵽻������ȫ�׻�ڱ�־
unsigned char little_N=0;//С����ʧ��־
unsigned char little_Y=0;//�·�ֻ��С����־

int16 H_leftLine[CAMERA_H]; //����������
int16 H_rightLine[CAMERA_H]; 
void Loop_process1()
{
    short i=0;
    int sub_M=0;
    Corner_L=0;
    Corner_R=0;
    White_down_count=0;
    White_up_count=0;
    down_line=0;
    up_line=0;
    Black_Huan_count_U=0;
    Black_Huan_count_D=0;
    Huan_Mid=0;
    Top_Point=0;
    Huan_Black_R=0;
    Huan_Black_L=0;
    Huan_White_R=0;
    Huan_White_L=0;

    //����ֱ���м���
    Var_R=0;
    Var_L=0;
    Aver_R=0;
    Aver_L=0;
    R_count=0;
    L_count=0;
    
    for(i=0;i<=59;i++)//��ʼ������
    {
         H_leftLine[i]=leftLine[i];
         H_rightLine[i]=rightLine[i];
    }
    
    Loop_C++;//���뻷����
      
    if(Loop_Flag_1==1) //1  ��
    {   
        if(Loop_C==1)//��һ�β���
        {
              Corner_L= Listart+1;
              for(i=Huan_Max;i<=Corner_L;i++)
              {
                   midLine[i] = rightLine[i]-Mid_cor[i];
              }
             // CommonRectificate(&leftLine[0],Huan_Max,Corner_L);
        }
        else if(Loop_C>1&&Loop_Flag_2==0)//��һ�β���
        {     
            // sub_M = leftLine[Huan_Mid]-0;  //��Ұ��Χ�����������ڻ��ж�
             
             if(D_W_F==0)//�Ƿ��·�ȫ��
             {
                    if(leftLine[CAMERA_H-1]==0)
                    {
                        i=CAMERA_H-1;
                        while(leftLine[i]==0)//�²�
                        {
                                White_down_count++; 
                                 i--;
                        }
                    }
                    else
                       D_W_F=0;
                    if(White_down_count>10)
                       D_W_F=1;
                    else
                       D_W_F=0;
             }
          //   printf("%d\n",D_W_F);
             White_down_count=0;
             if(D_W_F==1)//�����·�ȫ��
             {                     
                     Corner_L=CAMERA_H-1;
                     //�ж��·��Ƿ�ȫ����
                     if( little_Y==0)  
                     {
                           i=CAMERA_H-1;
                           while(leftLine[i]==0)
                           {
                               White_down_count++;
                               down_line=i-1;
                               i--;
                           }
                     }
                     //�а��У�������
                     if(White_down_count>0&&little_Y==0)
                     { 
                        BEEP_OFF;
                          little_Y=0;
                          i= down_line;
                          while(leftLine[i]!=0)//�²�
                          {
                               if(leftLine[i]<=leftLine[i-1])
                               {
                                   // Black_Huan_count_D++;   
                                    Huan_Mid=i;
                                    i--;
                               }
                               else 
                                 break;                   
                          }
                          for(i=Huan_Mid;i<=Corner_L;i++)
                          {
                               midLine[i] = rightLine[i]-Mid_cor[i];
                          }
                          //CommonRectificate(&leftLine[0],Huan_Mid,Corner_L);
                     }
                     //�ް���
                     else if(little_Y==1||White_down_count==0)
                     {  
                          BEEP_ON; 
                          little_Y=1;
                          //�ж��·���Ұ��С���Ƿ���ʧ
                          if(little_N==0)
                          {
                                 i=CAMERA_H-1;
                                 while(leftLine[i]!=0)//�ϲ�
                                {
                                          Black_Huan_count_U++;
                                          up_line=i;
                                          i--;             
                                }
                          }
                          //��Ұ����С������һ�β���
                          if(Black_Huan_count_U>0 && little_N==0)
                          {    
                                little_N=0;
                                for(i=up_line;i<=Corner_L;i++)
                                {
                                     midLine[i] = rightLine[i]-Mid_cor[i];
                                }
                               // CommonRectificate(&leftLine[0],Huan_Mid,Corner_L);
                          }
                          //��Ұ����С�����ڶ��β���
                          else if(little_N==1||Black_Huan_count_U==0)
                          {   
                                      little_N=1;
                                      Huan_Mid=CAMERA_H-1;
                                      gpio_init(PTA19,GPO,1);
                                      //Ѱ���Ϸ�����
                                      i=CAMERA_H-1;
                                      while(leftLine[i]<=1)
                                      {   
                                           White_up_count++;
                                           i--;
                                           Top_Point=i;
                                      }
                                      i=Top_Point;
                                      //����
                                      rightLine[Top_Point]=leftLine[Top_Point];
                                      CommonRectificate_2(&rightLine[0],Top_Point,Huan_Mid); 
                                      for(i=Top_Point;i<=Huan_Mid;i++)
                                      {
                                           midLine[i]=rightLine[i] - Mid_cor[i];
                                      }
                                      for(int a=Huan_Mid;a<=59;a++)
                                           midLine[a]=midLine[Huan_Mid-3];
                                      
                                       Loop_Flag_2=1;
                          }
                     }
             }
             else//���½�����
             {  
                     i=CAMERA_H-1;
                     while(leftLine[i]>0)
                     {
                         Corner_L=i;
                         i--;
                     }
                     i=Corner_L-1;
                     while(leftLine[i]<=1)
                     {
                         White_down_count++;
                         down_line=i-1;
                         i--;
                     }
                      i= down_line;
                      while(leftLine[i]!=0)//�²�
                      {
                           if(leftLine[i]<=leftLine[i-1])
                           {
                               // Black_Huan_count_D++;   
                                Huan_Mid=i;
                                i--;
                           }
                           else 
                             break;                   
                      }
                      for(i=Huan_Max;i<=Corner_L;i++)
                      {
                           midLine[i] = rightLine[i]-Mid_cor[i];
                      }
                      //CommonRectificate(&leftLine[0],Huan_Mid,Corner_L);                     
             }
        }
        //����ڶ��β���
        else if(Loop_Flag_2==1)//&&Loop_in==0)
        {   
            //��һ�α�־����
            D_W_F=0;           
            little_N=0;
            little_Y=0;
            //�жϽ����ֽ��  
            if(Loop_in_Start==0)
            {
                  for(i=CAMERA_H-1;i>=CAMERA_H-MidNumbers;i--)
                  {
                        if(leftLine[i]==0)  
                          Huan_White_L++;
                        if(rightLine[i]==79)
                          Huan_White_R++;    
                  }
            }

            if((Huan_White_R==Huan_White_L) && (Huan_White_L!=0||Huan_White_R!=0))
            {
                  Loop_in_Start=1;
                  gpio_init(PTC18,GPO,1);
            }
            //δ���ֽ��
            if(Loop_in_Start==0)
            {
                  BEEP_OFF;
                  gpio_init(PTA19,GPO,0);
                  Huan_Mid=CAMERA_H-1;
                  //�������
                  i=CAMERA_H-1;
                  while(leftLine[i]<=1)
                  {   
                       White_up_count++;
                       i--;
                       Top_Point=i;
                  }
                  //����
                  rightLine[Top_Point]=leftLine[Top_Point];
                  CommonRectificate_2(&rightLine[0],Top_Point,Huan_Mid); 
                  for(i=Top_Point;i<=Huan_Mid;i++)
                  {
                       midLine[i]=rightLine[i] - Mid_cor[i];
                  }
                  for(int a=Huan_Mid;a<=59;a++)
                       midLine[a]=midLine[Huan_Mid-3];
            }
            //���ֽ��
            else
            {     
                  BEEP_ON;
                   gpio_init(PTA19,GPO,0);
                  //ɨ�軷��Ϊ����
                  int right_flag = 0;
                  int num=0;//�ܼ���
                  int find_c=0;//�ҵ��߼���
                  unsigned char first_f=0;//�ҵ������б�־
                  for(i=CAMERA_H-1;i>=4;i--)
                  {
                          right_flag = 0;
                          if(ImageData[i][0]==Black&&ImageData[i][1]==Black)
                                        break;
                          for(int temp = 0; temp < 77; temp++)  
                          {  
                                // Ѱ���Һ���  
                                if(right_flag == 0   
                                && ImageData[i][temp] == White 
                                && ImageData[i][temp+1] == White  
                                && ImageData[i][temp+2] == Black)  
                                {  
                                    rightLine[i] = temp+2;  
                                    right_flag = 1; 
                                }  
                                if(right_flag== 1)                            
                                     break;      
                          } 
                          if(right_flag==0)
                             rightLine[i] = 79; ;
                          if(first_f==0)
                            if(right_flag==1)
                            {
                               first_f=1;
                               find_c=i;
                            }
                          num++;
                  }
                  //������
                  for(i=CAMERA_H-1;num!=0;i--)
                  {
                        gpio_init(PTA19,GPO,1);
                        if(rightLine[i] != 79)
                             midLine[i]=rightLine[i] - Mid_cor[i];                        
                        num--;
                        if(midLine[i]<=0)
                        {
                             midLine[i]=255;
                             break;
                        }
                  }
                  //������
                  if(find_c!=CAMERA_H-1)
                  {
                        CommonRectificate_2(&rightLine[0],find_c,59); 
                        for(i=find_c;i<=CAMERA_H-1;i++)
                        {
                             midLine[i]=rightLine[i] - Mid_cor[i];
                        } 
                  }
                /*  else
                    Loop_in=1;*/
            }    
       }
       //��ȫ���뻷
     /*  else if(Loop_in==1)
       {
         
        /*    for(i=CAMERA_H-1;i>=CAMERA_H-MidNumbers;i--)
            {
                 if()
            }*/
            //������
        /*    for(i=CAMERA_H-1;i>=CAMERA_H-MidNumbers;i--)
            {
                  gpio_init(PTA19,GPO,1);
                       midLine[i]=rightLine[i] - Mid_cor[i];                        
                  if(midLine[i]<=0)
                  {
                       midLine[i]=255;
                       break;
                  }
            }
       
       }*/
    }
    else if(Loop_Flag_1==2)//2��
    {                      
        if(Loop_C==1)//��һ�β���
        {
              Corner_R= Ristart+1; 
              CommonRectificate(&rightLine[0],Huan_Max,Corner_R);
        }
        else if(Loop_C>1&&Loop_Flag_2==0)
        {                                                                     
             for(i=CAMERA_H-1;i>25;i--)//Ѱ���·�ȫ��
             {                         //��һ�β����ҵ�
                 if(rightLine[i]==79)
                 {   
                     Corner_R=i+1;
                     while(rightLine[i]==79)
                     {
                         White_down_count++;
                         down_line=i-1;
                         i--;
                     }
                     break; 
                 }
             }
             //�ڻ�������
              i=down_line;
              while(rightLine[i]!=0&&i>5)
              {
               //    Black_Huan_count++; 
                   if(rightLine[i]>=rightLine[i-1])
                   {
                        Huan_Mid=i-1;
                   }
                   up_line=i;
                   i--;
              }
         //     printf("Black_Huan_count=%d,Huan_Mid=%d\n",Black_Huan_count,Huan_Mid);
              sub_M = 79-rightLine[Huan_Mid];
              //��һ�β���
              if(White_down_count>=7&&sub_M>=3)
              {      
                    CommonRectificate(&rightLine[0],Huan_Mid,Corner_R);
              } 
              //�ڶ��β���
              else if(White_down_count<7||sub_M<3)
              {     
                    i=up_line-1;//Ѱ���Ϸ�����
                    while(rightLine[i]==0)
                    {   
                         White_up_count++;
                         i--;
                         Top_Point=i;
                    }
                    leftLine[Top_Point]=rightLine[Top_Point];
                    CommonRectificate(&leftLine[0],Top_Point,Huan_Mid); 
                    Loop_Flag_2=1;
              }
        }
        else if(Loop_Flag_2==1)//����ڶ��β���
        {  
            //�����ұ߽��ж�
            Huan_White_L=0;
            White_down_count=0;
            
            for(i=CAMERA_H-1;i>=CAMERA_H-MidNumbers;i--)
            {
                   if(leftLine[i]==0)
                   {
                        while(leftLine[i]==0)
                        {
                             Huan_White_L++;
                             i--;
                        }
                        break;
                   }      
            }
            for(i=CAMERA_H-1;i>=CAMERA_H-MidNumbers;i--)
            {
                   if(rightLine[i]==0)
                   {
                        while(rightLine[i]==0)
                        {
                             White_down_count++;
                             i--;
                        }
                        break;
                   }      
            }
            if(Huan_White_L==White_down_count==MidNumbers)
            {       
                    Top_Point=CAMERA_H-MidNumbers;
                    Huan_Mid=CAMERA_H-1;
                    leftLine[Top_Point]=40;
                    CommonRectificate(&leftLine[0],Top_Point,Huan_Mid);
            }
            else
            {     BEEP_ON ;
                  Huan_White_L=0;
                  for(i=CAMERA_H-1;i>=40&&i>(CAMERA_H-MidNumbers);i--)
                  {
                         if(leftLine[i]==0)
                         {
                              while(leftLine[i]==0)
                              {
                                   Huan_White_L++;
                                   i--;
                                   Top_Point=i;
                                   printf("i=%d\n",i);
                              }
                              break;
                         }      
                  }
                  printf("Huan_White_L=%d\n",Huan_White_L);
                  if(Huan_White_L>3)
                  {     
                        if(Loop_in_Start==0)
                        {
                              if((leftLine[Top_Point]-leftLine[Top_Point+1])>20)
                              {
                                   Loop_in_Start=1;
                              }
                              else
                                   Loop_in_Start=0;
                        }
                        if(Loop_in_Start==0)
                        {
                              if(rightLine[CAMERA_H-1]==79)
                              {
                                   i=CAMERA_H-1;
                                   while(rightLine[i]==79)
                                   {
                                           White_down_count++;
                                           down_line=i-1;
                                           i--;   
                                   }
                                   if(White_down_count>=25)
                                   {
                                              Top_Point=down_line;
                                              Huan_Mid=CAMERA_H-1;
                                              leftLine[Top_Point]=rightLine[Top_Point];
                                              CommonRectificate(&leftLine[0],Top_Point,Huan_Mid);  
                                   }
                                   else
                                   {
                                           i=down_line;
                                           while(rightLine[i]!=79)
                                           {
                                        //       Black_Huan_count++; 
                                               if(rightLine[i]>=rightLine[i-1])
                                                  Huan_Mid=i-1;
                                               up_line=i;
                                               i--;
                                           }
                                           i=up_line-1;//Ѱ���Ϸ�����
                                           while(rightLine[i]==79)
                                           {   
                                                 White_up_count++;
                                                 i--;
                                                 Top_Point=i;
                                           }
                                          leftLine[Top_Point]=rightLine[Top_Point];
                                          CommonRectificate(&leftLine[0],Top_Point,Huan_Mid);
                                   }
                              }
                              else
                              {          
                                          i=CAMERA_H-1;
                                          while(rightLine[i]!=79)
                                          {
                                           //    Black_Huan_count++; 
                                               if(rightLine[i]>=rightLine[i-1])
                                                  Huan_Mid=i-1;
                                               up_line=i;
                                               i--;
                                          }
                                          i=up_line-1;//Ѱ���Ϸ�����
                                          while(rightLine[i]==79)
                                          {   
                                                 White_up_count++;
                                                 i--;
                                                 Top_Point=i;
                                          }
                                          leftLine[Top_Point]=rightLine[Top_Point];
                                          CommonRectificate(&leftLine[0],Top_Point,Huan_Mid); 
                              }
                        }
                        else
                        {
                             Huan_Mid=CAMERA_H-1;
                             CommonRectificate(&leftLine[0],Top_Point,Huan_Mid);
                        }
                  }
                  else
                  {  
                         for(i=CAMERA_H-1;i>=CAMERA_H-MidNumbers&&i>12;i--)//������Ұ
                         {
                                
                                   Var_L=Var_L+(leftLine[i-1]- leftLine[i]);//�ұ�Ϊֱ��
                                   L_count++;
                         }
                         Aver_L=Var_L*1.0/L_count;
                     //   printf("%f\n",Aver_R);
                         if(Aver_L>0)
                         {
                              if(rightLine[CAMERA_H-1]==79)
                              {
                                   i=CAMERA_H-1;
                                   while(rightLine[i]==79)
                                   {
                                           White_down_count++;
                                           down_line=i-1;
                                           i--;   
                                   }
                                   if(White_down_count>=25)
                                   {
                                              Top_Point=down_line;
                                              Huan_Mid=CAMERA_H-1;
                                              leftLine[Top_Point]=rightLine[Top_Point];
                                              CommonRectificate(&leftLine[0],Top_Point,Huan_Mid);  
                                   }
                                   else
                                   {
                                           i=down_line;
                                           while(rightLine[i]!=79)
                                           {
                                          //     Black_Huan_count++; 
                                               if(rightLine[i]>=rightLine[i-1])
                                                  Huan_Mid=i-1;
                                               up_line=i;
                                               i--;
                                           }
                                           i=up_line-1;//Ѱ���Ϸ�����
                                           while(rightLine[i]==79)
                                           {   
                                                 White_up_count++;
                                                 i--;
                                                 Top_Point=i;
                                           }
                                          leftLine[Top_Point]=rightLine[Top_Point];
                                          CommonRectificate(&leftLine[0],Top_Point,Huan_Mid);
                                   }
                              }
                              else
                              {          
                                          i=CAMERA_H-1;
                                          while(rightLine[i]!=79)
                                          {
                                           //    Black_Huan_count++; 
                                               if(rightLine[i]>=rightLine[i-1])
                                                  Huan_Mid=i-1;
                                               up_line=i;
                                               i--;
                                          }
                                          i=up_line-1;//Ѱ���Ϸ�����
                                          while(rightLine[i]==79)
                                          {   
                                                 White_up_count++;
                                                 i--;
                                                 Top_Point=i;
                                          }
                                          leftLine[Top_Point]=rightLine[Top_Point];
                                          CommonRectificate(&leftLine[0],Top_Point,Huan_Mid); 
                              }
                         }
                         else 
                         {   BEEP_OFF;
                              Loop_Flag_2=0;
                              Loop_Flag_1=0;
                              Loop_in_Start=0;
                              Loop_C=0;
                         }
                       
                  }
            }
        }   
    }
    LOOP_Error2 = Get_LOOP_Error2(0,0,0);
    LOOP_Error = SteerPidCal(LOOP_Error2);
    FTM_PWM_Duty(FTM1, FTM_CH0,LOOP_Error );
}