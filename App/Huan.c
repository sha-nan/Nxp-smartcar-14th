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

/*************************************两点拟合************************************/
void CommonRectificate(int16 data[],unsigned char begin,unsigned char end)
{
	unsigned char MidPos = 0;
	if (end > CAMERA_H-1)  //限位
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

		data[MidPos] = (data[begin]+data[end])/2;  //左（右）线中间点位置

		midLine[MidPos] =  leftLine[MidPos] + (rightLine[MidPos]-leftLine[MidPos])/2;	//中线中间点位置
	
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
	if (end > CAMERA_H-1)  //限位
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

		data[MidPos] = (data[begin]+data[end])/2;  //左（右）线中间点位置

		//midLine[MidPos] =  leftLine[MidPos] + (rightLine[MidPos]-leftLine[MidPos])/2;	//中线中间点位置
	
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
/*************************圆环判断***********************************/
//入圆环标志
int Loop_Flag_1=0;//1左，2右
int RLoop_Flag_1=0;
int LLoop_Flag_1=0;
int RLoop_Flag_2=0;
int LLoop_Flag_2=0;

unsigned char Huan_Max=0; //圆环中间点

unsigned char Ricount=0;
unsigned char Licount=0;
unsigned char Ristart=0;
unsigned char Listart=0;
unsigned char Riend=0;
unsigned char Liend=0;
//左右直线行计数
signed char Var_R=0;
signed char Var_L=0;
float Aver_R=0;
float Aver_L=0;
unsigned char R_count=0;
unsigned char L_count=0;
uint8 Loop_Judge(void)
{ 
    short i;
    //左右全白计数
    Ricount=0;
    Licount=0;
    Ristart=0;
    Listart=0;
    Riend=0;
    Liend=0;
    //左右直线行计数
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
    
    if(MidNumbers>=49)//保证视野，屏蔽弯道单边丢线
    {
        for(i=CAMERA_H-1;i>=CAMERA_H-MidNumbers&&i>10;i--)//找左边连续全白行
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
        for(i=CAMERA_H-1;i>=CAMERA_H-MidNumbers&&i>10;i--)//找右边连续全白行
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
    
    if(Licount>=20&&Licount<=35)//滤掉单边丢线过多
    {
         for(i=CAMERA_H-1;i>=CAMERA_H-MidNumbers&&i>10;i--)//限制视野
         {
             if((rightLine[i]>=rightLine[i-1])&&(rightLine[i]-rightLine[i-1])<=2&&rightLine[i]!=0) 
             {   
                   Var_R=Var_R+(rightLine[i]- rightLine[i-1]);//右边为直线
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
    else if(Ricount>=20&&Ricount<=35)//滤掉单边丢线过多
    {   
         for(i=CAMERA_H-1;i>=CAMERA_H-MidNumbers&&i>10;i--)//限制视野
         {
             if((leftLine[i-1]>=leftLine[i])&&(leftLine[i-1]-leftLine[i])<=2&&leftLine[i]!=0)
             {  
                 Var_L=Var_L+(leftLine[i-1]-leftLine[i]);//左边为直线
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
    //内部小环
    int Huan_Start=0;//环形开始
    int Huan_End=0;//环形结束
    int Huan_up_count=0;//上半部分计数
    int Huan_down_count=0;//下半部分计数
    int Huan_Var_D_up=0;//下半圆上半部分偏差和
    int Huan_Var_D_down=0;//下半圆下半部分偏差和
    float Huan_aver_D_up=0.0;//下半圆上半部分平均值
    float Huan_aver_D_down=0.0;//下半圆下半部分平均值
    int Mid=0;//下半部分计数中点
    
    if(LLoop_Flag_1)
    {
          Huan_Start= Liend;//判断为小环，且跳变小，防止全白后出现直线
          if((leftLine[Huan_Start-1]-leftLine[Huan_Start]<=12)&&(leftLine[Huan_Start-2]-leftLine[Huan_Start-1]<=9))
          {
                  i=Huan_Start-1;
                  while(leftLine[i]>=leftLine[i+1])//要求连续
                  {
                      Huan_Max=i;
                      Huan_down_count++;   
                      i--;
                  }
                  //下半圆上下两部分偏差平均值
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
                  
                  //上半部分圆
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

/***************************圆环处理**********************************/
int Loop_f_R_count=0;//环标志计数
int Loop_f_L_count=0;
int Loop_C=0;//环计数
unsigned char Corner_L=0;//第一次补线，下方的点
unsigned char Corner_R=0;
int White_up_count=0;//内部小环上下全白行
int White_down_count=0;
int down_line=0;//内环下方起始行
int up_line=0;//内环上方截止行
int Black_Huan_count_U=0,Black_Huan_count_D=0;//内环上下部行计数
unsigned char Huan_Mid=0;//内环中点
unsigned char Top_Point=0;//第二次补线上方跳变点
int Huan_Black_R=0;//进环边线
int Huan_Black_L=0;
int Huan_White_R=0;//进环边界(左右全白)判断
int Huan_White_L=0;

int Loop_Flag_2=0;//第二个入环标志
int Loop_in_Start=0;//开始进环
int Loop_in=0;//进入环
//int little_F=0;//小环标志
//int little_H=0;//小环
unsigned char D_W_F=0;//识别到环后进入全白或黑标志
unsigned char little_N=0;//小环消失标志
unsigned char little_Y=0;//下方只有小环标志

int16 H_leftLine[CAMERA_H]; //环边线数组
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

    //左右直线行计数
    Var_R=0;
    Var_L=0;
    Aver_R=0;
    Aver_L=0;
    R_count=0;
    L_count=0;
    
    for(i=0;i<=59;i++)//初始化数组
    {
         H_leftLine[i]=leftLine[i];
         H_rightLine[i]=rightLine[i];
    }
    
    Loop_C++;//进入环计数
      
    if(Loop_Flag_1==1) //1  左
    {   
        if(Loop_C==1)//第一次补线
        {
              Corner_L= Listart+1;
              for(i=Huan_Max;i<=Corner_L;i++)
              {
                   midLine[i] = rightLine[i]-Mid_cor[i];
              }
             // CommonRectificate(&leftLine[0],Huan_Max,Corner_L);
        }
        else if(Loop_C>1&&Loop_Flag_2==0)//第一次补线
        {     
            // sub_M = leftLine[Huan_Mid]-0;  //视野范围几乎看不见内环判断
             
             if(D_W_F==0)//是否下方全白
             {
                    if(leftLine[CAMERA_H-1]==0)
                    {
                        i=CAMERA_H-1;
                        while(leftLine[i]==0)//下部
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
             if(D_W_F==1)//进入下方全白
             {                     
                     Corner_L=CAMERA_H-1;
                     //判断下方是否全白行
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
                     //有白行，补左线
                     if(White_down_count>0&&little_Y==0)
                     { 
                        BEEP_OFF;
                          little_Y=0;
                          i= down_line;
                          while(leftLine[i]!=0)//下部
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
                     //无白行
                     else if(little_Y==1||White_down_count==0)
                     {  
                          BEEP_ON; 
                          little_Y=1;
                          //判断下方视野内小环是否消失
                          if(little_N==0)
                          {
                                 i=CAMERA_H-1;
                                 while(leftLine[i]!=0)//上部
                                {
                                          Black_Huan_count_U++;
                                          up_line=i;
                                          i--;             
                                }
                          }
                          //视野内有小环，第一次补线
                          if(Black_Huan_count_U>0 && little_N==0)
                          {    
                                little_N=0;
                                for(i=up_line;i<=Corner_L;i++)
                                {
                                     midLine[i] = rightLine[i]-Mid_cor[i];
                                }
                               // CommonRectificate(&leftLine[0],Huan_Mid,Corner_L);
                          }
                          //视野内无小环，第二次补线
                          else if(little_N==1||Black_Huan_count_U==0)
                          {   
                                      little_N=1;
                                      Huan_Mid=CAMERA_H-1;
                                      gpio_init(PTA19,GPO,1);
                                      //寻找上方跳变
                                      i=CAMERA_H-1;
                                      while(leftLine[i]<=1)
                                      {   
                                           White_up_count++;
                                           i--;
                                           Top_Point=i;
                                      }
                                      i=Top_Point;
                                      //补线
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
             else//左下角有线
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
                      while(leftLine[i]!=0)//下部
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
        //进入第二次补线
        else if(Loop_Flag_2==1)//&&Loop_in==0)
        {   
            //上一次标志清零
            D_W_F=0;           
            little_N=0;
            little_Y=0;
            //判断进环分界点  
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
            //未到分界点
            if(Loop_in_Start==0)
            {
                  BEEP_OFF;
                  gpio_init(PTA19,GPO,0);
                  Huan_Mid=CAMERA_H-1;
                  //找跳变点
                  i=CAMERA_H-1;
                  while(leftLine[i]<=1)
                  {   
                       White_up_count++;
                       i--;
                       Top_Point=i;
                  }
                  //补线
                  rightLine[Top_Point]=leftLine[Top_Point];
                  CommonRectificate_2(&rightLine[0],Top_Point,Huan_Mid); 
                  for(i=Top_Point;i<=Huan_Mid;i++)
                  {
                       midLine[i]=rightLine[i] - Mid_cor[i];
                  }
                  for(int a=Huan_Mid;a<=59;a++)
                       midLine[a]=midLine[Huan_Mid-3];
            }
            //到分界点
            else
            {     
                  BEEP_ON;
                   gpio_init(PTA19,GPO,0);
                  //扫描环作为右线
                  int right_flag = 0;
                  int num=0;//总计数
                  int find_c=0;//找到线计数
                  unsigned char first_f=0;//找到补线行标志
                  for(i=CAMERA_H-1;i>=4;i--)
                  {
                          right_flag = 0;
                          if(ImageData[i][0]==Black&&ImageData[i][1]==Black)
                                        break;
                          for(int temp = 0; temp < 77; temp++)  
                          {  
                                // 寻找右黑线  
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
                  //补中线
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
                  //补右线
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
       //完全进入环
     /*  else if(Loop_in==1)
       {
         
        /*    for(i=CAMERA_H-1;i>=CAMERA_H-MidNumbers;i--)
            {
                 if()
            }*/
            //补中线
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
    else if(Loop_Flag_1==2)//2右
    {                      
        if(Loop_C==1)//第一次补线
        {
              Corner_R= Ristart+1; 
              CommonRectificate(&rightLine[0],Huan_Max,Corner_R);
        }
        else if(Loop_C>1&&Loop_Flag_2==0)
        {                                                                     
             for(i=CAMERA_H-1;i>25;i--)//寻找下方全白
             {                         //第一次补线找点
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
             //内环连续行
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
              //第一次补线
              if(White_down_count>=7&&sub_M>=3)
              {      
                    CommonRectificate(&rightLine[0],Huan_Mid,Corner_R);
              } 
              //第二次补线
              else if(White_down_count<7||sub_M<3)
              {     
                    i=up_line-1;//寻找上方跳变
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
        else if(Loop_Flag_2==1)//进入第二次补线
        {  
            //进环右边界判断
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
                                           i=up_line-1;//寻找上方跳变
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
                                          i=up_line-1;//寻找上方跳变
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
                         for(i=CAMERA_H-1;i>=CAMERA_H-MidNumbers&&i>12;i--)//限制视野
                         {
                                
                                   Var_L=Var_L+(leftLine[i-1]- leftLine[i]);//右边为直线
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
                                           i=up_line-1;//寻找上方跳变
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
                                          i=up_line-1;//寻找上方跳变
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