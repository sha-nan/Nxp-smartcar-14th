#include "Image.h"
#include "Public.h"
#include "FIRE_OV7725_Eagle.h"
#include "CrossRoad.h"

/******************函数声明***********/
uint8 Crossroad_Judge(void);
extern void CommonRectificate(int16 data[],unsigned char begin,unsigned char end);
//中线矫正数组
extern unsigned char Mid_cor[60];
extern int World_Y[60];
extern int World_X[60][80];
/*****************全局变量******************/
#define FIND_CENTER     0  
#define FIND_LEFT       1  
#define FIND_RIGHT      2  
#define CENTER_POINT    CAMERA_W/2  
#define FIND_COUNT      8
unsigned char ImageData[CAMERA_H][CAMERA_W];//图像数据存储数组

int16 centerLine[CAMERA_H+1] = {0};          // 最后一个元素用来记录转向点对应的行数  
int16 leftLine[CAMERA_H] = {0};  
int16 rightLine[CAMERA_H] = {0}; 
int16 midLine[CAMERA_H] ={0};

static uint8 leftFindFlag;              // 用来标记左黑线是否找到  
static uint8 rightFindFlag;             // 用来标记右黑线是否找到  
//预测左右点  
static int16 leftCount;  
static int16 rightCount;  
static int16 findLine; 
//找不到左右或者找到计数
unsigned char N_Lcount;  //找不到左计数
unsigned char N_Rcount;  //找不到右计数 
unsigned char Find_Lcount;//找到左或右计数
unsigned char Find_Rcount;
unsigned char N_LRcount;//找不到左右
unsigned char Find_LRcount;//找到左右

unsigned char MidNumbers=0; //中线长
/********************绝对值************************/
int fabss(int n)
{
      if(n < 0)return (-1)*n;
      else return n;	
}

/********利用最小二乘法生成需要补全的点************/  
int16 createPoint(int type, int line)  
{  
    int16 *linePointer;  
    int8 tmp = 0; 
    int POINT_COUNT=5;
    double sumX = 0;  
    double sumY = 0;  
    double averageX = 0;  
    double averageY = 0;  
    double sumUp = 0;  
    double sumDown = 0;  
    double parameterA;  
    double parameterB;  
      
    if(type == FIND_LEFT)  //左线补点
        linePointer = &leftLine[line];  
    else if(type == FIND_RIGHT) //右线补点 
        
        linePointer = &rightLine[line];  
    else  
        linePointer = &midLine[line];  //中线补点
      
    // 取邻近的 POINT_COUNT 个点进行拟合  
    while(++tmp <= POINT_COUNT)  //POINT_COUNT初始化=5
    {  
        sumX += (line+tmp);  
        sumY += linePointer[tmp];  
    }  
    --tmp;  
    averageX = sumX/tmp;  //tmp=5
    averageY = sumY/tmp;  //tmp=5
    do  
    {  
        sumUp += (linePointer[tmp]-averageY) * (line+tmp-averageX);  
        sumDown += (line+tmp-averageX) * (line+tmp-averageX);  
    } while(--tmp > 0);  
      
    if(sumDown == 0)  
        parameterB = 0;  
    else  
        parameterB = sumUp/sumDown;  
    parameterA = averageY-parameterB*averageX; //最小二乘法的公式a=y(平均）-a*x(平均）
    return (int16)(parameterA+parameterB*line+0.5);  
} 
/**********************提取边缘线**************************************/ 
void Get_EdgeLine()
{  
    int8 temp; 
    unsigned char MidEnd = 0;
    unsigned char MidToBlackCount = 0;
    int M_B=0;//中间寻线或者边缘寻线标志
    
    N_Rcount=0;
    N_Lcount=0;
    N_LRcount=0;
    Find_Lcount=0;
    Find_Rcount=0;
    Find_LRcount=0;
    MidNumbers=0; 
    
    memset(leftLine, 0, sizeof(leftLine));//sizeof的作用，计数字节数
    for(int i=0;i<CAMERA_H;i++)
    {
       rightLine[i]=79;
    }
   /*****************前十行，从中间往两边查找**************/  
    for(findLine = CAMERA_H-1; findLine > CAMERA_H-11; findLine--)  
    {  
        leftFindFlag = 0;  
        rightFindFlag = 0;  
        for(temp = 0; temp < CENTER_POINT; temp++)  
        {  
            // 寻找左黑线  
            if(leftFindFlag == 0  
            && ImageData[findLine][CENTER_POINT-temp-1] == Black 
            && ImageData[findLine][CENTER_POINT-temp] == White 
            && ImageData[findLine][CENTER_POINT-temp+1] == White  
            && ImageData[findLine][CENTER_POINT-temp+2] == White)  
            {  
                leftLine[findLine] = CENTER_POINT-temp;  //此处注意左边界的列数等于Black+1;
                leftFindFlag = 1;  //用来标记左黑线已找到 
            }  
            // 寻找右黑线  
            if(rightFindFlag == 0  
            &&ImageData[findLine][CENTER_POINT+temp] == Black  
            && ImageData[findLine][CENTER_POINT+temp-1] == White  
            && ImageData[findLine][CENTER_POINT+temp-2] == White  
            && ImageData[findLine][CENTER_POINT+temp-3] == White)  
            {  
                rightLine[findLine] = CENTER_POINT+temp-1;  //此处注意右边界的列数等于Black-1;
                rightFindFlag = 1;  //用来标记右黑线已找到 
            }  
            if(leftFindFlag == 1 && rightFindFlag == 1)  
                break;  //跳出第二个for循环
        }  
        // 对未找到的黑线进行补全  
        if(leftFindFlag == 0)  
        {
             leftLine[findLine] = 0; 
             N_Lcount++;  //找不到左计数自加
        }    
        if(rightFindFlag == 0)  
        {
             rightLine[findLine] = CAMERA_W-1;
             N_Rcount++;  //找不到右计数自加
        }      
        MidNumbers++;  //中线长自加
        // 对中线进行赋值
        centerLine[findLine] = (leftLine[findLine]+rightLine[findLine])/2;
    }  
    /************** 十行后，根据前面行位置查找黑线****************/ 
   for(findLine = CAMERA_H-11; findLine >= 2 && !MidEnd; findLine--) 
   {
        /******************************寻左线****************************/
         leftFindFlag =0;
         /****基础行内全白行过多****/
         if(N_Lcount>=5||Find_Lcount<5)
         {    
              M_B=0;  //中间寻线或者边缘寻线标志
              for(temp = 0; temp < CENTER_POINT; temp++)  
              { 
                   if(leftFindFlag == 0  //注意是等于运算符
                   && ImageData[findLine][CENTER_POINT-temp-1] == Black 
                   && ImageData[findLine][CENTER_POINT-temp] == White 
                   && ImageData[findLine][CENTER_POINT-temp+1] == White  
                   && ImageData[findLine][CENTER_POINT-temp+2] == White)  
                    {  
                         leftLine[findLine] = CENTER_POINT-temp;  
                         leftFindFlag = 1;  
                    }
                    if(leftFindFlag == 1)  //注意是等于运算符
                    {
                         Find_Lcount++;  //找到左或右计数
                         break;
                    }
              }
         }
         else if(N_Lcount<5)
         {
               M_B=1;  //中间寻线或者边缘寻线标志,置1代表边缘寻线
               leftCount = createPoint(FIND_LEFT, findLine);  
               for(temp = 0;temp < FIND_COUNT*2+1; temp++)  
               {  
                   if(leftFindFlag != 0)  
                        break;  
                   else if((leftCount-temp+FIND_COUNT)+7 > CAMERA_W-1)  
                       continue;  
                   else if((leftCount-temp+FIND_COUNT) < 0)  
                        break;  
                   else if(ImageData[findLine][leftCount-temp+FIND_COUNT] == Black 
                        && ImageData[findLine][leftCount-temp+FIND_COUNT+1] == White 
                        && ImageData[findLine][leftCount-temp+FIND_COUNT+2] == White  
                        &&ImageData[findLine][leftCount-temp+FIND_COUNT+3] == White)  
                         {  
                              leftLine[findLine] = leftCount-temp+FIND_COUNT+1;  
                              leftFindFlag = 1; 
                              Find_Lcount++;
                         }  
               }  
         }
         if(leftFindFlag == 0&&M_B==1)
         { 
              for(temp = 0; temp < CENTER_POINT; temp++)  
              {   
                   if(leftFindFlag == 0  
                   && ImageData[findLine][CENTER_POINT-temp-1] == Black 
                   && ImageData[findLine][CENTER_POINT-temp] == White 
                   && ImageData[findLine][CENTER_POINT-temp+1] == White  
                   && ImageData[findLine][CENTER_POINT-temp+2] == White)  
                   { 
                        leftLine[findLine] = CENTER_POINT-temp;  
                        leftFindFlag = 1;  
                   }
                   if(leftFindFlag == 1)
                   {
                          Find_Lcount++;
                          break;
                   }
              }
              if(leftFindFlag == 0)
                   leftLine[findLine]=0;
         }
         else if(leftFindFlag == 0&&M_B==0)
               leftLine[findLine]=0;
         if(leftFindFlag == 0)//丢线计数，作为寻线基础
               N_Lcount++;
         else
               N_Lcount=0;
         if(N_Lcount>=5)
               Find_Lcount=0;
   /******************************************寻右线********************************/
         rightFindFlag = 0;
         if(N_Rcount>=5||Find_Rcount<5)
         {  
             M_B=0;
             for(temp = 0; temp < CENTER_POINT; temp++)  
             {    
                 if(rightFindFlag == 0  
                 &&ImageData[findLine][CENTER_POINT+temp] == Black  
                 && ImageData[findLine][CENTER_POINT+temp-1] == White  
                 && ImageData[findLine][CENTER_POINT+temp-2] == White  
                 && ImageData[findLine][CENTER_POINT+temp-3] == White)  
                 {  
                      rightLine[findLine] = CENTER_POINT+temp-1;  
                      rightFindFlag = 1;  
                 }
                 if(rightFindFlag == 1)
                 {
                       Find_Rcount++;
                       break;
                 }      
             }
         }
         else if(N_Rcount<5)
         {
               M_B=1;
               rightCount = createPoint(FIND_RIGHT, findLine);
               for(temp = 0;temp< FIND_COUNT*2+1; temp++)  
               {  
                     if(rightFindFlag != 0)  
                           break;  
                     else if((rightCount+temp-FIND_COUNT)-7 < 0)  
                           continue;  
                     else if(rightCount+temp-FIND_COUNT >CAMERA_W-1)  
                           break;  
                     else if(ImageData[findLine][rightCount+temp-FIND_COUNT] ==Black  
                            && ImageData[findLine][rightCount+temp-FIND_COUNT-1] == White  
                            && ImageData[findLine][rightCount+temp-FIND_COUNT-2] ==White  
                            && ImageData[findLine][rightCount+temp-FIND_COUNT-3] == White)  
                           {  
                               rightLine[findLine] = rightCount+temp-FIND_COUNT-1;  
                               rightFindFlag = 1;  
                               Find_Rcount++;
                           }  
                }
               
         }
         if(rightFindFlag == 0&&M_B==1)  //补右黑线
         {
              for(temp = 0; temp < CENTER_POINT; temp++)  
              {    
                   if(rightFindFlag == 0  
                    &&ImageData[findLine][CENTER_POINT+temp] == Black  
                    && ImageData[findLine][CENTER_POINT+temp-1] == White  
                    && ImageData[findLine][CENTER_POINT+temp-2] == White  
                    && ImageData[findLine][CENTER_POINT+temp-3] == White)  
                   {  
                         rightLine[findLine] = CENTER_POINT+temp-1;  
                         rightFindFlag = 1;  
                   }  
                   if(rightFindFlag == 1)
                   {
                         Find_Rcount++;
                         break;
                   }           
              }
              if(rightFindFlag==0)
                  rightLine[findLine] =79;//注意数组元素个数
         }
         else if(rightFindFlag == 0&&M_B==0)
               rightLine[findLine] =79;
         if(rightFindFlag == 0)//丢线计数，作为寻线基础
               N_Rcount++;
         else
               N_Rcount=0;
         if(N_Rcount>=5)
               Find_Rcount=0;
   /**************************************判断截止行*******************************/
        if ((ImageData[findLine][(leftLine[findLine]+rightLine[findLine])/2] == Black)||(leftLine[findLine]-rightLine[findLine]>1))
        {
             MidToBlackCount++;        //连续趋向黑线
             if (MidToBlackCount >= 2)
             {
                  MidEnd = 1;
             }
        }
        else   MidToBlackCount = 0;
        if (!MidEnd)
        {
            MidNumbers++;  //中线长
        }
     /*  if(leftLine[findLine]!=0&&rightLine[findLine]!=79&&leftLine[findLine+1]!=0&&rightLine[findLine+1]!=79
          &&(fabss(leftLine[findLine]-leftLine[findLine+1])>10||fabss(rightLine[findLine]-rightLine[findLine+1])>10))
                        break;*/
   }
  /*************显示边线************/
 //  int x=0,y=0;
 /* for(int i=59;i>=60-MidNumbers;i--)
  {

      printf("i=%d,left=%d,right=%d\n",i,leftLine[i],rightLine[i]);
  }
  printf("num=%d\n",MidNumbers);*/

  if(BOMA==3)
  {
     for(int j = 0;j<60;j++)
     {   
       if(j>findLine)
       {
           for(int i=0;i<80;i++)
           {
              if (i==rightLine[j])
              LCD_DrawPoint(i,j,1);
              else if(i==leftLine[j])
              LCD_DrawPoint( i,j,1);
           /*   else if(j==10)
                LCD_DrawPoint(i,j,1);
              else if(j==13)
                LCD_DrawPoint(i,j,1);*/
              else
                 LCD_DrawPoint( i,j,0);  
           }
       }
       else 
         for(int i=0;i<80;i++)
           LCD_DrawPoint( i,j,0);
     }
     LCD_Refresh_Gram();
  }
 // printf("midnumbers=%d\n",MidNumbers);

}

/************************图像滤波****************************/
void Clear()
{
  	int i=0;
	unsigned char *p=0;//unsigned char 一个字节
	
  	for (i =20;i < CAMERA_H;i++)
	{
		p = &ImageData[i][15];
		while (p < &ImageData[i][CAMERA_W-15])
		{
			if ((*(p-4) == White) && (*p == Black )&& (*(p+4) == White))//一个整形数据占4个字节
			{
				*p = White;
			}			
			p++;//指针在内存中移动1个字节//8位
		}
	}  
}
/*****************************通常中线************************/
unsigned char Mid_End=0;
int Mid_Line_C;
void Mid_Line_process(void)
{     
      int i,n,Sub;
      int temp=0;
      Mid_Line_C=0;
      int L_and_R_flag=0,L_flag=0,R_flag=0,L_no_R_flag=0;
      unsigned char no_start=0,no_end=0,no_count=0;
                         //左丢    //右丢
      for(i=CAMERA_H-1;i>CAMERA_H-1-MidNumbers;i--)
      {
             if(leftLine[i]!=0&&rightLine[i]!=79)//左右都有线
             {
                   midLine[i]=(leftLine[i]+rightLine[i])/2;
                   
                   L_and_R_flag=1;
                   L_flag=0;
                   R_flag=0;
                   L_no_R_flag=0;
                   
                    no_start=0;
                    no_count=0;
                    no_end=0;
             }
             else if(leftLine[i]==0&&rightLine[i]==79)//全丢
             {     
                    if(!no_count)
                      no_start=i;
                    no_count++;
                    no_end=i;
                    
                    if(L_and_R_flag==0&&L_flag==0&&R_flag==0&&L_no_R_flag==0)//直接进入全丢
                    {
                         midLine[i]=39;
                         L_and_R_flag=0;
                         L_flag=0;
                         R_flag=0;
                         L_no_R_flag=1;
                    }
                    else if(L_no_R_flag==1)
                    { 
                         midLine[i]=midLine[i+1];
                         L_and_R_flag=0;
                         L_flag=0;
                         R_flag=0;
                         L_no_R_flag=1;
                    }
                    else if(L_no_R_flag==0|| L_no_R_flag==2)
                    {
                         if(i<=53)
                         {
                              temp = createPoint(FIND_CENTER,i);
                              midLine[i]=temp;
                               
                               L_and_R_flag=0;
                               L_flag=0;
                               R_flag=0;
                               L_no_R_flag=2;
                         }
                         else if(midLine[i+1]!=0)
                         {
                                midLine[i]=midLine[i+1];
                                L_and_R_flag=0;
                                L_flag=0;
                                R_flag=0;
                                L_no_R_flag=3;
                         }  
                    }
                    else if(L_no_R_flag==3)
                    {
                                midLine[i]=midLine[i+1];
                                L_and_R_flag=0;
                                L_flag=0;
                                R_flag=0;
                                L_no_R_flag=3;
                    }
              }
              //左丢 
              else if(leftLine[i] <= 0 && rightLine[i] < 79)  
              {  
                      if(L_and_R_flag==0&&L_flag==0&&R_flag==0&&L_no_R_flag==0)//直接进入左丢线
                      {
                           midLine[i]=rightLine[i]-Mid_cor[i];//根据矫正数组确定中线
                           L_and_R_flag=0;
                           L_flag=1;
                           R_flag=0;
                           L_no_R_flag=0;
                      }
                      else if(L_flag==1)//上一次是左丢,且上一次用矫正数组
                      {
                           midLine[i]=rightLine[i]-Mid_cor[i];
                           L_and_R_flag=0;
                           L_flag=1;
                           R_flag=0;
                           L_no_R_flag=0;
                      }
                      else if(L_no_R_flag!=0)//上一次全丢
                      { 
                           if(L_no_R_flag==2)
                           {
                                midLine[i]=rightLine[i]-Mid_cor[i];
                                Sub=midLine[i]-midLine[i+1];
                                midLine[i]=midLine[i]-Sub;
                                
                                 L_and_R_flag=0;
                                 L_flag=2;
                                 R_flag=0;
                                 L_no_R_flag=0;
                           }
                           
                           else 
                           {
                                 midLine[i]=rightLine[i]-Mid_cor[i];
                                 CommonRectificate(&midLine[0],no_end,no_start);
                                 L_and_R_flag=0;
                                 L_flag=1;
                                 R_flag=0;
                                 L_no_R_flag=0;
                           }
                          
                      }
                      else if(L_and_R_flag==1||L_flag==2)//上一次全有，或上一次用偏移量
                      {
                          temp = midLine[i+1] + (rightLine[i] - rightLine[i+1]); // 根据右黑线的偏移量来确定中线   
                          if(temp <= 0)             
                              midLine[i] = 0;   // 中线超出范围则跳出循环，记录该行为转向行  
                          else  
                              midLine[i] = temp;  
                           L_and_R_flag=0;
                           L_flag=2;
                           R_flag=0;
                           L_no_R_flag=0;
                      }
                      else if(R_flag==1||R_flag==2)//上一次右丢
                      {
                           midLine[i]=rightLine[i]-Mid_cor[i];
                           Sub=midLine[i]-midLine[i+1];
                           midLine[i]=midLine[i]-Sub;
                           L_and_R_flag=0;
                           L_flag=2;
                           R_flag=0;
                           L_no_R_flag=0;
                      }
                    no_start=0;
                    no_count=0;
                    no_end=0;
              } 
              // 右丢
              else if(leftLine[i] > 0 && rightLine[i] == 79) //here
              { 
                 
                      if(L_and_R_flag==0&&L_flag==0&&R_flag==0&&L_no_R_flag==0)//直接进入右丢线
                      {
                           midLine[i]=leftLine[i]+Mid_cor[i];
                           L_and_R_flag=0;
                           L_flag=0;
                           R_flag=1;
                           L_no_R_flag=0;
                      }
                      else if(R_flag==1)//上一次是右丢
                      {
                           midLine[i]=leftLine[i]+Mid_cor[i];
                           L_and_R_flag=0;
                           L_flag=0;
                           R_flag=1;
                           L_no_R_flag=0;
                      }
                      else if(L_no_R_flag!=0)//上一次全丢
                      { 
                           if(L_no_R_flag==2)
                           {
                                midLine[i]=leftLine[i]+Mid_cor[i];
                                Sub=midLine[i]-midLine[i+1];
                                midLine[i]=midLine[i]-Sub;
                                
                                 L_and_R_flag=0;
                                 L_flag=0;
                                 R_flag=2;
                                 L_no_R_flag=0;
                           }
                           
                           else 
                           {
                                 midLine[i]=leftLine[i]+Mid_cor[i];
                                 CommonRectificate(&midLine[0],no_end,no_start);
                                                           
                                 L_and_R_flag=0;
                                 L_flag=0;
                                 R_flag=1;
                                 L_no_R_flag=0;
                           }

                      }
                      else if(L_and_R_flag==1||R_flag==2)
                      {
                          temp = midLine[i+1] + (leftLine[i] - leftLine[i+1]); // 根据左黑线的偏移量来确定中线   
                          if(temp>=79)             
                              midLine[i] = 0;   // 中线超出范围则跳出循环，记录该行为转向行  
                          else  
                              midLine[i] = temp;  
                           L_and_R_flag=0;
                           L_flag=0;
                           R_flag=2;
                           L_no_R_flag=0;
                      }
                      else if(L_flag==1||L_flag==2)//上一次左丢
                      {
                           midLine[i]=leftLine[i]+Mid_cor[i];
                           Sub=midLine[i]-midLine[i+1];
                           midLine[i]=midLine[i]-Sub;
                           L_and_R_flag=0;
                           L_flag=0;
                           R_flag=2;
                           L_no_R_flag=0;
                      }
                    no_start=0;
                    no_count=0;
                    no_end=0;
              }  

             Mid_Line_C++;
             Mid_End=i;
             if(midLine[i]<=0||midLine[i]>=79)
             {
                  Mid_End=i+1;
                  break;
             }
      }
      if(Mid_End>=(CAMERA_H-1))
         Mid_End=CAMERA_H-1;
    //  printf(" Mid_End=%d\n", Mid_End);
} 
/***************************中心线滤波************************/
void AverageFilter()
{       int m=0;
        unsigned char i = 0;
	for (i = CAMERA_H-2;i >=Mid_End;i--)
	{

		if(fabss(midLine[i+1]- midLine[i])>=6&&fabss(midLine[i-1]- midLine[i])>=6)
		{                  
                  //  BlackLineData[i] = (BlackLineData[i+1]+BlackLineData[i-1])/2;
		     midLine[i] = midLine[i+3];	
		}
	}
}
/************图像处理***********/
void ImageProcess(void)
{   
    for(int i=0;i<=59;i++)
    {
        midLine[i]=255;
    }
    Clear();//滤黑点
    Get_EdgeLine();//提边线
    if(Crossroad_Judge()&&Loop_Flag_1==0)//十字
    {
         cross_process();
    }
   if(!Crossroad_Judge())
      BEEP_OFF;
   if(!Crossroad_Judge()&&Loop_Flag_1==0)//圆环判断
    { 
      Loop_Judge(); 
    }
    if(Loop_Flag_1!=0)//圆环处理
    {
        Loop_process1();  
    }
    if(!Crossroad_Judge()&&Loop_Flag_1==0)//普通中线
    {
         Mid_Line_process();
       //  AverageFilter();
          Turn();
    }
}



        /* 查找中线 */  
/*#if (FIND_TYPE == TYPE_1)                 
          
        // 补全未找到的左右黑线  
        // if(leftFindFlag == 0)  
        //  leftLine[findLine] = leftCount;  
        // if(rightFindFlag == 0)  
        //  rightLine[findLine] = rightCount;  
          
        /* 对中线进行赋值 */  
     /*   centerLine[findLine] = (leftLine[findLine]+rightLine[findLine])/2;  
        if(centerLine[findLine] <= 0)  
        {  
            centerLine[findLine] = 0;  
            break;  
        }  
        else if(centerLine[findLine] >= IMAGE_W-1)  
        {  
            centerLine[findLine] = IMAGE_W-1;  
            break;  
        }         
#else         
        // 补全未找到的左右黑线  
        // if(leftFindFlag == 0)  
        //  leftLine[findLine] = (leftCount < 0) ? 0 : ((leftCount > IMAGE_W-1) ? IMAGE_W-1 : leftCount);  
        // if(rightFindFlag == 0)  
        //  rightLine[findLine] = (rightCount < 0) ? 0 : ((rightCount > IMAGE_W-1) ? IMAGE_W-1 : rightCount);  
          
        /* 对中线进行赋值 */  
        // 左右黑线都存在则取左右黑线中值作为黑线值  
    /*    if(leftLine[findLine] > 0 && rightLine[findLine] < IMAGE_W-1)  
            centerLine[findLine] = (leftLine[findLine]+rightLine[findLine])/2;  
        // 左黑线超出范围  
        else if(leftLine[findLine] <= 0 && rightLine[findLine] < IMAGE_W-1)  
        {  
            // 根据右黑线的偏移量来确定中线  
            temp = centerLine[findLine+1] + (rightLine[findLine] - rightLine[findLine+1]);  
            // 根据最小二乘法补全中线  
            //temp = createPoint(FIND_CENTER, findLine);  
            if(temp <= 0)  
            {  
                // 中线超出范围则跳出循环，记录该行为转向行  
                centerLine[findLine] = 0;  
                break;  
            }  
            else  
                centerLine[findLine] = temp;  
        }  
        // 右黑线超出范围  
        else if(leftLine[findLine] > 0 && rightLine[findLine] >= IMAGE_W-1)  
        {  
            // 根据左黑线的偏移量来确定中线  
            temp = centerLine[findLine+1] + (leftLine[findLine] - leftLine[findLine+1]);  
            // 根据最小二乘法补全中线  
            //temp = createPoint(FIND_CENTER, findLine);  
            if(temp >= IMAGE_W-1)  
            {  
                // 中线超出范围则跳出循环，记录该行为转向行  
                centerLine[findLine] = IMAGE_W-1;  
                break;  
            }  
            else  
                centerLine[findLine] = temp;  
        }  
        // 左右黑线都超出范围  
        else  
        {  
            // 根据最小二乘法补全中线  
            temp = createPoint(FIND_CENTER, findLine);  
            // 根据中线偏移量补全中线  
            //temp = centerLine[findLine+1] + (rightLine[findLine] - rightLine[findLine+1]);  
            if(temp <= 0)  
            {  
                // 中线超出范围则跳出循环，记录该行为转向行  
                centerLine[findLine] = 0;  
                break;  
            }  
            else if(temp >= IMAGE_W-1)  
            {  
                // 中线超出范围则跳出循环，记录该行为转向行  
                centerLine[findLine] = IMAGE_W-1;  
                break;  
            }  
            else  
                centerLine[findLine] = temp;  
        }         
// 
          
    }  
    if(findLine<0 && centerLine[0]<0)  
        centerLine[0] = 0;  
    else if(findLine<0 && centerLine[0]>IMAGE_W-1)  
        centerLine[0] = IMAGE_W-1;  
      
    // 最后一个元素用来记录转向行  
    centerLine[IMAGE_H] = (findLine < 0) ? 0 : findLine;  
#endif       
/* 检查停车线 */  
/*#ifdef RELEASE    
    if(centerLine[IMAGE_H] == 0   
    && leftLine[0] >= 23   
    && rightLine[0] < IMAGE_W-23  
    && (centerLine[0]-centerLine[29] <= 3 || centerLine[0]-centerLine[29] >= -3))  
    {  
        for(temp = IMAGE_H-1; temp >= centerLine[IMAGE_H]; temp--)  
        {  
            if(carParams.stopCarFlag != STOP_LINE  
            && image[temp][(leftLine[temp]+centerLine[temp])/2] == IMG_BLACK  
            && image[temp][(rightLine[temp]+centerLine[temp])/2] == IMG_BLACK  
            && image[temp][(leftLine[temp]+centerLine[temp])/2-1] == IMG_BLACK  
            && image[temp][(leftLine[temp]+centerLine[temp])/2+1] == IMG_BLACK  
            && image[temp][(rightLine[temp]+centerLine[temp])/2-1] == IMG_BLACK  
            && image[temp][(rightLine[temp]+centerLine[temp])/2+1] == IMG_BLACK  
            //&& image[temp][leftLine[temp]+1] == IMG_WHITE  
            //&& image[temp][rightLine[temp]-1] == IMG_WHITE  
            && carParams.carRunTime > 3000/ENCODE_TIME)      // 小车起跑超过三秒则为停车线  
            {  
                for(tmp = (leftLine[temp]+centerLine[temp])/2+1; (rightLine[temp]+centerLine[temp])/2-1; tmp++)  
                {  
                    if(image[temp][tmp] == IMG_WHITE && image[temp][tmp+2] == IMG_WHITE)  
                    {  
                        carParams.stopCarFlag = STOP_LINE;  
                        goto STOP_LINE_PASS;  
                    }  
                }  
            }  
        }  
    STOP_LINE_PASS:  
        if(carParams.stopCarFlag == STOP_LINE && carParams.passStopLine != PASS_STOP_LINE && temp < 0)  
            carParams.passStopLine = PASS_STOP_LINE;        // 小车已越过停车线  
    }  
#endif  
      
    /* 
    // 在 TFT 上绘制出 3 线 
    for(temp = IMAGE_H-1; temp >= centerLine[IMAGE_H]; temp--) 
    { 
        guiDot(3*leftLine[temp], 3*temp, GREEN); 
        guiDot(3*rightLine[temp], 3*temp, RED); 
        guiDot(3*centerLine[temp], 3*temp, BLUE); 
    } 
     
      
    return (int16 *)centerLine;  
} */  
  

