#include "Image.h"
#include "Public.h"
#include "FIRE_OV7725_Eagle.h"
#include "CrossRoad.h"

/******************��������***********/
uint8 Crossroad_Judge(void);
extern void CommonRectificate(int16 data[],unsigned char begin,unsigned char end);
//���߽�������
extern unsigned char Mid_cor[60];
extern int World_Y[60];
extern int World_X[60][80];
/*****************ȫ�ֱ���******************/
#define FIND_CENTER     0  
#define FIND_LEFT       1  
#define FIND_RIGHT      2  
#define CENTER_POINT    CAMERA_W/2  
#define FIND_COUNT      8
unsigned char ImageData[CAMERA_H][CAMERA_W];//ͼ�����ݴ洢����

int16 centerLine[CAMERA_H+1] = {0};          // ���һ��Ԫ��������¼ת����Ӧ������  
int16 leftLine[CAMERA_H] = {0};  
int16 rightLine[CAMERA_H] = {0}; 
int16 midLine[CAMERA_H] ={0};

static uint8 leftFindFlag;              // �������������Ƿ��ҵ�  
static uint8 rightFindFlag;             // ��������Һ����Ƿ��ҵ�  
//Ԥ�����ҵ�  
static int16 leftCount;  
static int16 rightCount;  
static int16 findLine; 
//�Ҳ������һ����ҵ�����
unsigned char N_Lcount;  //�Ҳ��������
unsigned char N_Rcount;  //�Ҳ����Ҽ��� 
unsigned char Find_Lcount;//�ҵ�����Ҽ���
unsigned char Find_Rcount;
unsigned char N_LRcount;//�Ҳ�������
unsigned char Find_LRcount;//�ҵ�����

unsigned char MidNumbers=0; //���߳�
/********************����ֵ************************/
int fabss(int n)
{
      if(n < 0)return (-1)*n;
      else return n;	
}

/********������С���˷�������Ҫ��ȫ�ĵ�************/  
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
      
    if(type == FIND_LEFT)  //���߲���
        linePointer = &leftLine[line];  
    else if(type == FIND_RIGHT) //���߲��� 
        
        linePointer = &rightLine[line];  
    else  
        linePointer = &midLine[line];  //���߲���
      
    // ȡ�ڽ��� POINT_COUNT ����������  
    while(++tmp <= POINT_COUNT)  //POINT_COUNT��ʼ��=5
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
    parameterA = averageY-parameterB*averageX; //��С���˷��Ĺ�ʽa=y(ƽ����-a*x(ƽ����
    return (int16)(parameterA+parameterB*line+0.5);  
} 
/**********************��ȡ��Ե��**************************************/ 
void Get_EdgeLine()
{  
    int8 temp; 
    unsigned char MidEnd = 0;
    unsigned char MidToBlackCount = 0;
    int M_B=0;//�м�Ѱ�߻��߱�ԵѰ�߱�־
    
    N_Rcount=0;
    N_Lcount=0;
    N_LRcount=0;
    Find_Lcount=0;
    Find_Rcount=0;
    Find_LRcount=0;
    MidNumbers=0; 
    
    memset(leftLine, 0, sizeof(leftLine));//sizeof�����ã������ֽ���
    for(int i=0;i<CAMERA_H;i++)
    {
       rightLine[i]=79;
    }
   /*****************ǰʮ�У����м������߲���**************/  
    for(findLine = CAMERA_H-1; findLine > CAMERA_H-11; findLine--)  
    {  
        leftFindFlag = 0;  
        rightFindFlag = 0;  
        for(temp = 0; temp < CENTER_POINT; temp++)  
        {  
            // Ѱ�������  
            if(leftFindFlag == 0  
            && ImageData[findLine][CENTER_POINT-temp-1] == Black 
            && ImageData[findLine][CENTER_POINT-temp] == White 
            && ImageData[findLine][CENTER_POINT-temp+1] == White  
            && ImageData[findLine][CENTER_POINT-temp+2] == White)  
            {  
                leftLine[findLine] = CENTER_POINT-temp;  //�˴�ע����߽����������Black+1;
                leftFindFlag = 1;  //���������������ҵ� 
            }  
            // Ѱ���Һ���  
            if(rightFindFlag == 0  
            &&ImageData[findLine][CENTER_POINT+temp] == Black  
            && ImageData[findLine][CENTER_POINT+temp-1] == White  
            && ImageData[findLine][CENTER_POINT+temp-2] == White  
            && ImageData[findLine][CENTER_POINT+temp-3] == White)  
            {  
                rightLine[findLine] = CENTER_POINT+temp-1;  //�˴�ע���ұ߽����������Black-1;
                rightFindFlag = 1;  //��������Һ������ҵ� 
            }  
            if(leftFindFlag == 1 && rightFindFlag == 1)  
                break;  //�����ڶ���forѭ��
        }  
        // ��δ�ҵ��ĺ��߽��в�ȫ  
        if(leftFindFlag == 0)  
        {
             leftLine[findLine] = 0; 
             N_Lcount++;  //�Ҳ���������Լ�
        }    
        if(rightFindFlag == 0)  
        {
             rightLine[findLine] = CAMERA_W-1;
             N_Rcount++;  //�Ҳ����Ҽ����Լ�
        }      
        MidNumbers++;  //���߳��Լ�
        // �����߽��и�ֵ
        centerLine[findLine] = (leftLine[findLine]+rightLine[findLine])/2;
    }  
    /************** ʮ�к󣬸���ǰ����λ�ò��Һ���****************/ 
   for(findLine = CAMERA_H-11; findLine >= 2 && !MidEnd; findLine--) 
   {
        /******************************Ѱ����****************************/
         leftFindFlag =0;
         /****��������ȫ���й���****/
         if(N_Lcount>=5||Find_Lcount<5)
         {    
              M_B=0;  //�м�Ѱ�߻��߱�ԵѰ�߱�־
              for(temp = 0; temp < CENTER_POINT; temp++)  
              { 
                   if(leftFindFlag == 0  //ע���ǵ��������
                   && ImageData[findLine][CENTER_POINT-temp-1] == Black 
                   && ImageData[findLine][CENTER_POINT-temp] == White 
                   && ImageData[findLine][CENTER_POINT-temp+1] == White  
                   && ImageData[findLine][CENTER_POINT-temp+2] == White)  
                    {  
                         leftLine[findLine] = CENTER_POINT-temp;  
                         leftFindFlag = 1;  
                    }
                    if(leftFindFlag == 1)  //ע���ǵ��������
                    {
                         Find_Lcount++;  //�ҵ�����Ҽ���
                         break;
                    }
              }
         }
         else if(N_Lcount<5)
         {
               M_B=1;  //�м�Ѱ�߻��߱�ԵѰ�߱�־,��1�����ԵѰ��
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
         if(leftFindFlag == 0)//���߼�������ΪѰ�߻���
               N_Lcount++;
         else
               N_Lcount=0;
         if(N_Lcount>=5)
               Find_Lcount=0;
   /******************************************Ѱ����********************************/
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
         if(rightFindFlag == 0&&M_B==1)  //���Һ���
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
                  rightLine[findLine] =79;//ע������Ԫ�ظ���
         }
         else if(rightFindFlag == 0&&M_B==0)
               rightLine[findLine] =79;
         if(rightFindFlag == 0)//���߼�������ΪѰ�߻���
               N_Rcount++;
         else
               N_Rcount=0;
         if(N_Rcount>=5)
               Find_Rcount=0;
   /**************************************�жϽ�ֹ��*******************************/
        if ((ImageData[findLine][(leftLine[findLine]+rightLine[findLine])/2] == Black)||(leftLine[findLine]-rightLine[findLine]>1))
        {
             MidToBlackCount++;        //�����������
             if (MidToBlackCount >= 2)
             {
                  MidEnd = 1;
             }
        }
        else   MidToBlackCount = 0;
        if (!MidEnd)
        {
            MidNumbers++;  //���߳�
        }
     /*  if(leftLine[findLine]!=0&&rightLine[findLine]!=79&&leftLine[findLine+1]!=0&&rightLine[findLine+1]!=79
          &&(fabss(leftLine[findLine]-leftLine[findLine+1])>10||fabss(rightLine[findLine]-rightLine[findLine+1])>10))
                        break;*/
   }
  /*************��ʾ����************/
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

/************************ͼ���˲�****************************/
void Clear()
{
  	int i=0;
	unsigned char *p=0;//unsigned char һ���ֽ�
	
  	for (i =20;i < CAMERA_H;i++)
	{
		p = &ImageData[i][15];
		while (p < &ImageData[i][CAMERA_W-15])
		{
			if ((*(p-4) == White) && (*p == Black )&& (*(p+4) == White))//һ����������ռ4���ֽ�
			{
				*p = White;
			}			
			p++;//ָ�����ڴ����ƶ�1���ֽ�//8λ
		}
	}  
}
/*****************************ͨ������************************/
unsigned char Mid_End=0;
int Mid_Line_C;
void Mid_Line_process(void)
{     
      int i,n,Sub;
      int temp=0;
      Mid_Line_C=0;
      int L_and_R_flag=0,L_flag=0,R_flag=0,L_no_R_flag=0;
      unsigned char no_start=0,no_end=0,no_count=0;
                         //��    //�Ҷ�
      for(i=CAMERA_H-1;i>CAMERA_H-1-MidNumbers;i--)
      {
             if(leftLine[i]!=0&&rightLine[i]!=79)//���Ҷ�����
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
             else if(leftLine[i]==0&&rightLine[i]==79)//ȫ��
             {     
                    if(!no_count)
                      no_start=i;
                    no_count++;
                    no_end=i;
                    
                    if(L_and_R_flag==0&&L_flag==0&&R_flag==0&&L_no_R_flag==0)//ֱ�ӽ���ȫ��
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
              //�� 
              else if(leftLine[i] <= 0 && rightLine[i] < 79)  
              {  
                      if(L_and_R_flag==0&&L_flag==0&&R_flag==0&&L_no_R_flag==0)//ֱ�ӽ�������
                      {
                           midLine[i]=rightLine[i]-Mid_cor[i];//���ݽ�������ȷ������
                           L_and_R_flag=0;
                           L_flag=1;
                           R_flag=0;
                           L_no_R_flag=0;
                      }
                      else if(L_flag==1)//��һ������,����һ���ý�������
                      {
                           midLine[i]=rightLine[i]-Mid_cor[i];
                           L_and_R_flag=0;
                           L_flag=1;
                           R_flag=0;
                           L_no_R_flag=0;
                      }
                      else if(L_no_R_flag!=0)//��һ��ȫ��
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
                      else if(L_and_R_flag==1||L_flag==2)//��һ��ȫ�У�����һ����ƫ����
                      {
                          temp = midLine[i+1] + (rightLine[i] - rightLine[i+1]); // �����Һ��ߵ�ƫ������ȷ������   
                          if(temp <= 0)             
                              midLine[i] = 0;   // ���߳�����Χ������ѭ������¼����Ϊת����  
                          else  
                              midLine[i] = temp;  
                           L_and_R_flag=0;
                           L_flag=2;
                           R_flag=0;
                           L_no_R_flag=0;
                      }
                      else if(R_flag==1||R_flag==2)//��һ���Ҷ�
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
              // �Ҷ�
              else if(leftLine[i] > 0 && rightLine[i] == 79) //here
              { 
                 
                      if(L_and_R_flag==0&&L_flag==0&&R_flag==0&&L_no_R_flag==0)//ֱ�ӽ����Ҷ���
                      {
                           midLine[i]=leftLine[i]+Mid_cor[i];
                           L_and_R_flag=0;
                           L_flag=0;
                           R_flag=1;
                           L_no_R_flag=0;
                      }
                      else if(R_flag==1)//��һ�����Ҷ�
                      {
                           midLine[i]=leftLine[i]+Mid_cor[i];
                           L_and_R_flag=0;
                           L_flag=0;
                           R_flag=1;
                           L_no_R_flag=0;
                      }
                      else if(L_no_R_flag!=0)//��һ��ȫ��
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
                          temp = midLine[i+1] + (leftLine[i] - leftLine[i+1]); // ��������ߵ�ƫ������ȷ������   
                          if(temp>=79)             
                              midLine[i] = 0;   // ���߳�����Χ������ѭ������¼����Ϊת����  
                          else  
                              midLine[i] = temp;  
                           L_and_R_flag=0;
                           L_flag=0;
                           R_flag=2;
                           L_no_R_flag=0;
                      }
                      else if(L_flag==1||L_flag==2)//��һ����
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
/***************************�������˲�************************/
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
/************ͼ����***********/
void ImageProcess(void)
{   
    for(int i=0;i<=59;i++)
    {
        midLine[i]=255;
    }
    Clear();//�˺ڵ�
    Get_EdgeLine();//�����
    if(Crossroad_Judge()&&Loop_Flag_1==0)//ʮ��
    {
         cross_process();
    }
   if(!Crossroad_Judge())
      BEEP_OFF;
   if(!Crossroad_Judge()&&Loop_Flag_1==0)//Բ���ж�
    { 
      Loop_Judge(); 
    }
    if(Loop_Flag_1!=0)//Բ������
    {
        Loop_process1();  
    }
    if(!Crossroad_Judge()&&Loop_Flag_1==0)//��ͨ����
    {
         Mid_Line_process();
       //  AverageFilter();
          Turn();
    }
}



        /* �������� */  
/*#if (FIND_TYPE == TYPE_1)                 
          
        // ��ȫδ�ҵ������Һ���  
        // if(leftFindFlag == 0)  
        //  leftLine[findLine] = leftCount;  
        // if(rightFindFlag == 0)  
        //  rightLine[findLine] = rightCount;  
          
        /* �����߽��и�ֵ */  
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
        // ��ȫδ�ҵ������Һ���  
        // if(leftFindFlag == 0)  
        //  leftLine[findLine] = (leftCount < 0) ? 0 : ((leftCount > IMAGE_W-1) ? IMAGE_W-1 : leftCount);  
        // if(rightFindFlag == 0)  
        //  rightLine[findLine] = (rightCount < 0) ? 0 : ((rightCount > IMAGE_W-1) ? IMAGE_W-1 : rightCount);  
          
        /* �����߽��и�ֵ */  
        // ���Һ��߶�������ȡ���Һ�����ֵ��Ϊ����ֵ  
    /*    if(leftLine[findLine] > 0 && rightLine[findLine] < IMAGE_W-1)  
            centerLine[findLine] = (leftLine[findLine]+rightLine[findLine])/2;  
        // ����߳�����Χ  
        else if(leftLine[findLine] <= 0 && rightLine[findLine] < IMAGE_W-1)  
        {  
            // �����Һ��ߵ�ƫ������ȷ������  
            temp = centerLine[findLine+1] + (rightLine[findLine] - rightLine[findLine+1]);  
            // ������С���˷���ȫ����  
            //temp = createPoint(FIND_CENTER, findLine);  
            if(temp <= 0)  
            {  
                // ���߳�����Χ������ѭ������¼����Ϊת����  
                centerLine[findLine] = 0;  
                break;  
            }  
            else  
                centerLine[findLine] = temp;  
        }  
        // �Һ��߳�����Χ  
        else if(leftLine[findLine] > 0 && rightLine[findLine] >= IMAGE_W-1)  
        {  
            // ��������ߵ�ƫ������ȷ������  
            temp = centerLine[findLine+1] + (leftLine[findLine] - leftLine[findLine+1]);  
            // ������С���˷���ȫ����  
            //temp = createPoint(FIND_CENTER, findLine);  
            if(temp >= IMAGE_W-1)  
            {  
                // ���߳�����Χ������ѭ������¼����Ϊת����  
                centerLine[findLine] = IMAGE_W-1;  
                break;  
            }  
            else  
                centerLine[findLine] = temp;  
        }  
        // ���Һ��߶�������Χ  
        else  
        {  
            // ������С���˷���ȫ����  
            temp = createPoint(FIND_CENTER, findLine);  
            // ��������ƫ������ȫ����  
            //temp = centerLine[findLine+1] + (rightLine[findLine] - rightLine[findLine+1]);  
            if(temp <= 0)  
            {  
                // ���߳�����Χ������ѭ������¼����Ϊת����  
                centerLine[findLine] = 0;  
                break;  
            }  
            else if(temp >= IMAGE_W-1)  
            {  
                // ���߳�����Χ������ѭ������¼����Ϊת����  
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
      
    // ���һ��Ԫ��������¼ת����  
    centerLine[IMAGE_H] = (findLine < 0) ? 0 : findLine;  
#endif       
/* ���ͣ���� */  
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
            && carParams.carRunTime > 3000/ENCODE_TIME)      // С�����ܳ���������Ϊͣ����  
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
            carParams.passStopLine = PASS_STOP_LINE;        // С����Խ��ͣ����  
    }  
#endif  
      
    /* 
    // �� TFT �ϻ��Ƴ� 3 �� 
    for(temp = IMAGE_H-1; temp >= centerLine[IMAGE_H]; temp--) 
    { 
        guiDot(3*leftLine[temp], 3*temp, GREEN); 
        guiDot(3*rightLine[temp], 3*temp, RED); 
        guiDot(3*centerLine[temp], 3*temp, BLUE); 
    } 
     
      
    return (int16 *)centerLine;  
} */  
  

