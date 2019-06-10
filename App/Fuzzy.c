/************************************************************************************
* @author：
* @date :
* @fuction name：FUZZY_PID_CONTROL
* @fuction description： 模糊自适应控制算法，为了方便测试默认e、ec在[-3,3]区间，
* 如需改变e、ec范围，需引入量化因子(Ke、Kec=N/emax)、缩放因子(Ku=umax/N)。以下代码采
*用三角隶属函数求隶属度以及加权平均法解模糊，PID采用位置式PID算法。
*************************************************************************************/
/*
   模糊PID控制
*/
#include "Fuzzy.h"

//p规则表
unsigned int rule_p[7][7]={ 
//-2 -1  0  1  2    ec  e
 {6, 5, 4, 3, 2, 0, 0,}, //   -3   
 {5, 4, 3, 2, 1, 0, 1,}, //   -2 
 {4, 3, 2, 1, 0, 1, 2,}, //   -1 
 {3, 2, 1, 0, 1, 2, 3,}, //    0 
 {2, 1, 0, 1, 2, 3, 4,}, //    1 
 {1, 0, 1, 2, 3, 4, 5,}, //    2 
 {0, 0, 2, 3, 4, 5, 6 }  //    3 
};   

//d规则表
unsigned int rule_d[7][7]={ 
//误差变化率 -3,-2,-1, 0, 1, 2, 3    // 误差     
 {2, 2, 6, 5, 6, 4, 2,},   //   -3   
 {1, 2, 5, 4, 3, 1, 0,},   //   -2 
 {0, 1, 3, 3, 1, 1, 0,},   //   -1 
 {0, 1, 1, 1, 1, 1, 0,},   //    0 
 {0, 0, 0, 0, 0, 0, 0,},   //    1 
 {5, 1, 1, 1, 1, 1, 1,},   //    2 
 {6, 4, 4, 3, 3, 1, 1 }    //    3 
};  

float PFF[7] = {0,150,300,450,600,750,900}; 
float DFF[7] = {0,100,200,300,400,500,600};
//float PFF[7] = {-75,-50,-25,0,25,50,75}; 
//float DFF[7] = {-7.5,-5,- 2.5,0, 2.5,5,7.5};
/*输出量U语言值特征点 0     1   2     3   4    5    6  */
//float UFF_P[7] = {0,2,4,6,8,10,12};
//float UFF_D[7] = {0,1,2,3,4,5,6};
float UFF_P[7] = {0,2,4,6,8,10,12};
float UFF_D[7] = {0,0.5,1.0,1.5,2.0,2.5,3.0};

/*输入限幅*/
const float PMAX = 100;
const float PMIN = -100; 
const float DMAX = 10; 
const float DMIN = -10;
//量化因子
//const float Kp = 6;
//const float Kd = 6;
const int16 FMAX =1; //语言值的满幅值

/*****************
P 代表智能车寻迹的误差值
D 误差值的变化率
模糊控制算法通过这两个计算量以及规则表给出当前应该调整的Kp,Ki,Kd值
*******************/
float Fuzzy_Kp(float P, float D)   //模糊运算引擎,返回Kp值
{
  uint16 PF[2];  //偏差隶属度 
  uint16 DF[2];  //偏差值变化率隶属度
  uint16 UF[4];  //输出值的隶属度
  int16 Pn, Dn;      //角标
  int16 Un[4];   //对应附近P值
  int32 temp1, temp2;
  float out ;      //输出值的精确量 
  float Un_out[4]; 
   
  //误差限幅
  if (P < PMIN)
    P = PMIN;
  if (P > PMAX)
    P = PMAX;
  if (D < DMIN)
    D = DMIN;
  if (D > DMAX)
    D = DMAX;
 /*输入线性方式量化*/
//  P = (int16) Kp * (double) (P - PMIN) / (PMAX - PMIN);  //归一化到0~500
//  D = (int16) Kd * (double) (D - DMIN) / (DMAX - DMIN); //归一化到0~300 
  

//  P = (int16) Kp * (double) P;  
//  D = (int16) Kd * (double) D; 
//  P = (int16) Kp/ (double) P;  //归一化到0~500
//  D = (int16) Kd/ (double) D; //归一化到0~300 
//  printf("P=%4.4f\n",P); 
/*根据E Ec的指定语言值获得有效隶属度*/ 
  //寻找e的隶属度 （模糊化）
  if (P > -PFF[6] && P < PFF[6])    //E的变化在幅值内
  {
      if(P<=-PFF[5])     
      {
	 Pn=-5;
         PF[0]=(int)(FMAX*((float)(-PFF[5]-P)/(PFF[6]-PFF[5])));
      } 
      else if(P<=-PFF[4])   
      {
         Pn=-4;
         PF[0]=(int)(FMAX*((float)(-PFF[4]-P)/(PFF[5]-PFF[4])));
      } 
      else if(P<=-PFF[3])   
      {
	 Pn=-3;
	 PF[0]=(int)(FMAX*((float)(-PFF[3]-P)/(PFF[4]-PFF[3])));
      } 
      else if(P<=-PFF[2])   
      {
         Pn=-2; 
         PF[0]=(int)(FMAX*((float)(-PFF[2]-P)/(PFF[3]-PFF[2])));
      } 
      else if(P<=-PFF[1]) 
      {
         Pn=-1; 
	 PF[0]=(int)(FMAX*((float)(-PFF[1]-P)/(PFF[2]-PFF[1])));
      } 
      else if(P<=PFF[0])   
      {
         Pn=0; 
	 PF[0]=(int)(FMAX*((float)( PFF[0]-P)/(PFF[1]-PFF[0])));
      }
      else if(P<=PFF[1])   
      {
         Pn=1; 
	 PF[0]=(int)(FMAX*((float)(PFF[1]-P)/(PFF[1]-PFF[0])));
      } 
      else if(P<=PFF[2])   
      {
         Pn=2; 
	 PF[0]=(int)(FMAX*((float)(PFF[2]-P)/(PFF[2]-PFF[1])));
      }  
      else if(P<=PFF[3])   
      {
         Pn=3; 
	 PF[0]=(int)(FMAX *((float)(PFF[3]-P)/(PFF[3]-PFF[2])));
      }
      else if(P<=PFF[4])   
      {
         Pn=4; 
	 PF[0]=(int)(FMAX*((float)(PFF[4]-P)/(PFF[4]-PFF[3])));
      }
      else if(P<=PFF[5])   
      {
         Pn=5; 
	 PF[0]=(int)(FMAX*((float)(PFF[5]-P)/(PFF[5]-PFF[4])));
      }
      else if(P<=PFF[6])   
      {
         Pn=6; 
	 PF[0]=(int)(FMAX*((float)(PFF[6]-P)/(PFF[6]-PFF[5])));
      }
  }
  else if(P<=-PFF[6])  //限幅 
  {
      Pn=-5;   
      PF[0]=FMAX;
  } 
  else if(P>=PFF[6])   
  {
      Pn=6;   
      PF[0]=0;
  } 
  PF[1] = (uint16)(FMAX - PF[0]);

//  if (P > PFF[0] && P < PFF[6])    //E的变化在幅值内
//  {
//      
//      if(P<=PFF[1])   
//      {
//         Pn=-2; 
//	 PF[0]=(int)(FMAX*((float)(PFF[1]-P)/(PFF[1]-PFF[0])));
//      } 
//      else if(P<=PFF[2])   
//      {
//         Pn=-2; 
//	 PF[0]=(int)(FMAX*((float)(PFF[2]-P)/(PFF[2]-PFF[1])));
//      }  
//      else if(P<=PFF[3])   
//      {
//         Pn=0; 
//	 PF[0]=(int)(FMAX *((float)(PFF[3]-P)/(PFF[3]-PFF[2])));
//      }
//      else if(P<=PFF[4])   
//      {
//         Pn=1; 
//	 PF[0]=(int)(FMAX*((float)(PFF[4]-P)/(PFF[4]-PFF[3])));
//      }
//      else if(P<=PFF[5])   
//      {
//         Pn=2; 
//	 PF[0]=(int)(FMAX*((float)(PFF[5]-P)/(PFF[5]-PFF[4])));
//      }
//      else if(P<=PFF[6])   
//      {
//         Pn=3; 
//	 PF[0]=(int)(FMAX*((float)(PFF[6]-P)/(PFF[6]-PFF[5])));
//      }
//  }
//  else if(P<=PFF[0])  //限幅 
//  {
//      Pn=-2;   
//      PF[0]=FMAX;
//  } 
//  else if(P>=PFF[6])   
//  {
//      Pn=3;   
//      PF[0]=0;
//  } 
//  PF[1] = (uint16)(FMAX - PF[0]);  
  
  //寻找eC的隶属度 （模糊化）
  if (D > -DFF[6] && D < DFF[6])
  {
      if(D<=-DFF[5])     
      {
	 Dn=-5;         
         DF[0]=(int)(FMAX*((float)(-DFF[5]-D)/(DFF[6]-DFF[5])));
      } 
      else if(D<=-DFF[4])   
      {
         Dn=-4;
         DF[0]=(int)(FMAX*((float)(-DFF[4]-D)/(DFF[5]-DFF[4])));
      } 
      else if(D<=-DFF[3])   
      {
	 Dn=-3;
	 DF[0]=(int)(FMAX*((float)(-DFF[3]-D)/(DFF[4]-DFF[3])));
      } 
      else if(D<=-DFF[2])   
      {
         Dn=-2; 
         DF[0]=(int)(FMAX*((float)(-DFF[2]-D)/(DFF[3]-DFF[2])));
      } 
      else if(D<=-DFF[1]) 
      {
         Dn=-1; 
	 DF[0]=(int)(FMAX*((float)(-DFF[1]-D)/(DFF[2]-DFF[1])));
      } 
      else if(D<=DFF[0])   
      {
         Dn=0; 
	 DF[0]=(int)(FMAX*((float)( DFF[0]-D)/(DFF[1]-DFF[0])));
      }
      else if(D<=DFF[1])   
      {
         Dn=1; 
	 DF[0]=(int)(FMAX*((float)(DFF[1]-D)/(DFF[1]-DFF[0])));
      } 
      else if(D<=DFF[2])   
      {
         Dn=2; 
	 DF[0]=(int)(FMAX*((float)(DFF[2]-D)/(DFF[2]-DFF[1])));
      }  
      else if(D<=DFF[3])   
      {
         Dn=3; 
	 DF[0]=(int)(FMAX*((float)(DFF[3]-D)/(DFF[3]-DFF[2])));
      }
      else if(D<=DFF[4])   
      {
         Dn=4; 
	 DF[0]=(int)(FMAX*((float)(DFF[4]-D)/(DFF[4]-DFF[3])));
      }
      else if(D<=DFF[5])   
      {
         Dn=5; 
	 DF[0]=(int)(FMAX*((float)(DFF[5]-D)/(DFF[5]-DFF[4])));
      }
      else if(D<=DFF[6])   
      {
         Dn=6; 
	 DF[0]=(int)(FMAX*((float)(DFF[6]-D)/(DFF[6]-DFF[5])));
      }
  }
  else if(D<=-DFF[6])  //限幅 
  {
      Dn=-5;   
      DF[0]=FMAX;
  } 
  else if(D>=DFF[6])   
  {
      Dn=6;   
      DF[0]=0;
  } 
  DF[1] = (uint16)(FMAX - DF[0]);  

//  if (D > DFF[0] && D < DFF[6])
//  {
//      if(D<=DFF[1])   
//      {
//         Dn=-2; 
//	 DF[0]=(int)(FMAX*((float)(DFF[1]-D)/(DFF[1]-DFF[0])));
//      } 
//      else if(D<=DFF[2])   
//      {
//         Dn=-2; 
//	 DF[0]=(int)(FMAX*((float)(DFF[2]-D)/(DFF[2]-DFF[1])));
//      }  
//      else if(D<=DFF[3])   
//      {
//         Dn=0; 
//	 DF[0]=(int)(FMAX*((float)(DFF[3]-D)/(DFF[3]-DFF[2])));
//      }
//      else if(D<=DFF[4])   
//      {
//         Dn=1; 
//	 DF[0]=(int)(FMAX*((float)(DFF[4]-D)/(DFF[4]-DFF[3])));
//      }
//      else if(D<=DFF[5])   
//      {
//         Dn=2; 
//	 DF[0]=(int)(FMAX*((float)(DFF[5]-D)/(DFF[5]-DFF[4])));
//      }
//      else if(D<=DFF[6])   
//      {
//         Dn=3; 
//	 DF[0]=(int)(FMAX*((float)(DFF[6]-D)/(DFF[6]-DFF[5])));
//      }
//  }
//  else if(D<=-DFF[6])  //限幅 
//  {
//      Dn=-2;   
//      DF[0]=FMAX;
//  } 
//  else if(D>=DFF[6])   
//  {
//      Dn=1;   
//      DF[0]=0;
//  } 
//  DF[1] = (uint16)(FMAX - DF[0]);
  
  //使用误差范围优化后的规则表rule[7][7]
  //输出值使用13个隶属函数,中心值由UFF[7]指定
  //一般都是四个规则有效
  Un[0]=rule_p[Pn+2][Dn+2]; 
  Un[1]=rule_p[Pn+3][Dn+2]; 
  Un[2]=rule_p[Pn+2][Dn+3];   
  Un[3]=rule_p[Pn+3][Dn+3]; 
  
  if(PF[0]<=DF[0])
    UF[0]=PF[0];  
  else 
    UF[0]=DF[0]; 
	  
  if(PF[1]<=DF[0])
    UF[1]=PF[1];
  else 
    UF[1]=DF[0];
	 
  if(PF[0]<=DF[1])
    UF[2]=PF[0]; 
  else 
    UF[2]=DF[1]; 
	 
  if(PF[1]<=DF[1])
    UF[3]=PF[1]; 
  else 
    UF[3]=DF[1]; 
/*同隶属函数输出语言值求大*/ 
  if(Un[0]==Un[1])
  {
    if(UF[0]>UF[1])
      UF[1]=0;
    else 
      UF[0]=0;
  } 
  
  if(Un[0]==Un[2])
  {
    if(UF[0]>UF[2])
      UF[2]=0;
    else 
      UF[0]=0;
  } 
  
  if(Un[0]==Un[3])
  {
    if(UF[0]>UF[3])
       UF[3]=0;
    else
      UF[0]=0;
  } 
  
  if(Un[1]==Un[2])
  {
    if(UF[1]>UF[2])
      UF[2]=0;
    else
      UF[1]=0;
  } 
  
  if(Un[1]==Un[3])
  {
    if(UF[1]>UF[3])
      UF[3]=0;
    else
      UF[1]=0;
  } 
  
  if(Un[2]==Un[3])
  {
    if(UF[2]>UF[3])
      UF[3]=0;
    else 
      UF[2]=0;
  } 
  
  
  /*重心法反模糊*/ 
  /*Un[]原值为输出隶属函数标号，转换为隶属函数值*/ 
  if(Un[0]>=0)
    Un_out[0]=UFF_P[Un[0]];
  else 
    Un_out[0]=-UFF_P[-Un[0]]; 
	  
  if(Un[1]>=0)
    Un_out[1]=UFF_P[Un[1]];
  else
    Un_out[1]=-UFF_P[-Un[1]]; 
	  
  if(Un[2]>=0)
    Un_out[2]=UFF_P[Un[2]];
  else
    Un_out[2]=-UFF_P[-Un[2]]; 
	  
  if(Un[3]>=0)
    Un_out[3]=UFF_P[Un[3]];
  else  
    Un_out[3]=-UFF_P[-Un[3]];  
  
  temp1=(int)(UF[0]*Un_out[0]+UF[1]*Un_out[1]+UF[2]*Un_out[2]+UF[3]*Un_out[3]); 
  temp2=UF[0]+UF[1]+UF[2]+UF[3]; 
  out=(float) temp1/temp2; 
  
  return out;
}

/*****************
P 代表智能车寻迹的误差值
D 误差值的变化率
模糊控制算法通过这两个计算量以及规则表给出当前应该调整的Kp,Ki,Kd值
*******************/
float Fuzzy_Kd(float P, float D)   //模糊运算引擎,返回Kd值
{
  uint16 PF[2];  //偏差隶属度 
  uint16 DF[2];  //偏差值变化率隶属度
  uint16 UF[4];  //输出值的隶属度
  int16 Pn,Dn;
  int16 Un[4];
  int32 temp1, temp2;
  float out ;  //输出值的精确量 
  float Un_out[4]; 
   
  //误差限幅
//  if (P < PMIN)
//    P = PMIN;
//  if (P > PMAX)
//    P = PMAX;
//  if (D < DMIN)
//    D = DMIN;
//  if (D > DMAX)
//    D = DMAX;
  
//  P = (int16) Kp * (double) (P - PMIN) / (PMAX - PMIN);  //归一化到0~500
//  D = (int16) Kd * (double) (D - DMIN) / (DMAX - DMIN); //归一化到0~300 
//  P = (int16) Kp * (double) P;  
//  D = (int16) Kd * (double) D; 
//  P = (int16) Kp/ (double) P;  //归一化到0~500
//  D = (int16) Kd/ (double) D; //归一化到0~300   
  /*根据E Ec的指定语言值获得有效隶属度*/ 
  //寻找e的隶属度 
//if (P > -PFF[6] && P < PFF[6])
//  {
//      if(P<=-PFF[5])     
//      {
//	 Pn=-5;
//         PF[0]=(int)(FMAX*((float)(-PFF[5]-P)/(PFF[6]-PFF[5])));
//      } 
//      else if(P<=-PFF[4])   
//      {
//         Pn=-4;
//         PF[0]=(int)(FMAX*((float)(-PFF[4]-P)/(PFF[5]-PFF[4])));
//      } 
//      else if(P<=-PFF[3])   
//      {
//	 Pn=-3;
//	 PF[0]=(int)(FMAX*((float)(-PFF[3]-P)/(PFF[4]-PFF[3])));
//      } 
//      else if(P<=-PFF[2])   
//      {
//         Pn=-2; 
//         PF[0]=(int)(FMAX*((float)(-PFF[2]-P)/(PFF[3]-PFF[2])));
//      } 
//      else if(P<=-PFF[1]) 
//      {
//         Pn=-1; 
//	 PF[0]=(int)(FMAX*((float)(-PFF[1]-P)/(PFF[2]-PFF[1])));
//      } 
//      else if(P<=PFF[0])   
//      {
//         Pn=0; 
//	 PF[0]=(int)(FMAX*((float)(-PFF[0]-P)/(PFF[1]-PFF[0])));
//      }
//      else if(P<=PFF[1])   
//      {
//         Pn=1; 
//	 PF[0]=(int)(FMAX*((float)(PFF[1]-P)/(PFF[1]-PFF[0])));
//      } 
//      else if(P<=PFF[2])   
//      {
//         Pn=2; 
//	 PF[0]=(int)(FMAX*((float)(PFF[2]-P)/(PFF[2]-PFF[1])));
//      }  
//      else if(P<=PFF[3])   
//      {
//         Pn=3; 
//	 PF[0]=(int)(FMAX*((float)(PFF[3]-P)/(PFF[3]-PFF[2])));
//      }
//      else if(P<=PFF[4])   
//      {
//         Pn=4; 
//	 PF[0]=(int)(FMAX*((float)(PFF[4]-P)/(PFF[5]-PFF[4])));
//      }
//  }
//  else if(P<=-PFF[6])  //限幅 
//  {
//      Pn=-5;   
//      PF[0]=FMAX;
//  } 
//  else if(P>=PFF[6])   
//  {
//      Pn=6;   
//      PF[0]=0;
//  } 
//  PF[1] = (uint16)(FMAX - PF[0]);

  if (P > PFF[0] && P < PFF[6])    //E的变化在幅值内
  {
      
      if(P<=PFF[1])   
      {
         Pn=-2; 
	 PF[0]=(int)(FMAX*((float)(PFF[1]-P)/(PFF[1]-PFF[0])));
      } 
      else if(P<=PFF[2])   
      {
         Pn=-2; 
	 PF[0]=(int)(FMAX*((float)(PFF[2]-P)/(PFF[2]-PFF[1])));
      }  
      else if(P<=PFF[3])   
      {
         Pn=0; 
	 PF[0]=(int)(FMAX *((float)(PFF[3]-P)/(PFF[3]-PFF[2])));
      }
      else if(P<=PFF[4])   
      {
         Pn=1; 
	 PF[0]=(int)(FMAX*((float)(PFF[4]-P)/(PFF[4]-PFF[3])));
      }
      else if(P<=PFF[5])   
      {
         Pn=2; 
	 PF[0]=(int)(FMAX*((float)(PFF[5]-P)/(PFF[5]-PFF[4])));
      }
      else if(P<=PFF[6])   
      {
         Pn=3; 
	 PF[0]=(int)(FMAX*((float)(PFF[6]-P)/(PFF[6]-PFF[5])));
      }
  }
  else if(P<=PFF[0])  //限幅 
  {
      Pn=-2;   
      PF[0]=FMAX;
  } 
  else if(P>=PFF[6])   
  {
      Pn=3;   
      PF[0]=0;
  } 
  PF[1] = (uint16)(FMAX - PF[0]);   

  
  //寻找eC的隶属度 
//  if (D > -DFF[6] && D < DFF[6])
//  {
//      if(D<=-DFF[5])     
//      {
//	 Dn=-5;
//         DF[0]=(int)(FMAX*((float)(-DFF[5]-D)/(DFF[6]-DFF[5])));
//      } 
//      else if(D<=-DFF[4])   
//      {
//         Dn=-4;
//         DF[0]=(int)(FMAX*((float)(-DFF[4]-D)/(DFF[5]-DFF[4])));
//      } 
//      else if(D<=-DFF[3])   
//      {
//	 Dn=-3;
//	 DF[0]=(int)(FMAX*((float)(-DFF[3]-D)/(DFF[4]-DFF[3])));
//      } 
//      else if(D<=-DFF[2])   
//      {
//         Dn=-2; 
//         DF[0]=(int)(FMAX*((float)(-DFF[2]-D)/(DFF[3]-DFF[2])));
//      } 
//      else if(D<=-DFF[1]) 
//      {
//         Dn=-1; 
//	 DF[0]=(int)(FMAX*((float)(-DFF[1]-D)/(DFF[2]-DFF[1])));
//      } 
//      else if(D<=DFF[0])   
//      {
//         Dn=0; 
//	 DF[0]=(int)(FMAX*((float)( DFF[0]-D)/(DFF[1]-DFF[0])));
//      }
//      else if(D<=DFF[1])   
//      {
//         Dn=1; 
//	 DF[0]=(int)(FMAX*((float)(DFF[1]-D)/(DFF[1]-DFF[0])));
//      } 
//      else if(D<=DFF[2])   
//      {
//         Dn=2; 
//	 DF[0]=(int)(FMAX*((float)(DFF[2]-D)/(DFF[2]-DFF[1])));
//      }  
//      else if(D<=DFF[3])   
//      {
//         Dn=3; 
//	 DF[0]=(int)(FMAX*((float)(DFF[3]-D)/(DFF[3]-DFF[2])));
//      }
//      else if(D<=DFF[4])   
//      {
//         Dn=4; 
//	 DF[0]=(int)(FMAX*((float)(DFF[4]-D)/(DFF[4]-DFF[3])));
//      }
//      else if(D<=DFF[5])   
//      {
//         Dn=5; 
//	 DF[0]=(int)(FMAX*((float)(DFF[5]-D)/(DFF[5]-DFF[4])));
//      }
//      else if(D<=DFF[6])   
//      {
//         Dn=6; 
//	 DF[0]=(int)(FMAX*((float)(DFF[6]-D)/(DFF[6]-DFF[5])));
//      }      
//  }
//  else if(D<=-DFF[6])  //限幅 
//  {
//      Dn=-5;   
//      DF[0]=FMAX;
//  } 
//  else if(D>=DFF[6])   
//  {
//      Dn=6;   
//      DF[0]=0;
//  } 
//  DF[1] = (uint16)(FMAX - DF[0]); 

  if (D > DFF[0] && D < DFF[6])
  {
      if(D<=DFF[1])   
      {
         Dn=-2; 
	 DF[0]=(int)(FMAX*((float)(DFF[1]-D)/(DFF[1]-DFF[0])));
      } 
      else if(D<=DFF[2])   
      {
         Dn=-2; 
	 DF[0]=(int)(FMAX*((float)(DFF[2]-D)/(DFF[2]-DFF[1])));
      }  
      else if(D<=DFF[3])   
      {
         Dn=0; 
	 DF[0]=(int)(FMAX*((float)(DFF[3]-D)/(DFF[3]-DFF[2])));
      }
      else if(D<=DFF[4])   
      {
         Dn=1; 
	 DF[0]=(int)(FMAX*((float)(DFF[4]-D)/(DFF[4]-DFF[3])));
      }
      else if(D<=DFF[5])   
      {
         Dn=2; 
	 DF[0]=(int)(FMAX*((float)(DFF[5]-D)/(DFF[5]-DFF[4])));
      }
      else if(D<=DFF[6])   
      {
         Dn=3; 
	 DF[0]=(int)(FMAX*((float)(DFF[6]-D)/(DFF[6]-DFF[5])));
      }
  }
  else if(D<=-DFF[6])  //限幅 
  {
      Dn=-2;   
      DF[0]=FMAX;
  } 
  else if(D>=DFF[6])   
  {
      Dn=1;   
      DF[0]=0;
  } 
  DF[1] = (uint16)(FMAX - DF[0]);

  //使用误差范围优化后的规则表rule[7][7]
  //输出值使用13个隶属函数,中心值由UFF[7]指定
  //一般都是四个规则有效
  Un[0]=rule_d[Pn+2][Dn+2]; 
  Un[1]=rule_d[Pn+3][Dn+2]; 
  Un[2]=rule_d[Pn+2][Dn+3];   
  Un[3]=rule_d[Pn+3][Dn+3]; 
  
   if(PF[0]<=DF[0])
	UF[0]=PF[0];  
  else 
	UF[0]=DF[0]; 
	  
  if(PF[1]<=DF[0])
	  UF[1]=PF[1];
  else 
	  UF[1]=DF[0];
	   
  if(PF[0]<=DF[1])
	  UF[2]=PF[0]; 
  else 
	  UF[2]=DF[1]; 
	  
  if(PF[1]<=DF[1])
	  UF[3]=PF[1]; 
  else 
	  UF[3]=DF[1]; 
	  	  
  /*同隶属函数输出语言值求大*/ 
  if(Un[0]==Un[1])
  {
      if(UF[0]>UF[1])
	 UF[1]=0;
      else 
         UF[0]=0;
  } 
  
  if(Un[0]==Un[2])
  {
      if(UF[0]>UF[2])
         UF[2]=0;
      else 
         UF[0]=0;
  } 
  
  if(Un[0]==Un[3])
  {
      if(UF[0]>UF[3])
	 UF[3]=0;
      else
	 UF[0]=0;
  } 
  
  if(Un[1]==Un[2])
  {
      if(UF[1]>UF[2])
	 UF[2]=0;
      else
	 UF[1]=0;
  } 
  
  if(Un[1]==Un[3])
  {
      if(UF[1]>UF[3])
	 UF[3]=0;
      else 
         UF[1]=0;
  } 
  
  if(Un[2]==Un[3])
  {
      if(UF[2]>UF[3])
	 UF[3]=0;
      else 
         UF[2]=0;
  } 
  
  
  /*重心法反模糊*/ 
  /*Un[]原值为输出隶属函数标号，转换为隶属函数值*/ 
  if(Un[0]>=0)
      Un_out[0]=UFF_D[Un[0]];
  else 
      Un_out[0]=-UFF_D[-Un[0]]; 
	  
  if(Un[1]>=0)
      Un_out[1]=UFF_D[Un[1]];
  else 
      Un_out[1]=-UFF_D[-Un[1]]; 
	  
  if(Un[2]>=0)
      Un_out[2]=UFF_D[Un[2]];
  else
      Un_out[2]=-UFF_D[-Un[2]]; 
	  
  if(Un[3]>=0)
      Un_out[3]=UFF_D[Un[3]];
  else  
      Un_out[3]=-UFF_D[-Un[3]];  
  
  temp1=(int)(UF[0]*Un_out[0]+UF[1]*Un_out[1]+UF[2]*Un_out[2]+UF[3]*Un_out[3]); 
  temp2=UF[0]+UF[1]+UF[2]+UF[3]; 
  out=(float)temp1/temp2; 
  
  return out;
}


