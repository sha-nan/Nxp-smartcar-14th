/************************************************************************************
* @author��
* @date :
* @fuction name��FUZZY_PID_CONTROL
* @fuction description�� ģ������Ӧ�����㷨��Ϊ�˷������Ĭ��e��ec��[-3,3]���䣬
* ����ı�e��ec��Χ����������������(Ke��Kec=N/emax)����������(Ku=umax/N)�����´����
*�����������������������Լ���Ȩƽ������ģ����PID����λ��ʽPID�㷨��
*************************************************************************************/
/*
   ģ��PID����
*/
#include "Fuzzy.h"

//p�����
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

//d�����
unsigned int rule_d[7][7]={ 
//���仯�� -3,-2,-1, 0, 1, 2, 3    // ���     
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
/*�����U����ֵ������ 0     1   2     3   4    5    6  */
//float UFF_P[7] = {0,2,4,6,8,10,12};
//float UFF_D[7] = {0,1,2,3,4,5,6};
float UFF_P[7] = {0,2,4,6,8,10,12};
float UFF_D[7] = {0,0.5,1.0,1.5,2.0,2.5,3.0};

/*�����޷�*/
const float PMAX = 100;
const float PMIN = -100; 
const float DMAX = 10; 
const float DMIN = -10;
//��������
//const float Kp = 6;
//const float Kd = 6;
const int16 FMAX =1; //����ֵ������ֵ

/*****************
P �������ܳ�Ѱ�������ֵ
D ���ֵ�ı仯��
ģ�������㷨ͨ���������������Լ�����������ǰӦ�õ�����Kp,Ki,Kdֵ
*******************/
float Fuzzy_Kp(float P, float D)   //ģ����������,����Kpֵ
{
  uint16 PF[2];  //ƫ�������� 
  uint16 DF[2];  //ƫ��ֵ�仯��������
  uint16 UF[4];  //���ֵ��������
  int16 Pn, Dn;      //�Ǳ�
  int16 Un[4];   //��Ӧ����Pֵ
  int32 temp1, temp2;
  float out ;      //���ֵ�ľ�ȷ�� 
  float Un_out[4]; 
   
  //����޷�
  if (P < PMIN)
    P = PMIN;
  if (P > PMAX)
    P = PMAX;
  if (D < DMIN)
    D = DMIN;
  if (D > DMAX)
    D = DMAX;
 /*�������Է�ʽ����*/
//  P = (int16) Kp * (double) (P - PMIN) / (PMAX - PMIN);  //��һ����0~500
//  D = (int16) Kd * (double) (D - DMIN) / (DMAX - DMIN); //��һ����0~300 
  

//  P = (int16) Kp * (double) P;  
//  D = (int16) Kd * (double) D; 
//  P = (int16) Kp/ (double) P;  //��һ����0~500
//  D = (int16) Kd/ (double) D; //��һ����0~300 
//  printf("P=%4.4f\n",P); 
/*����E Ec��ָ������ֵ�����Ч������*/ 
  //Ѱ��e�������� ��ģ������
  if (P > -PFF[6] && P < PFF[6])    //E�ı仯�ڷ�ֵ��
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
  else if(P<=-PFF[6])  //�޷� 
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

//  if (P > PFF[0] && P < PFF[6])    //E�ı仯�ڷ�ֵ��
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
//  else if(P<=PFF[0])  //�޷� 
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
  
  //Ѱ��eC�������� ��ģ������
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
  else if(D<=-DFF[6])  //�޷� 
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
//  else if(D<=-DFF[6])  //�޷� 
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
  
  //ʹ����Χ�Ż���Ĺ����rule[7][7]
  //���ֵʹ��13����������,����ֵ��UFF[7]ָ��
  //һ�㶼���ĸ�������Ч
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
/*ͬ���������������ֵ���*/ 
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
  
  
  /*���ķ���ģ��*/ 
  /*Un[]ԭֵΪ�������������ţ�ת��Ϊ��������ֵ*/ 
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
P �������ܳ�Ѱ�������ֵ
D ���ֵ�ı仯��
ģ�������㷨ͨ���������������Լ�����������ǰӦ�õ�����Kp,Ki,Kdֵ
*******************/
float Fuzzy_Kd(float P, float D)   //ģ����������,����Kdֵ
{
  uint16 PF[2];  //ƫ�������� 
  uint16 DF[2];  //ƫ��ֵ�仯��������
  uint16 UF[4];  //���ֵ��������
  int16 Pn,Dn;
  int16 Un[4];
  int32 temp1, temp2;
  float out ;  //���ֵ�ľ�ȷ�� 
  float Un_out[4]; 
   
  //����޷�
//  if (P < PMIN)
//    P = PMIN;
//  if (P > PMAX)
//    P = PMAX;
//  if (D < DMIN)
//    D = DMIN;
//  if (D > DMAX)
//    D = DMAX;
  
//  P = (int16) Kp * (double) (P - PMIN) / (PMAX - PMIN);  //��һ����0~500
//  D = (int16) Kd * (double) (D - DMIN) / (DMAX - DMIN); //��һ����0~300 
//  P = (int16) Kp * (double) P;  
//  D = (int16) Kd * (double) D; 
//  P = (int16) Kp/ (double) P;  //��һ����0~500
//  D = (int16) Kd/ (double) D; //��һ����0~300   
  /*����E Ec��ָ������ֵ�����Ч������*/ 
  //Ѱ��e�������� 
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
//  else if(P<=-PFF[6])  //�޷� 
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

  if (P > PFF[0] && P < PFF[6])    //E�ı仯�ڷ�ֵ��
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
  else if(P<=PFF[0])  //�޷� 
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

  
  //Ѱ��eC�������� 
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
//  else if(D<=-DFF[6])  //�޷� 
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
  else if(D<=-DFF[6])  //�޷� 
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

  //ʹ����Χ�Ż���Ĺ����rule[7][7]
  //���ֵʹ��13����������,����ֵ��UFF[7]ָ��
  //һ�㶼���ĸ�������Ч
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
	  	  
  /*ͬ���������������ֵ���*/ 
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
  
  
  /*���ķ���ģ��*/ 
  /*Un[]ԭֵΪ�������������ţ�ת��Ϊ��������ֵ*/ 
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


