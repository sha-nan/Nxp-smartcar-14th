#ifndef _CONTROL_H_
#define _CONTROL_H_

extern int16 Error;
extern int16 Car_Sudu;

extern void Turn_PD(float Input);
extern int16 Speed_PID(int16 Goal,int16 Input);

extern uint8 Run_Flag;


extern void AD_Date_analyse();

typedef struct speed_PID 
  {
        float Goal; // �趨Ŀ��Desired value
	float Kp; // ��������Proportional Const
	float Ki; // ���ֳ���Integral Const
	float Kd; // ΢�ֳ���Derivative Const
	float LastError; // Error[-1]	
	float PrevError; // Error[-2]
        float ErrorSum; // Sums of Errors
        float I_MAX_sum;      // ���ּ�����
        float I_MIN_sum;      // ���ּ��޸�
        float output;		// �����
        float output_Min;   // �����λ��С��
        float output_Max;   // �����λ�����
  }PID;

#endif