#ifndef _PID_H_
#define _PID_H


struct PID_servo
{
	float Goal; 	    // Ŀ��
	float Input; 		// ������
	float KT;			// �ܱ���

	float KP; 			// ������1ϵ��
	float I_MAX_P;      // ���ּ�����
	float I_MAX_N;      // ���ּ��޸�
	float KD; 			// ΢��ϵ��

	float Output;		// �����
	float Output_Min;   // �����λ��С��
	float Output_Max;   // �����λ�����

	float ErrorSum; 	// ����
	float LastError; 	// ��һ�����
  //float PreError;
} PID_SERVO;



struct PID_fuzzy
{

        float Goal; 	    // Ŀ��
	float P_Output;		// P�����
	float D_Output;		// D�����
        
	float P_Output_Min;   // P�����λ��С��
	float P_Output_Max;   // P�����λ�����
	float D_Output_Min;   // D�����λ��С��
	float D_Output_Max;   // D�����λ�����

	float LastError; 	// ��һ�����
		
	float Ke;//Error_Max;    //ƫ�������ֵ
	float Kc;//DError_Max;   //ƫ��΢�����ֵ
        
        float KPu;
        float KDu;
        
        float E;                 //ƫ��
        float Ec;                //ƫ��仯��

	float KP;//DP_Max;       //�����仯��
	float KD;//DD_Max;       //΢�ֱ仯��
	
} PID_FUZZY;


//��������
extern void PID_Init(void);
#endif