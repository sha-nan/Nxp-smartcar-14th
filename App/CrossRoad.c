#include "CrossRoad.h"
#include "common.h"
#include "Init.h"
#include "Public.h"

extern int World_X[60][80];
extern uint16_t SteerPidCal(float excursion);
/*********************************************************************/
float cross_Error2=0.0;
int cross_Error=0;

float Get_cross_Error2(unsigned char start,unsigned char end,float midpos)
{
	unsigned char i=0;
	int  Black_Sum=0;
	int weightSum = 0;
        int X,Y;
	float TemError = 0.0;
        for(i = 0; i < 60; i++) 
	{	
                Y=i;
                X=midLine[i];
                if(midLine[i]!=255) 
		{
                   Black_Sum += World_X[Y][X];//*LineWeight[i];
		   weightSum += 1;//LineWeight[i];
                }

	}
	TemError = Black_Sum*1.0/weightSum - midpos;
	
	/*if(TemError > 80.0)    //限位
	{
		TemError = 80.0;
	}
	if(TemError < -80.0)
	{
		TemError = -80.0;	
	}*/
    //    printf("TemError=%f\n",TemError);
	return TemError;
}
/***********************************************************************/
float k,d;

uint8 road_len_min[60] = {
	9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
	9, 9, 9, 9, 9, 9, 9, 9, 10, 11, 12, 13,
	14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
	24, 25, 26, 27, 28, 28, 29, 29, 29, 29, 29,
	30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30
};
uint8 road_len_max[60] = {
	24, 24, 24, 24, 24, 24, 25, 26, 27, 28,
	29, 29, 31, 32, 33, 34, 34, 36, 36, 37,
	38, 39, 40, 41, 41, 43, 44, 44, 46, 46,
	47, 48, 49, 49, 51, 51, 52, 53, 53, 54,
	55, 56, 56, 57, 58, 59, 59, 61, 61, 62,
	63, 64, 64, 66, 66, 67, 68, 69, 69, 69
};
/*************************************/
int division45(float kfloat)
{//四舍五入
	return (int)(kfloat + 0.5);
}
/*************判断十字************/
uint8 Crossroad_Judge(void)
{
    short i;//行查找
    //short Cross_Find_Line;//找到十字标志行
    uint8 cross_flag_judge =0;//十字判断标志
    uint8 cross_flag=0;//十字标志
    unsigned char cross_count=0;//全白行计数
    unsigned char Ricount=0;
    unsigned char Licount=0;
    unsigned char Ristart=0;
    unsigned char Listart=0;
    unsigned char Riend=0;
    unsigned char Liend=0;
    
    if(MidNumbers>50)
    {
        for(i=CAMERA_H-1;i>=CAMERA_H-MidNumbers;i--)
        {
            if(leftLine[i]==0&&rightLine[i]==79)
            {
                cross_count++;
            }
            else cross_count=0;
            if(cross_count>=10)
            {
               cross_flag_judge =1;
               break;
            }
        }
    }
    else 
    {
         cross_flag=0;
         return cross_flag;
    }
    if(cross_flag_judge==1)
    {
          for(i=CAMERA_H-1;i>=CAMERA_H-MidNumbers;i--)
          {
                if(leftLine[i]==0)
                {
                    if(!Licount) Listart=i;
                    
                    Licount++;
                    Liend=i;
                    if(Licount>7&&abs(Listart-Liend-2)>Licount)
                    {
                       Listart=Liend-Licount/2;
                    }
                }
                if(rightLine[i]==79)
                {
                    if(!Ricount) Ristart=i;
                    
                    Ricount++;
                    Riend=i;
                    if(Ricount>7&&abs(Ristart-Riend-2)>Ricount)
                    {
                        Ristart = Riend-Ricount/2;
                    }
                }
          }
          if((Licount>20&&Ricount>20)||(Licount>15&&Ricount>25)||(Licount>25&&Ricount>15))
            cross_flag=1;
    }
   return cross_flag;
}

/**************************************/
//赛道边界检测显示
void LeastSquareMethod(PosType *data, uint8_t size)
{
	uint8 i;
	int xsum = 0;
	int x2sum = 0;
	int ysum = 0;
	int xysum = 0;

	for (i = 0; i < size; i++)
	{
		xsum += data[i].x;
		x2sum += (data[i].x)*(data[i].x);
		ysum += data[i].y;
		xysum += (data[i].x)*(data[i].y);
	}

	if (size*x2sum - xsum*xsum == 0)
	{
		k = 0;
		d = 0;
	}
	k = (float)(size*xysum - xsum*ysum) / (float)(size*x2sum - xsum*xsum);//（最小二乘法）
	d = (float)(x2sum*ysum - xysum*xsum) / (float)(size*x2sum - xsum*xsum);
}
/*******************************************************/
uint8 catch_road_j() 
{
	uint8 max_j = 0, max_cnt = 0, cnt;   //（20到60全白列）
	for (uint8 j = 20; j < 60; j++)
        {
		cnt = 0;
		for (uint8 i = 0; i < CAMERA_H; i++)
                {
			if (ImageData[i][j] == White)	cnt++;
		}

		if (cnt > max_cnt) 
                {
			if (ImageData[59][j] == White&&ImageData[58][j] == White&&ImageData[58][j - 1] == White&&ImageData[58][j + 1] == White) 
                        {
				max_j = j;
				max_cnt = cnt;
			}
		}
	}

	return max_j;
}
/**************************************************/
void cross_process()
{       BEEP_ON;
	uint16 status[CAMERA_H][3];
	memset(status, 0, sizeof(status));

	uint8 basic_j = catch_road_j();
	uint8 i, j;


	uint8 len = 0;
	for (i = CAMERA_H - 1; i > 4; i--)
	{
		len = 0;
		j = basic_j;
		while (ImageData[i][j] == White&&j > 0) { j--; }
		status[i][1] = j;//（左边界）
		j = basic_j;
		while (ImageData[i][j] == White&&j < CAMERA_W) { j++; }
		status[i][2] = j;//（右边界）
		len = status[i][2] - status[i][1];//（宽度）
		////("\nline%d: %d <=    %d   <= %d", i, road_len_min[i], len, road_len_max[i]);
		if (len >= road_len_min[i] && len <= road_len_max[i])
		{
			if (((status[i + 2][1] - status[i][1]) > 3 || (status[i][2] - status[i + 2][2]) > 3) && status[i + 2][0] != 0 && i < 56)
			{
				status[i][0] = 0;         //（满足十字）
			}
			else
			{
				status[i][0] = len;
				j = basic_j;
				while (ImageData[i][j] == White&&j > 0) { j--; }
				j = basic_j;
				while (ImageData[i][j] == White&&j < CAMERA_W - 1) { j++; }
				midLine[i] = (status[i][1] + status[i][2]) / 2;
			}
		}
	}
	for (i = CAMERA_H - 1; i > 4; i--)
	{
		if (status[i][0] == 0)   //（寻找=0的行，即十字标志）
                {
			uint8 it = i, iup = i, idown = 0;
			while (status[it][0] == 0 && abs(it - i) < 40 && it > 0) { it--; }//向上找//（寻找不=0）
			if (status[it][0] != 0)
                        {
				iup = it;//（向上找到的）
				it = i;
				while (status[it][0] == 0 && it < CAMERA_H - 1 && abs(it - i) < 32) { it++; }//（向下寻找）
				if (status[it][0] != 0)
                                {
					idown = it;//（向下找到的）
				}
				else 
                                {
					idown = 59;
					status[59][1] = 15;
					status[59][2] = 65;

				}
				status[i][1] = status[iup][1] * (idown - i) + status[idown][1] * (i - iup);
				status[i][1] = division45((float)status[i][1] / (idown - iup));
				if (status[i][1] < 0)status[i][1] = 0;

				status[i][2] = status[iup][2] * (idown - i) + status[idown][2] * (i - iup);
				status[i][2] = division45((float)status[i][2] / (idown - iup));
				if (status[i][2] > 79)status[i][2] = 79;
				int q1, q2;
				q1 = status[i][1];
				q2 = status[i][2];
				status[i][0] = q2 - q1;

				////////////////set center//////////////

				midLine[i] = (q1 + q2) / 2;
			}
			else {//向上找不到了
			}

		}

	}
	int no_result_flag = 1;
	for (int p = 10; p < 40; p++) 
        {
		if (status[p][0] != 0)
                {
			no_result_flag = 0;
			break;
		}
	}
	if (no_result_flag) 
        {
		for (i = CAMERA_H - 1; i > 4; i--)
		{
			len = status[i][2] - status[i][1];
			if (len >= road_len_min[i] && len <= 40)
			{
				if (((status[i + 2][1] - status[i][1]) > 3 || (status[i][2] - status[i + 2][2]) > 3) && status[i + 2][0] != 0 && i < 56)
				{
					status[i][0] = 0;
				}
				else
				{
					status[i][0] = len;

					j = basic_j;
					while (ImageData[i][j] == White&&j > 0) { j--; }

					j = basic_j;
					while (ImageData[i][j] == White&&j < CAMERA_W - 1) { j++; }

					midLine[i] = (status[i][1] + status[i][2]) / 2;

				}
			}


		}
		int row = 59;
		PosType row_middle[60]; int cnt = 0;
		for (row = 59; row > 0; row--)
		{
			if (status[row][0] > 0)
			{

				row_middle[cnt].x = row;
				row_middle[cnt].y = midLine[row];
				cnt++;

			}
		}
		int row_min_normal = row;
		if (cnt == 0)return;

		LeastSquareMethod(row_middle, cnt);

		for (row = 59; row > 0; row--) {
			midLine[row] = k*row + d;
		}
	}
	else if (status[20][0] == 0)
        {

		int row = 59;
		PosType row_middle[60]; int cnt;
		for (cnt = 0; status[row][0] > 0 && row > 0; row--, cnt++)
		{
			row_middle[cnt].x = row;
			row_middle[cnt].y = midLine[row];
		}
		int row_min_normal = row;
		if (cnt == 0)return;

		LeastSquareMethod(row_middle, cnt);

		for (row = row_min_normal; row > 0; row--) 
                {
			midLine[row] = k*row + d;
		}

	}
    cross_Error2 = Get_cross_Error2(0,0,0);
    cross_Error = SteerPidCal(cross_Error2);
    FTM_PWM_Duty(FTM1, FTM_CH0,cross_Error );
}



