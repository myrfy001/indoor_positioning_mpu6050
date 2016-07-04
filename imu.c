#include "imu.h"
#include <math.h>
#include <stdio.h>



#ifndef DEBUG_ON_PC

PROCESS(imu_process, "IMU Process");
process_event_t imu_new_data_event;
#endif


#ifdef DEBUG_ON_PC
extern FILE *fpp, *fp_posout, *fp_rawout;
#endif // 0











void quaternProd(float* ab, float* a, float* b)
{
	ab[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
	ab[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
	ab[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
	ab[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

void quaternConj(float* a)
{
	a[1] = -a[1];
	a[2] = -a[2];
	a[3] = -a[3];
}


void SetThreshold(IMU_Handler* hIMU)
{
	hIMU->ChannelThresHold_A[0] = hIMU->InitCaliAverage.Ax - 0.2;
	hIMU->ChannelThresHold_A[1] = hIMU->InitCaliAverage.Ax + 0.2;
	hIMU->ChannelThresHold_A[2] = hIMU->InitCaliAverage.Ay - 0.2;
	hIMU->ChannelThresHold_A[3] = hIMU->InitCaliAverage.Ay + 0.2;
	hIMU->ChannelThresHold_A[4] = hIMU->InitCaliAverage.Az - 0.2;
	hIMU->ChannelThresHold_A[5] = hIMU->InitCaliAverage.Az + 0.2;


	hIMU->ChannelThresHold_G[0] = -20;
	hIMU->ChannelThresHold_G[1] = 20;
	hIMU->ChannelThresHold_G[2] = -20;
	hIMU->ChannelThresHold_G[3] = 20;
	hIMU->ChannelThresHold_G[4] = -20;
	hIMU->ChannelThresHold_G[5] = 20;
	
	hIMU->Z_threadhold = 0.05;
}


void ResetInitialCalibration(IMU_Handler* hIMU)
{
	hIMU->InitCaliAverage.PointCount = 0;
}


void RotateIMUCoordinateByAccAndGyr(IMU_Handler* hIMU, float* Gyroscope, float* Accelerometer)
{
	int i;
	float Gyro[3], Accelero[3];
	hIMU->Kp = 50;

	Gyro[0] = Gyroscope[0]; Gyro[1] = Gyroscope[1]; Gyro[2] = Gyroscope[2];

	for (i = 0; i < 20; i++)
	{
		//updateIMU会修改Acc值
		Accelero[0] = Accelerometer[0]; Accelero[1] = Accelerometer[1]; Accelero[2] = Accelerometer[2];
		updateIMU(hIMU, Gyro, Accelero);
	}

}

void DoInitialAverageValueCalculate(IMU_Handler* hIMU, float* Gyroscope, float* Accelerometer)
{
	hIMU->InitCaliAverage.Ax = hIMU->InitCaliAverage.Ax * hIMU->InitCaliAverage.PointCount + Accelerometer[0];
	hIMU->InitCaliAverage.Ay = hIMU->InitCaliAverage.Ay * hIMU->InitCaliAverage.PointCount + Accelerometer[1];
	hIMU->InitCaliAverage.Az = hIMU->InitCaliAverage.Az * hIMU->InitCaliAverage.PointCount + Accelerometer[2];

	hIMU->InitCaliAverage.Gx = hIMU->InitCaliAverage.Gx * hIMU->InitCaliAverage.PointCount + Gyroscope[0];
	hIMU->InitCaliAverage.Gy = hIMU->InitCaliAverage.Gy * hIMU->InitCaliAverage.PointCount + Gyroscope[1];
	hIMU->InitCaliAverage.Gz = hIMU->InitCaliAverage.Gz * hIMU->InitCaliAverage.PointCount + Gyroscope[2];


	hIMU->InitCaliAverage.PointCount++;

	hIMU->InitCaliAverage.Ax /= hIMU->InitCaliAverage.PointCount;
	hIMU->InitCaliAverage.Ay /= hIMU->InitCaliAverage.PointCount;
	hIMU->InitCaliAverage.Az /= hIMU->InitCaliAverage.PointCount;
	hIMU->InitCaliAverage.Gx /= hIMU->InitCaliAverage.PointCount;
	hIMU->InitCaliAverage.Gy /= hIMU->InitCaliAverage.PointCount;
	hIMU->InitCaliAverage.Gz /= hIMU->InitCaliAverage.PointCount;


}


void resetIMU(IMU_Handler* hIMU, float SamplePeriod)
{
	int i;

	hIMU->SamplePeriod = SamplePeriod;

	hIMU->q[0] = 1; hIMU->q[1] = 0; hIMU->q[2] = 0; hIMU->q[3] = 0;

	hIMU->Int_Error[0] = 0; hIMU->Int_Error[1] = 0; hIMU->Int_Error[2] = 0;
	hIMU->PosX = 0; hIMU->PosY = 0; hIMU->PosZ = 0;

	for (i = 0; i < PREPROCESS_BUF_SIZE; i++)
	{
		hIMU->PreprocessBuf[i].Az = Accelerometer_Scale;
	}
	hIMU->LPF_X1 = 0;
}




void IMU_new_data(IMU_Handler* hIMU, float* Gyroscope, float* Accelerometer)
{
	int t;
	t = (hIMU->Raw_Buffer_Head + 1) % IMU_RAW_DATA_BUFFER_LENGTH;
	if (t == hIMU->Raw_Buffer_Tail)
	{
#ifndef DEBUG_ON_PC
		process_post(&imu_process,imu_new_data_event,hIMU);
		printf("!!!!!!IMU_RAW_BUFFER_FULL\n");
#endif
		return;
	}
	hIMU->Raw_Buffer[t * 6 + 0] = Gyroscope[0];
	hIMU->Raw_Buffer[t * 6 + 1] = Gyroscope[1];
	hIMU->Raw_Buffer[t * 6 + 2] = Gyroscope[2];
	hIMU->Raw_Buffer[t * 6 + 3] = Accelerometer[0];
	hIMU->Raw_Buffer[t * 6 + 4] = Accelerometer[1];
	hIMU->Raw_Buffer[t * 6 + 5] = Accelerometer[2];
	hIMU->Raw_Buffer_Head = t;
	
	
#ifndef DEBUG_ON_PC
	process_post(&imu_process,imu_new_data_event,hIMU);
#endif

}

#ifdef DEBUG_ON_PC
void IMU_Process(IMU_Handler* hIMU)
#else
PROCESS_THREAD(imu_process, ev, data)
#endif
{
	
#ifndef DEBUG_ON_PC	
	static IMU_Handler* hIMU ;
#endif
	
	
	static int i;
	static char tmp_ChannelCounter;
	static float Filter_tmp1;

	static float Gyro[3], Accelero[3];

	static float MY_q[4], tmp_vector1[4], tmp_vector2[4];

	static int StationaryChannelCounterForThreshold;//用于记录有多少通道满足阈值限制要求

	static int16_t TMP1;

	//用于补偿速度误差的系数
	static float Velocity_Error_K_x, Velocity_Error_K_y, Velocity_Error_K_z;

#ifndef DEBUG_ON_PC	
	PROCESS_BEGIN();
	
	imu_new_data_event = process_alloc_event();
	
	while(1)
	{
		PROCESS_YIELD();//通过这个强行使函数返回一次，更新ev的值……否则由于ev不变，下一行条件会永远成立
		PROCESS_YIELD_UNTIL(ev == imu_new_data_event);
		hIMU = (IMU_Handler*)data;
		
PROCESS_NEXT_DATA:
		
#endif
		
		
		

		
		if (hIMU->Raw_Buffer_Head == hIMU->Raw_Buffer_Tail)
		{
#ifdef DEBUG_ON_PC	
			return;
#else
			continue;
#endif
		}
		
		hIMU->PreprocessBuf_Pointer = (hIMU->PreprocessBuf_Pointer + 1) % PREPROCESS_BUF_SIZE;

		i = (hIMU->Raw_Buffer_Tail + 1) % IMU_RAW_DATA_BUFFER_LENGTH;


		hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gx = hIMU->Raw_Buffer[i * 6 + 0] - hIMU->InitCaliAverage.Gx;
		hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gy = hIMU->Raw_Buffer[i * 6 + 1] - hIMU->InitCaliAverage.Gy;
		hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gz = hIMU->Raw_Buffer[i * 6 + 2] - hIMU->InitCaliAverage.Gz;
		hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ax = hIMU->Raw_Buffer[i * 6 + 3];
		hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ay = hIMU->Raw_Buffer[i * 6 + 4];
		hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Az = hIMU->Raw_Buffer[i * 6 + 5];
		hIMU->Raw_Buffer_Tail = i;


	#ifdef DEBUG_ON_PC
		fprintf(fpp, "%f,%f,%f,%f,%f,%f,", hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gx, hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gy, hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gz, hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ax, hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ay, hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Az);
	#endif

	#define PreprocessBuf_Offset(n) (hIMU->PreprocessBuf[(hIMU->PreprocessBuf_Pointer + PREPROCESS_BUF_SIZE + (n)) % PREPROCESS_BUF_SIZE ])


		tmp_ChannelCounter = 0;
		if ((hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gx > hIMU->ChannelThresHold_G[0]) && (hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gx < hIMU->ChannelThresHold_G[1])) tmp_ChannelCounter++;
		if ((hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gy > hIMU->ChannelThresHold_G[2]) && (hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gy < hIMU->ChannelThresHold_G[3])) tmp_ChannelCounter++;
		if ((hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gz > hIMU->ChannelThresHold_G[4]) && (hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gz < hIMU->ChannelThresHold_G[5])) tmp_ChannelCounter++;
		if ((hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ax > hIMU->ChannelThresHold_A[0]) && (hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ax < hIMU->ChannelThresHold_A[1])) tmp_ChannelCounter++;
		if ((hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ay > hIMU->ChannelThresHold_A[2]) && (hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ay < hIMU->ChannelThresHold_A[3])) tmp_ChannelCounter++;
		if ((hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Az > hIMU->ChannelThresHold_A[4]) && (hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Az < hIMU->ChannelThresHold_A[5])) tmp_ChannelCounter++;


		//一阶IIR滤波器
		//a(1)*y(n) = b(1)*x(n) + b(2)*x(n - 1) + ... + b(nb + 1)*x(n - nb)
		//	         -a(2)*y(n - 1) - ... - a(na + 1)*y(n - na)
		//
		//y(n) = b(1)*x(n) + b(2)*x(n-1) - a(2)*y(n-1)
		//     



		//低通滤波
		//HPF_Y1是高通滤波后的结果，在此作为x(n)
		hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].StationaryChannelCounter = tmp_ChannelCounter * 0.0245 + hIMU->LPF_X1 * (0.0245) - PreprocessBuf_Offset(-1).StationaryChannelCounter *(-0.9510);
		hIMU->LPF_X1 = tmp_ChannelCounter;




		//循环队列队尾的元素是用来更新IMU的元素

		Gyro[0] = hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gx / 180 * Pi;
		Gyro[1] = hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gy / 180 * Pi;
		Gyro[2] = hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gz / 180 * Pi;
		Accelero[0] = hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ax * G;
		Accelero[1] = hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ay * G;
		Accelero[2] = hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Az * G;

		//通过PreprocessBuf_Offset访问当前用来更新IMU的元素之后的第PREPROCESS_STATE_CHANGE_CONDITION_LENGTH个元素
		//如果
		if (hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].StationaryChannelCounter >= 5)
		{
			hIMU->In_Static_State = 1;
		}
		else
		{
			hIMU->In_Static_State = 0;
		}
		
		

	#ifdef DEBUG_ON_PC
		fprintf(fpp, "%d\n", (hIMU->In_Static_State == 1 ? 1 : -1));
	#endif


		if (hIMU->In_Static_State)
		{
			hIMU->Kp = 3.0f;
		}
		else
		{
			hIMU->Kp = 0.0f;
		}


		//由于updateIMU会修改Accelero的值，所以这里提前拷贝出一份
		tmp_vector1[0] = 0;
		tmp_vector1[1] = Accelero[0];
		tmp_vector1[2] = Accelero[1];
		tmp_vector1[3] = Accelero[2];



		updateIMU(hIMU, Gyro, Accelero);


	#ifdef DEBUG_ON_PC
		fprintf(fp_rawout, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", Gyro[0], Gyro[1], Gyro[2], tmp_vector1[0], tmp_vector1[1], tmp_vector1[2], hIMU->q[0], hIMU->q[1], hIMU->q[2], hIMU->q[3]);
	#endif

		if (hIMU->In_Static_State)
		{

			//如果VelocityBuf_Pointer不是0，说明上一次刚从运动状态进入静止状态
			if (hIMU->VelocityBuf_Pointer != 0)
			{
				Velocity_Error_K_x = hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer - 1].Vx / (hIMU->VelocityBuf_Pointer);
				Velocity_Error_K_y = hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer - 1].Vy / (hIMU->VelocityBuf_Pointer);
				Velocity_Error_K_z = hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer - 1].Vz / (hIMU->VelocityBuf_Pointer);

				hIMU->Z_old_val = hIMU->PosZ;
				for (i = 0; i<hIMU->VelocityBuf_Pointer; i++)
				{
					
#ifndef DEBUG_ON_PC						
					if ( ( i % 100 ) == 0)   PROCESS_YIELD();
#endif
					
					hIMU->PosX += ((hIMU->VelocityBuf[i].Vx - Velocity_Error_K_x * i) * hIMU->SamplePeriod);
					hIMU->PosY += ((hIMU->VelocityBuf[i].Vy - Velocity_Error_K_y * i) * hIMU->SamplePeriod);
					hIMU->PosZ += ((hIMU->VelocityBuf[i].Vz - Velocity_Error_K_z * i) * hIMU->SamplePeriod);
					
	#ifdef DEBUG_ON_PC
					fprintf(fp_posout, "%f,%f,%f,%f,%f,%f\n", hIMU->PosX, hIMU->PosY, hIMU->PosZ, (hIMU->VelocityBuf[i].Vx - Velocity_Error_K_x * i), (hIMU->VelocityBuf[i].Vy - Velocity_Error_K_y * i), (hIMU->VelocityBuf[i].Vz - Velocity_Error_K_z * i));
	#endif			
				}
				
				if (   fabs(hIMU->Z_old_val - hIMU->PosZ ) < hIMU->Z_threadhold   )
				{
					hIMU->PosZ = hIMU->Z_old_val;
				}
				
	#ifdef DEBUG_ON_PC
				fprintf(fp_posout, "%f,%f,%f,%f,%f,%f\n", hIMU->PosX, hIMU->PosY, hIMU->PosZ, 0.0f, 0.0f, 0.0f);
	#endif

				//清零表示上一步已经处理完成，没有必要进行下一次处理，同时为存储下一步数据做准备
				hIMU->VelocityBuf_Pointer = 0;
	#ifdef DEBUG_ON_PC
				printf("%f,%f,%f\n", hIMU->PosX, hIMU->PosY, hIMU->PosZ);
	#endif

			}
			else
			{
	#ifdef DEBUG_ON_PC
				fprintf(fp_posout, "%f,%f,%f,%f,%f,%f\n", hIMU->PosX, hIMU->PosY, hIMU->PosZ, 0.0f, 0.0f, 0.0f);
	#endif
			}
		}
		else  //处于运动状态
		{


			MY_q[0] = hIMU->q[0];
			MY_q[1] = hIMU->q[1];
			MY_q[2] = hIMU->q[2];
			MY_q[3] = hIMU->q[3];

			quaternProd(tmp_vector2, MY_q, tmp_vector1);
			quaternConj(MY_q);
			quaternProd(tmp_vector1, tmp_vector2, MY_q);

			//把Z轴方向上的重力加速度减掉
			tmp_vector1[3] = tmp_vector1[3] - G;

			//积分加速度得到速度
			if (hIMU->VelocityBuf_Pointer != 0)
			{
				hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer].Vx = hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer - 1].Vx + tmp_vector1[1] * hIMU->SamplePeriod;
				hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer].Vy = hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer - 1].Vy + tmp_vector1[2] * hIMU->SamplePeriod;
				hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer].Vz = hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer - 1].Vz + tmp_vector1[3] * hIMU->SamplePeriod;
			}
			else
			{
				hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer].Vx = tmp_vector1[1] * hIMU->SamplePeriod;
				hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer].Vy = tmp_vector1[2] * hIMU->SamplePeriod;
				hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer].Vz = tmp_vector1[3] * hIMU->SamplePeriod;
			}


			if (hIMU->VelocityBuf_Pointer < VELOCITY_BUF_SIZE - 1)
			{
				hIMU->VelocityBuf_Pointer++;
			}
			else
			{
	#ifdef DEBUG_ON_PC
				printf("overflow\n");
	#endif
				hIMU->VelocityBuf_Pointer = 0;
			}

		}
		
		
		
		
		
		

#ifndef DEBUG_ON_PC	
		goto PROCESS_NEXT_DATA;
		
	}//while(1);
#endif

#undef PreprocessBuf_Offset
	
	PROCESS_END();
	
}

void updateIMU(IMU_Handler* hIMU, float* Gyroscope, float* Accelerometer)
{
	float norm = 1;
	float v[3];
	float error[3];
	float Ref[4];
	float pDot[4];

	norm = sqrt(Accelerometer[0] * Accelerometer[0] + Accelerometer[1] * Accelerometer[1] + Accelerometer[2] * Accelerometer[2]);

	Accelerometer[0] /= norm;
	Accelerometer[1] /= norm;
	Accelerometer[2] /= norm;

	v[0] = 2 * (hIMU->q[1] * hIMU->q[3] - hIMU->q[0] * hIMU->q[2]);
	v[1] = 2 * (hIMU->q[0] * hIMU->q[1] + hIMU->q[2] * hIMU->q[3]);
	v[2] = 2 * (hIMU->q[0] * hIMU->q[0] - hIMU->q[1] * hIMU->q[1] - hIMU->q[2] * hIMU->q[2] + hIMU->q[3] * hIMU->q[3]);

	//cross product
	error[0] = Accelerometer[2] * v[1] - Accelerometer[1] * v[2];
	error[1] = Accelerometer[0] * v[2] - Accelerometer[2] * v[0];
	error[2] = Accelerometer[1] * v[0] - Accelerometer[0] * v[1];

	hIMU->Int_Error[0] += error[0];
	hIMU->Int_Error[1] += error[1];
	hIMU->Int_Error[2] += error[2];


	//Apply feedback terms
	Ref[1] = Gyroscope[0] - (hIMU->Kp * error[0] + hIMU->Ki * hIMU->Int_Error[0]);
	Ref[2] = Gyroscope[1] - (hIMU->Kp * error[1] + hIMU->Ki * hIMU->Int_Error[1]);
	Ref[3] = Gyroscope[2] - (hIMU->Kp * error[2] + hIMU->Ki * hIMU->Int_Error[2]);

	Ref[0] = 0;
	quaternProd(pDot, hIMU->q, Ref);
	pDot[0] *= 0.5;
	pDot[1] *= 0.5;
	pDot[2] *= 0.5;
	pDot[3] *= 0.5;

	hIMU->q[0] += pDot[0] * hIMU->SamplePeriod;
	hIMU->q[1] += pDot[1] * hIMU->SamplePeriod;
	hIMU->q[2] += pDot[2] * hIMU->SamplePeriod;
	hIMU->q[3] += pDot[3] * hIMU->SamplePeriod;

	norm = sqrt(hIMU->q[0] * hIMU->q[0] + hIMU->q[1] * hIMU->q[1] + hIMU->q[2] * hIMU->q[2] + hIMU->q[3] * hIMU->q[3]);
	hIMU->q[0] /= norm;
	hIMU->q[1] /= norm;
	hIMU->q[2] /= norm;
	hIMU->q[3] /= norm;

}

