#ifndef __IMU_H__
#define __IMU_H__



#include <stdint.h>


#ifndef DEBUG_ON_PC
#include "contiki.h"
PROCESS_NAME(imu_process);
#endif


#define VELOCITY_BUF_SIZE 2048
#define Accelerometer_Scale 2048.0f
#define Gyroscope_Scale 16.4f
#define Pi 3.14159f
#define G 9.81f


#define PREPROCESS_BUF_SIZE 64

//当数值连续多少个采样点保持稳定才认为进入稳定态或退出稳定态
#define PREPROCESS_STATE_CHANGE_CONDITION_LENGTH 16


typedef struct {
	float Vx;
	float Vy;
	float Vz;
} IMU_Velocity_Frame;

typedef struct {
	float Ax;
	float Ay;
	float Az;
	float Gx;
	float Gy;
	float Gz;
	float StationaryChannelCounter;
} IMU_Preprocess_Frame;

typedef struct {
	float Ax;
	float Ay;
	float Az;
	float Gx;
	float Gy;
	float Gz;
	uint32_t PointCount;
}InitialCalibrationAverageCalculator;




typedef struct
{

#define IMU_RAW_DATA_BUFFER_LENGTH 20

	float Raw_Buffer[IMU_RAW_DATA_BUFFER_LENGTH * 6];
	int8_t Raw_Buffer_Head;
	int8_t Raw_Buffer_Tail;






	//用于判断是否为静止状态的滤波器状态变量
	// y = filter( x )
	char LPF_X1;

	//用于判断当前状态的各个通道阈值 Xmin,Xmax,Ymin,Ymax,Zmin,Zmax
	float ChannelThresHold_G[6];
	float ChannelThresHold_A[6];


	uint8_t IMU_New_Data_To_Be_Process;
	uint8_t In_Static_State;


	//最终输出位置
	float PosX, PosY, PosZ;


	IMU_Preprocess_Frame PreprocessBuf[PREPROCESS_BUF_SIZE];

	IMU_Velocity_Frame VelocityBuf[VELOCITY_BUF_SIZE];

	uint16_t PreprocessBuf_Pointer;
	uint16_t VelocityBuf_Pointer;



	float q[4];
	float Int_Error[3];
	float Kp, Ki;
	float SamplePeriod;

	InitialCalibrationAverageCalculator  InitCaliAverage;
	
	//用于减小Z方向漂移，如果两次步态之间Z方向变化小于阈值，则认为Z方向没有移动，强行令两次Z相同.但是有可能影响到上下楼梯检测
	float Z_threadhold;
	float Z_old_val;

}IMU_Handler;


void resetIMU(IMU_Handler* hIMU, float SamplePeriod);
void updateIMU(IMU_Handler* hIMU, float* Gyroscope, float* Accelerometer);
void IMU_new_data(IMU_Handler* hIMU, float* Gyroscope, float* Accelerometer);
#ifndef DEBUG_ON_PC
void IMU_Process(IMU_Handler* hIMU);
#endif

void ResetInitialCalibration(IMU_Handler* hIMU);
void DoInitialAverageValueCalculate(IMU_Handler* hIMU, float* Gyroscope, float* Accelerometer);
void SetThreshold(IMU_Handler* hIMU);
void RotateIMUCoordinateByAccAndGyr(IMU_Handler* hIMU, float* Gyroscope, float* Accelerometer);




#endif
