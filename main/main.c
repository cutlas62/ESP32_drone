//**********************************************************************
//Includes
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "definitions.h"

#include "MadgwickFilter.h"
//#include "MahonyFilter.h"

#include "mpu9250.h"
#include "m_pwm.h"
#include "m_i2c.h"

//**********************************************************************
//Defines
//#define DEBUG				//Define for debugging verbose
//#define TIME_MEASURE		//Define for time measurements

/*
 * 0-RED[32]     1-BLUE[23]
 *     x            x
 *      x    ^     x
 *       x   |    x
 *        x     x
 *         x   x
 *          x x
 *         x   x
 *        x     x
 *       x       x
 *      x         x
 *     x           x
 * 2-BLUE[26]     3-RED[16]
 */

//**********************************************************************
//Global Variables

float magCalibration[3];	//Store magnetometer calibration data

int16_t acceData[3];	//Store the 16-bit signed raw accelerometer data
int16_t gyroData[3];	//Store the 16-bit signed raw gyroscope data
int16_t magData[3];		//Store the 16-bit signed raw magnetometer data

float acceRealData[3];		//Store the accelerometer converted data
float gyroRealData[3];		//Store the gyroscope converted data
float magRealData[3];		//Store the magnetometer converted data
float tempRealData;		//Store the temperature data in Celsius

float aRes, gRes, mRes;		//Store the sensors resolution

uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS;
uint8_t Mmode = 0x06;	// 100Hz continuous magnetometer read

float acceBias[3];			//Store the accelerometer bias after initialization
float gyroBias[3];			//Store the gyroscope bias after initialization

float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;
float deltat = 0.0f;
float roll, pitch, yaw;
float iRoll, iPitch, iYaw;
float last_eRoll, last_ePitch, last_eYaw;
float targetRoll, targetPitch, targetYaw;
float eInt[3] = { 0.0f, 0.0f, 0.0f };		//Mahony integral error

struct timeval tv;
uint32_t Now;
uint32_t lastTime;
uint32_t PID_IterationTime;
uint32_t PID_Now;
uint32_t PID_LastTime;
TickType_t xLastWakeTime;


int16_t duty[4];

#ifdef TIME_MEASURE
uint32_t filter_time = 0;
uint32_t sensor_read_time = 0;
#endif
//**********************************************************************
//Auxiliary Functions
uint32_t getMicros() {
	uint32_t ret;
	gettimeofday(&tv, NULL);
	ret = 1000000 * tv.tv_sec + tv.tv_usec;
	return ret;
}



//**********************************************************************
//Main App
void app_main() {

	I2C_Init();
	vTaskDelay(1000 / portTICK_RATE_MS);


	MPU9250_GetAres();
	MPU9250_GetGres();
	MPU9250_GetMres();
	MPU9250_Calibrate(&gyroBias[0], &acceBias[0]);
	MPU9250_Init();

	AK8963_Init(&magCalibration[0]);

	PWM_Init();

	MPU9250_Read_All();
	targetYaw = yaw;
	targetPitch = 0;
	targetRoll = 0;

	while (1) {
		/*

#ifdef TIME_MEASURE
		uint32_t bundle_start;
		gettimeofday(&tv, NULL);
		bundle_start = 1000000 * tv.tv_sec + tv.tv_usec;
		uint32_t loop_start = 0;
		uint32_t read_all_time = 0;
		sensor_read_time = 0;
		filter_time = 0;
#endif

		//for (int tt = 0; tt < 1000; tt++) {

#ifdef TIME_MEASURE
			gettimeofday(&tv, NULL);
			loop_start = 1000000 * tv.tv_sec + tv.tv_usec;
#endif
			//Read current position
			MPU9250_Read_All();

#ifdef TIME_MEASURE
			gettimeofday(&tv, NULL);
			read_all_time += (1000000 * tv.tv_sec + tv.tv_usec) - loop_start;
#endif

			//Calculate the error
			float eRoll = targetRoll - roll;
			float eYaw = targetYaw - yaw;
			float ePitch = targetPitch - pitch;

			//Calculate proportional component
			float pRoll = PID_P * eRoll;
			float pYaw = PID_P * eYaw;
			float pPitch = PID_P * ePitch;

			//Calculate integral component
			iRoll = iRoll + (PID_I * eRoll);
			iYaw = iYaw + (PID_I * eYaw);
			iPitch = iPitch + (PID_I * ePitch);

			//Calculate elapsed time
			PID_LastTime = PID_Now;
			PID_Now = getMicros();
			PID_IterationTime = PID_Now - PID_LastTime;

			//Calculate derivative component
			float dRoll = PID_D * ((eRoll - last_eRoll) / PID_IterationTime);
			float dYaw = PID_D * ((eYaw - last_eYaw) / PID_IterationTime);
			float dPitch = PID_D * ((ePitch - last_ePitch) / PID_IterationTime);

			//Calculate contributions
			float cRoll = pRoll + iRoll + dRoll;
			float cYaw = pYaw + iYaw + dYaw;
			float cPitch = pPitch + iPitch + dPitch;

			//printf("cRoll = %.4f\ncYaw = %.4f\ncPitch = %.4f\n", cRoll, cYaw,cPitch);

			//Calculate the control variable
			duty[0] = BASE_THROTTLE + cRoll + cPitch;
			duty[1] = BASE_THROTTLE - cRoll + cPitch;
			duty[2] = BASE_THROTTLE + cRoll - cPitch;
			duty[3] = BASE_THROTTLE - cRoll - cPitch;

			//Limit the control variable within MIN_THROTTLE and LEDC_HS_MAX_DUTY
			for (uint8_t i = 0; i < 4; i++) {
				if (duty[i] < MIN_THROTTLE) {
					duty[i] = MIN_THROTTLE;
				} else if (duty[i] > LEDC_HS_MAX_DUTY) {
					duty[i] = LEDC_HS_MAX_DUTY;
				}
				//printf("duty[%d] = %d\n", i, duty[i]);
			}

			//Update the PWM duty cycle
			PWM_Set_Duty(&duty[0]);
			//printf("%d\t%d\t%d\t%d\n",duty[0],duty[1],duty[2],duty[3]);

			//Update variables
			last_eRoll = eRoll;
			last_eYaw = eYaw;
			last_ePitch = ePitch;
*/









			vTaskDelayUntil(&xLastWakeTime, 1);	//1ms -> 1000/1 = 1000Hz
		//}

#ifdef TIME_MEASURE
		uint32_t bundle_end;
		gettimeofday(&tv, NULL);
		bundle_end = 1000000 * tv.tv_sec + tv.tv_usec;
		printf("Total time = %.2f ms\n", (bundle_end - bundle_start) / 1000.0);
		printf("Read_All time = %.2f ms\n", read_all_time / 1000.0);
		printf("Read sensor time = %.2f ms\n", sensor_read_time / 1000.0);
		printf("Filter time = %.2f ms\n", filter_time / 1000.0);
		printf("Loop frequency = %.2f Hz\n\n", 1000.0 / ((bundle_end - bundle_start) / 1000000.0));
#endif
	}

	//xTaskCreate(MPU9250_Read_All, "MPU9250_Read_All", 1024 * 2, NULL, 5, NULL);

}
