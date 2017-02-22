#include "stm32f4xx.h"

extern float gimbal_yaw_target;

extern float lpf_current;

typedef struct{
	float Kp, Ki, Kd;
	float MAX_Integral;
	float MAX_Output;
	float temp_integral;
	float temp_derivative;
	float pre_error;
	int16_t output;
}PID_Controller;

void gimbal_setyaw( float setpoint);
void gimbal_pid(PID_Controller *pid, float target,float current);
void low_pass_filter(float current);