#include "gimbal.h"
#include "stm32f4xx.h"
#include "Dbus.h"
float lpf_a = 0.9;

float gimbal_yaw_target;


void gimbal_setyaw( float setpoint)
{
	
	gimbal_yaw_target=setpoint/40;
	
}
void gimbal_pid(PID_Controller *pid, float target,float current)
{
	float error = target-current;
	
	float Kout = error * pid->Kp;
	
	pid->temp_integral += error;
	
	if(pid->MAX_Integral!=0)
	{
		if(pid->temp_integral > pid->MAX_Integral)
		{
			pid->temp_integral = pid->MAX_Integral;
		}
		if(pid->temp_integral < -pid->MAX_Integral)
		{
			pid->temp_integral = -pid->MAX_Integral;
		}
	}
	float Iout = pid->temp_integral * pid->Ki;
	
	pid->temp_derivative = error - pid->pre_error;
	
	float Dout = pid->temp_derivative * pid->Kd;
	
	pid->output= Iout + Dout + Kout;
	
	pid->pre_error=error;
	
	
	if(pid->output > pid->MAX_Output)
	{
		pid->output = pid->MAX_Output;
	}
	if(pid->output < -pid->MAX_Output)
	{
		pid->output = -pid->MAX_Output;
	}
}

float lpf_current=0;

void low_pass_filter(float current)
{
	
  lpf_current = current * lpf_a +  lpf_current * (1 - lpf_a);
}

