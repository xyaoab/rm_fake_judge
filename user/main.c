#include "main.h"
#include "function_list.h"

static u32 ticks_msimg = (u32)-1;
//////
CanRxMsg * msg;
s32 target_angle=0;
int32_t max=0;
float w=60;
ControlMotor motor[4];
///////
void init(){
	SysTick_Init();  
	Dbus_init();//usart1
	//buzzer_init();	 //initialization of buzzer
	tft_init(2,BLACK,GREEN,GREEN);
	LED_master_init();
	gyro_init();
	ADC1_init();
	judging_system_init(); //usart3
	gyro_init();
	pneumatic_init();
	CAN1_Configuration();
	CAN2_Configuration();
	gyro_init();
	gyro_cal();
	Quad_Encoder_Configuration();
	Encoder_Start1();
	Encoder_Start2();
	Friction_wheel_init();
	TIM5_Int_Init(24,13124);// 256hz //3.9xx ms for gyro usage
	DataMonitor_Init();
}

int main(void)
{	
//	u8 dum[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '\n'};
	init();
	//s32 target_gimbal_yaw=GMYawEncoder.ecd_angle;
	
	static PID_Controller motor_speed={85,7,90,32000/15,32000,0,0,0,0};
	static PID_Controller gimbal_speed={85,7,90,9000/15,9000,0,0,0,0};
	static PID_Controller gimbal_position={0.27,0.0030,0,1000,100,0,0,0,0};//0/752

	s32 init_gimbal_pos=GMYawEncoder.ecd_angle;
	s32 target_gimbal_pos= init_gimbal_pos;	
	s32 gimbal_pos_max=init_gimbal_pos+1215;
	s32 gimbal_pos_min=init_gimbal_pos-1215;
	while (1)  {	

		if (ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			
			switch(DBUS_ReceiveData.rc.switch_right){ 
				case 2:	  // gimbal position control 
					rc_target_motor();
					if(DBUS_ReceiveData.rc.switch_left==2)
					{ 
						if(target_gimbal_pos > gimbal_pos_max)
						{
							target_gimbal_pos = gimbal_pos_max;
						}
						else if (target_gimbal_pos < gimbal_pos_min)
						{
							target_gimbal_pos = gimbal_pos_min;
						}
						gimbal_pid(&gimbal_position,target_gimbal_pos,GMYawEncoder.ecd_angle );
						gimbal_pid(&gimbal_speed,gimbal_position.output,GMYawEncoder.filter_rate);
						Set_CM_Speed(CAN1,gimbal_speed.output,0,0,0);
						
					}
					else if(DBUS_ReceiveData.rc.switch_left==3 || DBUS_ReceiveData.rc.switch_left==1)
					{
							Set_CM_Speed(CAN1,0,0,0,0);
							for(int i=0;i<4;i++)
								{
									CM_target[i]=0;
								}	
								target_angle=output_angle;
							
					}
					if(ticks_msimg%3==0)
					{	if(DBUS_ReceiveData.mouse.press_left==1)
						{
							low_pass_filter((float)DBUS_ReceiveData.mouse.x/3);
							target_gimbal_pos -= lpf_current;
							
						}
						else
						{
							target_gimbal_pos-=(DBUS_ReceiveData.rc.ch2/60);
						}
				}
					break;
			
			//buzzer_check();			
				case 1: //mode for chassis 
						yaw_axis_pid_cal(target_angle,output_angle);
						
						rc_target_motor();
						if(DBUS_ReceiveData.rc.switch_left==2)
						{ 
						
							Set_CM_Speed(CAN2,base_pid_cal(CM_target[0]*(1),CM1Encoder.filter_rate,&motor[0]),base_pid_cal(CM_target[1]*(1),CM2Encoder.filter_rate,&motor[1]),base_pid_cal(CM_target[2]*(1),CM3Encoder.filter_rate,&motor[2]),base_pid_cal(CM_target[3]*(1),CM4Encoder.filter_rate,&motor[3]));
						}
						else if(DBUS_ReceiveData.rc.switch_left==1||DBUS_ReceiveData.rc.switch_left==3)
						{ 
								Set_CM_Speed(CAN2,0,0,0,0);
								for(int i=0;i<4;i++)
									{
										CM_target[i]=0;
									}	
									target_angle=output_angle;
								
						}
						
						
						
						if(ticks_msimg%3==0)
						{
						if (DBUS_ReceiveData.rc.ch2>2||DBUS_ReceiveData.rc.ch2<-2)
						{
							target_angle=(target_angle + (DBUS_ReceiveData.rc.ch2*0.01 +(DBUS_CheckPush(KEY_W)-DBUS_CheckPush(KEY_Q))));
						} 
						else
						{
							target_angle=(target_angle +(DBUS_CheckPush(KEY_W)-DBUS_CheckPush(KEY_Q)));
						}
						}
						
						if(ticks_msimg%20==0)
						{	
							w +=(-(InfantryJudge.RealVoltage*InfantryJudge.RealCurrent - 80)*0.02);  //buffer=60J
							if(w>=60)
							{
								w=60;
								mul=1;
							}
							else if( w>0 && w<60)
							{
								power_pid(w,60); 
								mul=1-power_pid_output; //range(0,60*0.007)				
								if(mul>1){mul=1;}
								else if(mul<0.4){mul=0.4;}//protection
							} 
							else//w<=0
							{
								w=0;
								mul=0.4;
							}
						}
					break;
				default:break;
				}	
						
			if(ticks_msimg%50==0)
			{
				tft_clear();
				
				
				switch(DBUS_ReceiveData.rc.switch_right){
					case 1:
						tft_prints(1,2," %d: %d",CM1Encoder.filter_rate,CM2Encoder.filter_rate);
						tft_prints(1,3,"%d: %d",CM4Encoder.filter_rate,CM3Encoder.filter_rate);
						tft_prints(1,4,"p%g i%g d%g",base_Kp,base_Ki,base_Kd); 
						tft_prints(1,5,"yaw_pid:%f",yaw_pid_output_angle);
						tft_prints(1,6,"gyro%d",output_angle);

						tft_prints(1,7,"target%d %d",CM_target[0],CM_target[1]);
						tft_prints(1,8,"      %d %d",CM_target[3],CM_target[2]);
						//tft_prints(1,10,"t_angle%d",target_angle);
						tft_prints(1,9,"HP:%d ",InfantryJudge.LastBlood);//power_pid_output);
						tft_prints(1,10,"W:%f ",w);
						tft_prints(1,11,"mul:%f ",mul);
					break;
				
					case 2:
						tft_prints(0,2,"s:%d",DBUS_ReceiveData.rc.switch_left);
						tft_prints(0,3,"angle:%f",GMYawEncoder.ecd_angle);
								
						tft_prints(0,4,"setpos%d: %d",init_gimbal_pos,target_gimbal_pos);
						//tft_prints(0,5,"speed%f: %f: %f",motor_speed.Kp,motor_speed.Ki,motor_speed.Kd);
					tft_prints(0,5,"Vx:%d",DBUS_ReceiveData.mouse.x);
						tft_prints(0,6,"Px:%d",DBUS_ReceiveData.mouse.x_position);
						tft_prints(0,7,"o:%d",gimbal_position.output);
						tft_prints(0,8,"T:%d",target_gimbal_pos);
					
					break;
					default: break;
			}
				tft_update();
				LED_blink(LED1);
			}			
			
			
		}
	}	
}	

	



