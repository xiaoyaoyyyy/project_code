/***
?*
?*???█████?█????██??▄████???██?▄█?????????██████╗?██╗???██╗?██████╗
?*?▓██?????██??▓██??██????█???██▄█?????????██╔══██╗██║???██║██╔════╝
?*???████▓██????██??█????????▓██▄???????????██████╔╝██║???██║██║??███╗
?*???█????▓▓█???██?▓▓█▄???█?▓██?█▄?????????██╔══██╗██║???██║██║???██║
?*???█?????????████▓???▓███????██??█▄???????██████╔╚██████╔╝██████╔╝
?*??????????x????????????????????▓????????╚═════╝??╚═════╝??╚═════╝
?*?????????????????????????????????
?*????????????????????????????????
?*??????????????????????????????
?*/ 

#include "superstratum.h"
#include "chassis.h"
#include "auto.h"
int tesssst1,tesssst2=0;
int  Str_angle_max=21000;
int  Pit_angle_min=0;
int  Lif_angle_max = 18000;
int  Str_angle_min =  0;
int  Pit_angle_max  = 0;

int16_t  flag=0;
int  Lif_angle_min=00;
int16_t M_flag,M_flag2=0;
int16_t  M_Lif_L_Init  =0;
int16_t  M_Lif_R_Init   =0;

PID_Typedef Uplift_Motor3_Pid,\
						Uplift_Motor3_Angle_Pid,	\
					
						Uplift_Motor4_Pid,\
						Uplift_Motor4_Angle_Pid, \
						
						Stretch_Motor5_Pid,\
						Stretch_Motor5_Angle_Pid,\
						
						Stretch_Motor6_Pid,\
						Stretch_Motor6_Angle_Pid,\
											
						Pitch_Motor7_Pid,\
						Pitch_Motor7_Angle_Pid;\

PID_ring_take_TypeDef fa_output,fa_target;
ANGLE_TypeDef angle;

int get_acoder_flag=0;
bool change_mode_flag=1;
//▓▓▓▓▓▓▓▓▓▓▓▓▓▓主程序▓▓▓▓▓▓▓▓▓▓▓▓▓▓
void control_sup_task()
{	
	  if(pinstate[0]==true)		
		  {
					Lif_R_Init=motor_supestratum[2].total_angle;
					Lif_R_Init=motor_supestratum[3].total_angle;
					Lif_angle_min=0;
				
				}else  Lif_angle_min=-1000;
			
			if(pinstate[1]==true)		
		  {
					Str_L_Init=motor_supestratum[4].total_angle;
					Str_R_Init=motor_supestratum[5].total_angle;
					Str_angle_min=0;
					Sinit_flag2=false;
				}else  Str_angle_min=-200;
			
				if(pinstate[2]==true)		
	 	  {
					Pit_Init = motor_supestratum[6].total_angle;							
					Pit_angle_min=0;
					Pinit_flag2=false;
				}	else		Pit_angle_min=-1000;
  
	
	 //初始化
 if( control_current_State == Manual) remote_control();
		/*****该函数通过条件： 1.遥控锁定 2.遥控信号丢失 3.遥控未连接接收机**/
	if(sbus_struct.sbus_channels[3]<240 || sbus_flag==1  || sbus_struct.frame_lost!=0||control_current_State==N_power)
		{//遥控未开启处理
			CAN_cmd_gimbal1(0,0,0,0) ;
			CAN_cmd_gimbal2(0,0,0,0) ; 			
		}
		else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	if(control_current_State!=Manual)	change_mode_flag=1;
}
	int  L_L,L_R,S_L,S_R,P,L_L2,L_R2,S_L2,S_R2,P2=0;//自动切会手动的时候保持不动
void remote_control()
{
//	int min;
	if(	change_mode_flag==1)
	 {    
		 L_L=motor_supestratum[2].total_angle-Lif_L_Init;
		 L_R=motor_supestratum[3].total_angle-Lif_R_Init;
		 S_L=motor_supestratum[4].total_angle-Str_L_Init;
		 S_R=motor_supestratum[5].total_angle-Str_R_Init;
	//	 if(Pit_Init<0)
		 P  =motor_supestratum[6].total_angle-Pit_Init;	 
  //  else 		  P  =motor_supestratum[6].total_angle+Pit_Init;	 
	 }

		 //有力
	if(control_current==	M_power)
	{
		 if(M_flag2==1)//只有在无力后切换会有力，才能复位
		 {
	 	    M_flag=0;
		  }
	  	 M_flag2=0;
	}
	//上层模式
  else if(control_current==M_sup)
	{	
	   	tesssst1++;
		//遥控器只控制上层
			if(sbus_struct.sbus_channels[1]!=1024)//抬升
				angle.current[0]=Steer(sbus_struct.sbus_channels[1]-1024,50);
	  	else	angle.current[0]=0;// 1807为max速
	
			if(sbus_struct.sbus_channels[2]!=1024)//伸缩
				angle.current[1]=Steer(sbus_struct.sbus_channels[2]-1024,50);
	  	else	 angle.current[1]=0;// 1807为max速
	
			if(sbus_struct.sbus_channels[3]!=1024)//P
				angle.current[2]=Steer(sbus_struct.sbus_channels[3]-1024,200); 
	  	else		angle.current[2]=0;// 1807为max速
		
		 // diancifa(1);

			if(M_flag==0)
				{
					M_Lif_L_Init=motor_supestratum[2].total_angle;
					M_Lif_R_Init=motor_supestratum[3].total_angle;
				}
				M_flag=1;
						
	//根据键位确保给定值
				angle.tar_angle[0] += calculateJoystickIncrement(angle.current[0],0);
				angle.tar_angle[1] -= calculateJoystickIncrement(angle.current[0],0);	
				angle.tar_angle[2] -= calculateJoystickIncrement(angle.current[1],0);
				angle.tar_angle[3] += calculateJoystickIncrement(angle.current[1],0);
				angle.tar_angle[4] += calculateJoystickIncrement(angle.current[2],0);	 
				
			if(	change_mode_flag==1)
			{ 
				if(L_L2!=L_L)angle.tar_angle[0]=0;
				if(L_R2!=L_R)angle.tar_angle[1]=0;
					if(S_L2!=S_L)angle.tar_angle[2]=0;
					if(S_R2!=S_R)angle.tar_angle[3]=0;
					if(P2!=P)angle.tar_angle[4]=0;
						
					angle.tar_angle[0]=angle.tar_angle[0]+L_L;
					angle.tar_angle[1]=angle.tar_angle[1]+L_R;
					angle.tar_angle[2]=angle.tar_angle[2]+S_L;
					angle.tar_angle[3]=angle.tar_angle[3]+S_R;
					angle.tar_angle[4]=angle.tar_angle[4]+P;	
				 
		 }	
		 L_L2=L_L;
		 L_R2=L_R;
		 S_L2=S_L;
		 S_R2=S_R;
		  P2  = P  ;
		 if(motor_supestratum[5].total_angle<Str_R_Init+5000) 		Pit_angle_max=45000;
		 else Pit_angle_max  = 170000;

	 angle.tar_angle[0]=angle_limit(angle.tar_angle[0],  Lif_angle_min , Lif_angle_max );
	 angle.tar_angle[1]=angle_limit(angle.tar_angle[1], -Lif_angle_max ,-Lif_angle_min );
	 angle.tar_angle[2]=angle_limit(angle.tar_angle[2], -Str_angle_max ,-Str_angle_min );
	 angle.tar_angle[3]=angle_limit(angle.tar_angle[3],  Str_angle_min , Str_angle_max );
	 angle.tar_angle[4]=angle_limit(angle.tar_angle[4],  Pit_angle_min , Pit_angle_max );

    change_mode_flag=0; 
		motion(M_Lif_L_Init+  + angle.tar_angle[0],&Uplift_Motor3_Angle_Pid ,&Uplift_Motor3_Pid ,2);
		motion(M_Lif_R_Init   + angle.tar_angle[1],&Uplift_Motor4_Angle_Pid ,&Uplift_Motor4_Pid ,3);
		motion(Str_L_Init	    + angle.tar_angle[2],&Stretch_Motor5_Angle_Pid,&Stretch_Motor5_Pid,4);
		motion(Str_R_Init 	  + angle.tar_angle[3],&Stretch_Motor6_Angle_Pid,&Stretch_Motor6_Pid,5);
		 motion(Pit_Init       + angle.tar_angle[4],&Pitch_Motor7_Angle_Pid  ,&Pitch_Motor7_Pid  ,6);

  }																											
		

	#ifdef DEBUG_GIMBAL
	CAN_cmd_gimbal1(0,0,Uplift_Motor3_Pid._out,Uplift_Motor4_Pid._out);	
	CAN_cmd_gimbal2(Stretch_Motor5_Pid._out,Stretch_Motor6_Pid._out,Pitch_Motor7_Pid._out,0);	
	#endif

}
//▓▓▓▓▓▓▓▓▓▓▓▓▓▓初始化/功能函数▓▓▓▓▓▓▓▓▓▓▓▓▓▓
void Hair_ring_Init(void)
{
	
	PID_Init(&Uplift_Motor3_Pid);// id 5
	PID_Init(&Uplift_Motor3_Angle_Pid);
	
	PID_Init(&Uplift_Motor4_Pid);// id 6
	PID_Init(&Uplift_Motor4_Angle_Pid);

	PID_Init(&Stretch_Motor5_Pid);
	PID_Init(&Stretch_Motor5_Angle_Pid);
	
	PID_Init(&Stretch_Motor6_Pid);
	PID_Init(&Stretch_Motor6_Angle_Pid);
	
	PID_Init(&Pitch_Motor7_Pid);
	PID_Init(&Pitch_Motor7_Angle_Pid);
}

/**
 * @brief	PID设置 除底盘外所有的PID设置
 * @param	none
 * @ratval  none
 * @attention none
*/
void ALL_PID_SET(void)
{
	int target_value = 0;//目标值
	int target_angle = 0;//目标角度
	
//	fa_target._Friction_wheel1 = target_value;
//	fa_target._Friction_wheel2 = target_value;
	
	fa_target._Uplift_L = target_value;
	fa_target._Uplift_Angle_L =  target_angle;
	
	fa_target._Uplift_R = target_value;
	fa_target._Uplift_Angle_R =  target_angle;
		
	fa_target._Stretch = target_value;
	fa_target._Stretch_Angle = target_angle;
	
	fa_target._Stretch = target_value;
	fa_target._Stretch_Angle = target_angle;
	
	fa_target._Pitch = target_value;
	fa_target._Pitch_Angle = target_angle;

	
	PID_SET(&Uplift_Motor3_Pid,fa_target._Uplift_L,10, 0.01, 0);
	PID_SET(&Uplift_Motor3_Angle_Pid,fa_target._Uplift_Angle_L,11, 0.01, 0);
	
	PID_SET(&Uplift_Motor4_Pid,fa_target._Uplift_R,10, 0.01, 0);/*PID要调整*/
	PID_SET(&Uplift_Motor4_Angle_Pid,fa_target._Uplift_Angle_R,11, 0.01, 0);/*PID要调整*/
		
	PID_SET(&Stretch_Motor5_Pid,fa_target._Stretch,6, 0, 0.0);
	PID_SET(&Stretch_Motor5_Angle_Pid,fa_target._Stretch_Angle,6, 0.0, 0);
	
	PID_SET(&Stretch_Motor6_Pid,fa_target._Stretch,6, 0, 0.0);
	PID_SET(&Stretch_Motor6_Angle_Pid,fa_target._Stretch_Angle,6, 0.0, 0);
	
	
	PID_SET(&Pitch_Motor7_Pid,fa_target._Pitch,8, 0.01, 0);
	PID_SET(&Pitch_Motor7_Angle_Pid,fa_target._Pitch_Angle,10,0.01,0);
}



void motion(int Target_angle,PID_Typedef*Angle_pid,PID_Typedef*Speed_pid,int motor_)
{
	int Actual_angle;
	int up_angele_cha;

 Actual_angle = (int)((float)motor_supestratum[motor_].total_angle);
	
	Angle_pid->_out = PID_position(Angle_pid,Target_angle,Actual_angle,1000,14000);

	up_angele_cha=Angle_pid->_out;
	
	Speed_pid->_out= PID_position(Speed_pid,up_angele_cha,(int)((float)motor_supestratum[motor_].speed_rpm),1000,10000);

}




float angle_limit(float angle_,float angle_min,float angle_max)
{
	
	if(angle_<angle_min)angle_=angle_min;
	else if(angle_>angle_max)angle_=angle_max;

	return angle_;
}


float calculateJoystickIncrement( float currentInput, float previousInput)
{
		float chazhi;
			if(currentInput>0)
	{
	  chazhi= currentInput - previousInput;
		chazhi = (chazhi < 0) ? 0 : chazhi;		
	}
	else 
	{
			   chazhi= currentInput - previousInput;
				chazhi = (chazhi > 0) ? 0 : chazhi;
		}
			return chazhi;
}


	int D_flag,D_flag1,D_flag2=0;
	
void diancifa(bool a)
{
	D_flag = sbus_struct.sbus_channels[9];	
	if(D_flag < 1022 && D_flag1>= 1022)
	{
		D_flag2++;
	}
	D_flag1= sbus_struct.sbus_channels[9];
	

//	if (D_flag2% 2 == 0)
	if(a==false)
	{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET);
 }
else	if(a==true)
	//	else if(D_flag2% 2 != 0)
		{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_SET);
			
		}
}


