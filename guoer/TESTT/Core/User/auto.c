/***
?*?_ooOoo_
?*?o8888888o
?*?88"?.?"88
?*?(|?-_-?|)
?*??O\?=?/O
?*?___/`---'\____
?*?.???'?\\|?|//?`.
?*?/?\\|||?:?|||//?\
?*?/?_|||||?-:-?|||||-?\     
?*?|?|?\\\?-?///?|?|
?*?|?\_|?''\---/''?|?|
?*?\?.-\__?`-`?___/-.?/
?*?___`.?.'?/--.--\?`.?.?__
?*?.""?'<?`.___\_<|>_/___.'?>'"".
?*?|?|?:?`-?\`.;`\?_?/`;.`/?-?`?:?|?|
?*?\?\?`-.?\_?__\?/__?_/?.-`?/?/
?*?======`-.____`-.___\_____/___.-`____.-'======
?*?`=---='
?*??????????.............................................
?*???????????佛曰：bug泛滥，我已瘫痪！
?*/

#include "auto.h"
#include "tim.h"
#include "sbus.h"
#include "HWT101.h"
#include "add.h"
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
extern SensorData receivedDataa[2];
extern PID_ring_take_TypeDef fa_output,fa_target;
extern PID_chassis_TypeDef target,output;
extern ANGLE_TypeDef angle;
bool ball_canget,ball_got,ball_put = false;
extern TIM_HandleTypeDef htim4;
bool turn_flag1=true;
bool turn_flag2=false;
bool ball_g=false;
int last_flag_9 ,flag_9 ,flag3_9,last_flag_10,flag_10,flag3_10 ;
int  Lif_L_Init, Lif_R_Init , Str_L_Init,Str_R_Init , Pit_Init    =0;

uint8_t get_Destination =1;
uint8_t put_Destination =2;
uint8_t get_task=1;
uint8_t put_task=2;	
uint8_t none_task=3;
uint8_t canot_get_ball_cnt=0;
uint8_t message[2][8]={{0,0X01,0X02,0X03,0X04,0X05,0X06,0X07},{0x03,0x04,0X02,0X01}};
int tesst1,tesst2,tesst3,tesst4=0;
int usa_flag;
int have_arrivied =4;
int action_flag=0;
uint8_t super_control_Stack=Init;
uint8_t super_control_State=INIT;
double V,distnce,motion_flag;
bool ballcanget_flag,can_s_flag,canot_flag=false;
float distance;
int Stre_ball_go;
//float distance;
bool change_flag;
double z=0;
bool Sinit_flag=true,Sinit_flag1=true,Pinit_flag=true,Pinit_flag1=true;
bool Pinit_flag2,Sinit_flag2=0;
int ab,abc,abcd=0;
bool turn_flag,turn_to90_flag,turn_to0_flag=true;
bool turn_90_flag,turn_70_flag,turn_0_flag,turn_110_flag,first=false;
int XXXX=0,YYYY=0;
int FANGQIU_MUBIAODIAN,CNTT,to2_1=0;
int jiazhuacnt=0,distance_s,qianjinfangqiu;
bool qianjinburangRADAR=false;
//▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓主程序▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓
void Auto_control_task()
{		 
	CAN_cmd_gimbal1(0,10000,1000,1000);	
//	CAN_cmd_gimbal2(Stretch_Motor5_Pid._out,Stretch_Motor6_Pid._out,Pitch_Motor7_Pid._out,0);
		
/// CAN_cmd_AGV_chassis(1000,1000,1000,1000);
  CAN_cmd_chassis(1000,1000,1000,1000);

	//	usart_send_char(&huart6,message[VISION][PUT_TASK]);
		 		//	usart_send_char(&huart2,message[VISION][GET_TASK]);
		///	usart_send_message(&huart2,RADAR,6);

 if(sbus_struct.sbus_channels[8]==240)//红方
	 {  
		 if(YYYY<1)YYYY++;
		if(YYYY==1)
		{	 	//	usart_send_char(&huart6,message[VISION][0]);
			YYYY=2;
		 }

	if(All_Data.Ang_Z<93.3&&All_Data.Ang_Z>87)
	{ turn_90_flag=true;
	}
	else turn_90_flag=false;
	
	
	
}
 else//蓝方
 {
		if(YYYY<1)YYYY++;
		if(YYYY==1)
		{	 	//	usart_send_char(&huart6,message[VISION][1]);
			YYYY=2;
		 }
	 if(All_Data.Ang_Z>-93.3&&All_Data.Ang_Z<-87)
	{ turn_90_flag=true;
	}
	else turn_90_flag=false;

 }
 
 
 if(XXXX==50)
	{SYSTEM_Init_only(&huart3,CALIYAW);
		XXXX++;
	}
	else if(XXXX<50)	XXXX++;
	
	if(pinstate[1]==true)		
		  { 
        ballcanget_flag=true;
			  can_s_flag=true;
			}

//	 if(sbus_struct.sbus_channels[8]==240)ball_canget=false;
//	 else ball_canget =true;
	
	GET_InitPinstate();
	checkRxBufferTimeout();
	if(sbus_struct.sbus_channels[3]>240 && sbus_flag!=1  && sbus_struct.frame_lost==0 && control_current_State==Auto)
		{//遥控开启处理
				auto_control_state();				
		}
		else
		{
// 	 	CAN_cmd_gimbal1(0,0,0,0);
//     CAN_cmd_gimbal1(0,0,0,0);
		}
		//切换了模式了就可以初始化了
		if(control_current!=A_init)
		{
			Pinit_flag2=0;
			Sinit_flag2=0;
			 Pinit_flag1=true;
			 Sinit_flag1=true;
		}
}

int cuttt=0;
void auto_control_state()
{

	if(control_current==A_init)
	{
     value_init();
	}
	else if (control_current==A_reset)
	{
		
		super_control_Stack=None_;
		ball_canget=false;
		ball_got=false;

	}
	else if (control_current==A_power)
	{
 //   usa_flag=VISION;
		
	 	
		if(have_arrivied==1)//拿球点
		{
//	usart_send_message(&huart6,VISION,GET_TASK);
			qianjinburangRADAR=true;
				usa_flag=VISION;							
		}
//    else if(have_arrivied==2)//放球找点
//		{
//			//告诉视觉我要去放球
//      //	usart_send_message(&huart6,VISION,PUT_TASK);
//		//		usart_send_char(&huart6,message[VISION][PUT_TASK]);
//		//	if(rx_V2!=0x00)
//			//{
//			if(CNTT==0)	message[RADAR][BASKET_DESTINATION]= 0X05;
//			if(CNTT==1)	message[RADAR][BASKET_DESTINATION]= 0X05;
//			if(CNTT==2)	message[RADAR][BASKET_DESTINATION]= 0X05;
//			if(CNTT==3)	message[RADAR][BASKET_DESTINATION]= 0X06;
//			if(CNTT==4)	message[RADAR][BASKET_DESTINATION]= 0X06;
//			if(CNTT==5)	message[RADAR][BASKET_DESTINATION]= 0X06;
//			if(CNTT==6)	message[RADAR][BASKET_DESTINATION]= 0X07;
//			if(CNTT==7)	message[RADAR][BASKET_DESTINATION]= 0X07;
//			if(CNTT==8)	message[RADAR][BASKET_DESTINATION]= 0X07;
//			//}
//		//	else message[RADAR][BASKET_DESTINATION]=message[RADAR][BASKET_DESTINATION];
//   
//				usart_send_message(&huart2,RADAR,BASKET_DESTINATION);
//		  abcd++;
//			if(abcd>50)
//			 {
//				 if(V==0) 
//         { 
//					 usa_flag=qianjin;
//					 qianjinburangRADAR=false;
//					 qianjinfangqiu++;
//					 if(qianjinfangqiu>100)
//					 {	 have_arrivied=3;           
//					 }
//					 CNTT++;
//           abcd=0;	
//         }
//       }
//			 if(qianjinburangRADAR)
//			 {
//			 	usa_flag=RADAR;//但是是雷达控，因为视觉将位置信息发给雷达
//        turn_flag2=true;
//			  to2_1=0;
//			 }
//		}
		else if(have_arrivied==3)//如果到放球点了（雷达会控到放球点，但是需要告知放完求了）
		{
			qianjinburangRADAR=true;
			  qianjinfangqiu=0;    
			 super_control_Stack=Put;				
		}
		else if(have_arrivied==4)
		{
			if(ball_g)//加到球先自转再给雷达控制		
      {       
      	if(turn_90_flag==false)	usa_flag=ZIZHUAN;	
		 	 else{
         abc++;
				 if(qianjinburangRADAR) 
				 {		usa_flag=RADAR;
              turn_flag2=true;   
					    to2_1=0;
			     }
				 
				 //usart_send_message(&huart2,RADAR,PUT_DESTINATION);			
	    if(CNTT==0)	message[RADAR][BASKET_DESTINATION]= 0X06;
			if(CNTT==1)	message[RADAR][BASKET_DESTINATION]= 0X05;
			if(CNTT==2)	message[RADAR][BASKET_DESTINATION]= 0X07;
			if(CNTT==3)	message[RADAR][BASKET_DESTINATION]= 0X06;
			if(CNTT==4)	message[RADAR][BASKET_DESTINATION]= 0X06;
			if(CNTT==5)	message[RADAR][BASKET_DESTINATION]= 0X06;
			if(CNTT==6)	message[RADAR][BASKET_DESTINATION]= 0X07;
			if(CNTT==7)	message[RADAR][BASKET_DESTINATION]= 0X07;
			if(CNTT==8)	message[RADAR][BASKET_DESTINATION]= 0X07;
			//}
		//	else message[RADAR][BASKET_DESTINATION]=message[RADAR][BASKET_DESTINATION];
         
				usart_send_message(&huart2,RADAR,BASKET_DESTINATION);				 
        if(abc>80)
			 {			 
         if(V==0){ 
					 usa_flag=qianjin;
					 qianjinburangRADAR=false;
					 qianjinfangqiu++;
					 if(qianjinfangqiu>60)
					 {	 have_arrivied=3;           
					 }
					 CNTT++;
					// have_arrivied=3;		 	
           abc=0;		
         }
        }
			 }
      }
		else	
			{
				 usa_flag=RADAR;
				if(first)		
				{	 if(to2_1<1)to2_1++;
		      if(to2_1==1)
		       {	 	
		        usart_send_message(&huart2,RADAR,5);	
		      	to2_1=2;
		       }					
		      	usart_send_message(&huart2,RADAR,GET_DESTINATION);	
				}	
					ab++;
			  if(ab>60)
					{if(V==0) 
						{ 
							first=true;
							if(turn_90_flag==false)		 	usa_flag=ZIZHUAN;	
			       	else
							{
							 have_arrivied=1;
							 ab=0;		
					   }		
					  }				
				  }
       }    
     }
 
//usa_flag=RADAR;	
	//上层模式选择！！！！！！！！！！！！！！！！！！	
			switch(super_control_Stack)
		{	
			Auto_init();
			//@@@@@@@@@@@@@@@@@
			case Init:
			fa_output._Uplift_L=0;
			fa_output._Uplift_R=0;
			fa_output._Stretch=0;
			fa_output._Pitch=0;
			
			break;
			//@@@@@@@@@@@@@@@@@@@@@@@@@@@
			case None_://初始模式，会自动记录下并回到初始位置。
				//cnt=0;
			jiazhuacnt=0;
			tesst4++;
				Auto_init();	
			diancifa(false);			
        canot_flag=true;
				if(ball_canget)		super_control_Stack=Getting_motion2;//如果球在夹球范围内，就可以让上层夹球了
	    	if(canot_get_ball_cnt==3)  have_arrivied=1;
			break;
		
			//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
			case Getting_motion2:
							
			
     //  if(distance==0) distance=20;			
				//	if(V==0)	
				//	{
             //if(can_s_flag)
						 Stre_ball_go= map_value(230, 130, 230, 20000, 21400);		
						
					//	can_s_flag=false;
					//	ball_got=true;	
				//	}	
					
				ballcanget_flag=false;
#ifdef MONTION_TIME
	      HAL_TIM_Base_Start_IT(&htim4);	
#endif
				if(pinstate[0]==true)		change_flag=true;//只有刚开始允许，为了记录起爪瞬间的目标距离
					
      if(motor_supestratum[4].total_angle<stre_can_pit+Str_L_Init)				
					control_sup(up_ball_getting,-up_ball_getting,-Stre_ball_go,Stre_ball_go,pit_ball_get );							
			else 
					control_sup(up_ball_getting,-up_ball_getting,-Stre_ball_go,Stre_ball_go,pit_ball_init );//上升
				
			if (motor_supestratum[2].total_angle>up_ball_getting-LIF_L_OFFSET)//是否到顶		//&&motor_supestratum[4].total_angle<-stre_ball_go+Str_L_Init+STR_OFFSET
				{
					if(receivedDataa[VISION].B!=0) distance =receivedDataa[VISION].B;					
				
					super_control_Stack=Getting_motion2_B;
				}
			 
				break;	
			case Getting_motion2_B:
						turn_flag=true;
				   control_sup(up_ball_got,-up_ball_got,-Stre_ball_go,Stre_ball_go,pit_ball_get);//前伸，动p,下降高度
					 
					if (motor_supestratum[2].total_angle<=up_ball_got+LIF_L_OFFSET) //&& motor_supestratum[6].total_angle<=(pit_ball_get+Pit_Init+PIT_OFFSET))//&&motor_supestratum[4].total_angle<=stre_ball_go+Str_L_Init+STR_OFFSET
					{			 
				  //	HAL_TIM_Base_Stop_IT(&htim4);	
						cnt=0;
				 //  	ball_canget=false;
          	//微动开关判断是否加到球	
						if(ball_g)	//如果夹到球
							{
//								if(rx_buffer_V[17]==0x01)
//								{
									super_control_Stack=Got;
							  	have_arrivied=4; //告诉雷达我要去目的地	
//						    }	
//								else 
//								{			
//                 if(canot_flag)	canot_get_ball_cnt++;
//								   canot_flag=false;										
//							      super_control_Stack=None_;	
//								  }
//								 
							}
							else 
							{	
									if(canot_flag) canot_get_ball_cnt++;
							   	canot_flag=false;
							  	super_control_Stack=None_;			//如果没夹到，直接初始化		
								
#ifdef MONTION_TIME
	        	  	HAL_TIM_Base_Stop_IT(&htim4);	
#endif
							}
						}
				
					break;
		//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@				 
			case Got:

			if(pinstate[1])
					control_sup(up_ball_getting,-up_ball_getting,-stre_ball_init,stre_ball_init,pit_ball_put+13500); 
       else
					control_sup(up_ball_getting,-up_ball_getting,-Stre_ball_init,Stre_ball_init,Pit_ball_put+13500); 
			

//     		if ( (motor_supestratum[6].total_angle<=(pit_ball_V_max+Pit_Init+PIT_OFFSET)))
//					{		
//             diancifa(true);
//            diancifa(false);   
						//if(rx_V1==0x01) super_control_Stack=throw_ball;
		//			}
				
						
				 break;
		//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@		
			case throw_ball:

      	if(pinstate[1])
					control_sup(up_ball_init,-up_ball_init,-stre_ball_init,stre_ball_init,pit_ball_put); 
       else
					control_sup(up_ball_init,-up_ball_init,-Stre_ball_init,Stre_ball_init,Pit_ball_put); 
			
     		if ( motor_supestratum[6].total_angle<=(pit_ball_put+Pit_Init+PIT_OFFSET))
					{			
						 diancifa(true);
            diancifa(false);						
						
						super_control_Stack=None_;
					}

		 break;
		//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@		
			
			case Put:
				//先上升再伸出

			if(pinstate[3])
					control_sup(up_ball_put,-up_ball_put,-stre_ball_init,stre_ball_init,pit_ball_put); 
       else
					control_sup(up_ball_put,-up_ball_put,-stre_ball_init,stre_ball_init,Pit_ball_put); 
			
					if (motor_supestratum[2].total_angle <= up_ball_put+LIF_L_OFFSET)
					{					
						  	jiazhuacnt++;
           if(jiazhuacnt>400&&jiazhuacnt<600)     
						 diancifa(true);
           else if(jiazhuacnt>800&&jiazhuacnt<1000)			
						{   diancifa(false);			
										super_control_Stack=None_;
									have_arrivied=4;
						}
					}
						
					break;
		}	
		Auto_chassic_mode();
	}
	#ifdef DEBUG_GIMBAL
	CAN_cmd_gimbal1(0,0,Uplift_Motor3_Pid._out,Uplift_Motor4_Pid._out);	
	CAN_cmd_gimbal2(Stretch_Motor5_Pid._out,Stretch_Motor6_Pid._out,Pitch_Motor7_Pid._out,0);	
	#endif

		 
}

float auto_tar_angle,t_angle,t_angle_rad,distance, tar_change_flag=0;
int ANG=0;
void Auto_chassic_mode()
{
			if (usa_flag==RADAR)
		{   HAL_TIM_Base_Start_IT(&htim4);	
		  	cnt=0;
		   	canot_get_ball_cnt=0;
				double	y= receivedDataa[RADAR].A;
				double  x= receivedDataa[RADAR].B;

			  if(control_current_State==Auto)
		  	{
			   if(x!=0||y!=0)  
				 { V=sqrt(x*x+(y*y));
					 if(V>0)V=-V;	
				 }
					 else V=0;
			  }
//				 if(x!=0||y!=0)ANG=atan2(x,y)* 180.0f / 3.1415;		 
//	         else ANG=0;
//		
			if( (motor_chassis[1].total_angle>out_6020_angle[0]-3|| motor_chassis[1].total_angle<out_6020_angle[0]+3)
				 &&(motor_chassis[2].total_angle>out_6020_angle[0]-3|| motor_chassis[2].total_angle<out_6020_angle[0]+3)
				 &&(motor_chassis[3].total_angle>out_6020_angle[0]-3|| motor_chassis[3].total_angle<out_6020_angle[0]+3)
				 &&(motor_chassis[4].total_angle>out_6020_angle[0]-3|| motor_chassis[4].total_angle<out_6020_angle[0]+3))
					 V=V;
			else V=0;
				Target_speed(&target,x,-y,V/8,0);  
		}

 else if(usa_flag==VISION)
		{	
			
			 
			 tar_change_flag  = receivedDataa[VISION].C;	
		if(super_control_Stack!=None_) 		
		{distance=0;
			t_angle=0;
		}
		else
		{
			t_angle	        =	receivedDataa[VISION].A;
			 distance 				= receivedDataa[VISION].B;
		}
			if(cnt>10)
			{
				have_arrivied=4;
			}
			if(canot_get_ball_cnt<3)
			{   if(t_angle>7)
		   	{V=0.16;
           auto_tar_angle=0	;			
		  	}
				else  if(t_angle<-7)
				{auto_tar_angle=0	;V=-0.16;
        }	
		   else
				{ 
					auto_tar_angle=t_angle;
		   if(distance>=550)  			//距离远直接开冲
			  {			 V=10;
	       ball_canget=false;	
					HAL_TIM_Base_Stop_IT(&htim4);						
         cnt=0;
			  }				
        else if(distance==0)
				{		 
         HAL_TIM_Base_Start_IT(&htim4);						
						ball_canget=false;	
			  	if(canot_get_ball_cnt<1)    V=0;				
				  else   //夹不到球就自转
					{	
						auto_tar_angle=0;
//						    	 if((All_Data.Ang_Z>-130&&All_Data.Ang_Z<-50))
//						 {	 V=-0.2;//	识别陀螺仪 在规定角度转
//                 
//						 }            							 
//							else  if((All_Data.Ang_Z>=90&&All_Data.Ang_Z<=130)) V=0.2;
//						
						
					}	
				}
				else if(distance<550)  //跑的时候HZ不够，提前减速				
			 {	HAL_TIM_Base_Stop_IT(&htim4);						
          cnt=0; 	
					 if(distance>230) V=3;//距离小慢慢接近
				  else if(distance<200)V=-2;
					 else V=0;						 
			 }	 
			 
			 //判断是否可以夹球
 			 if(distance<230&&distance>200&&distance!=0)
				{					
					distance_s=distance;
					HAL_TIM_Base_Stop_IT(&htim4);						
          cnt=0;
					if(ballcanget_flag==true)	 
						ball_canget=true;
				  else ball_canget=false;
				}
			}
     Target_speed(&target,0,0,V,auto_tar_angle);   			
		}
		else 
		{have_arrivied=4;
			canot_get_ball_cnt=0;
		}
		
	}
	
		else if(usa_flag==ZIZHUAN)
		{					
		  	 if(sbus_struct.sbus_channels[8]==240)
		  	 {
				
					 if(turn_90_flag==false)
					 {
							if(All_Data.Ang_Z<75)V=-0.60;
							else if(All_Data.Ang_Z>75&&All_Data.Ang_Z<87)V=-0.2;	
							else if(All_Data.Ang_Z>110)V=0.60;
							else if(All_Data.Ang_Z<110&&All_Data.Ang_Z>93)V=0.2;
							else if(All_Data.Ang_Z<93.3&&All_Data.Ang_Z>87)
							 {
								V=0;					
								}
							}
							else V=0;	
						
										
					}
					else
				 {	
					
						 if(turn_90_flag==false)
						 {
								if(All_Data.Ang_Z>-70)V=0.50;
							else if(All_Data.Ang_Z<-70&&All_Data.Ang_Z>-87)V=0.05;	
							else if(All_Data.Ang_Z<-160)V=-0.50;
							else if(All_Data.Ang_Z>-160&&All_Data.Ang_Z<-93)V=-0.05;
							else if(All_Data.Ang_Z>-93.3&&All_Data.Ang_Z<-87)
							 {
								V=0;					
								}
							}
              else V=0;
						
				
				}
	
			if(  (motor_chassis[1].total_angle>out_6020_angle[0]-3|| motor_chassis[1].total_angle<out_6020_angle[0]+3)
				 &&(motor_chassis[2].total_angle>out_6020_angle[0]-3|| motor_chassis[2].total_angle<out_6020_angle[0]+3)
				 &&(motor_chassis[3].total_angle>out_6020_angle[0]-3|| motor_chassis[3].total_angle<out_6020_angle[0]+3)
				 &&(motor_chassis[4].total_angle>out_6020_angle[0]-3|| motor_chassis[4].total_angle<out_6020_angle[0]+3))
					 V=V;
			else V=0;
			
			 Target_speed(&target,0,0,V,0);  
		}
		else if(usa_flag==qianjin)
		{
			
			 Target_speed(&target,0,0,-0.4,0.01);  
		}
		
		
//	}
	Speed_loop();
  CAN_cmd_AGV_chassis(output.ChassisWheel_Aangle_Speed_1,output.ChassisWheel_Aangle_Speed_2,output.ChassisWheel_Aangle_Speed_3,output.ChassisWheel_Aangle_Speed_4);
  CAN_cmd_chassis(output.ChassisWheel_Speed_1,output.ChassisWheel_Speed_2,output.ChassisWheel_Speed_3,output.ChassisWheel_Speed_4);
}


//▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓初始化/功能函数▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓
void control_sup(int up_l_L,int up_R_L,int stre_L ,int stre_R,int pit)
{

  	target_limit[0] = up_l_L;
    target_limit[1] = up_R_L;
    target_limit[2] = stre_L;
		target_limit[3] = stre_R;
    target_limit[4] = pit;
	 
		motion(Lif_L_Init + tar_arr[0],&Uplift_Motor3_Angle_Pid ,&Uplift_Motor3_Pid ,2);
		motion(Lif_R_Init + tar_arr[1],&Uplift_Motor4_Angle_Pid ,&Uplift_Motor4_Pid ,3);
		motion(Str_L_Init + tar_arr[2],&Stretch_Motor5_Angle_Pid,&Stretch_Motor5_Pid,4);
		motion(Str_R_Init + tar_arr[3],&Stretch_Motor6_Angle_Pid,&Stretch_Motor6_Pid,5);
		motion(Pit_Init 	+ tar_arr[4],&Pitch_Motor7_Angle_Pid  ,&Pitch_Motor7_Pid  ,6);
	
}


void Auto_init()
{
		
	if(pinstate[1])	control_sup(up_ball_init,-up_ball_init,-stre_ball_init,stre_ball_init,pit_ball_init);
	else 	control_sup(up_ball_init,-up_ball_init,-Stre_ball_init,Stre_ball_init,pit_ball_init);
}


void Key_judgement(int a)
{
		if(a==9)
	{	
		flag_9 = sbus_struct.sbus_channels[a];
		if (flag_9  < 1022 && last_flag_9  >= 1022)  flag3_9++;
		last_flag_9  = sbus_struct.sbus_channels[a];	
		ball_canget=(flag3_9%2==0)?false:true;
	}
	else if(a==10)
	{
	 	flag_10 = sbus_struct.sbus_channels[a];
		if (flag_10 < 1022 && last_flag_10 >= 1022)  flag3_10++;	
		last_flag_10 = sbus_struct.sbus_channels[a];	
		ball_canget=(flag3_10%2==0)?false:true;
	}
			
}


int map_value(int value, int in_min, int in_max, int out_min, int out_max)
	
{
    // 将输入值从输入范围映射到输出范围
	 if (value == 0)    return 0;
	return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}




float usa_flag1,usa_flag2,usa_flag11,usa_flag22=0;
int usa_count=0;
void usart_send_message(UART_HandleTypeDef *huart,int a,int b)
{
	if(	usa_count<1)	usa_count++;
	  usa_flag1=a;	usa_flag2=b;
	if(usa_count==1)
  {
	//	usart_send_char(huart,0xA5);
		usart_send_char(huart,message[a][b]);
		usa_count=2;
	}
	if(usa_flag11!=usa_flag1||usa_flag22!=usa_flag2)	usa_count=0;
  else usa_count=2;
  	usa_flag11=a;  usa_flag22=b;  
}



bool buchang_flag;
float Distance[2],timetostop=0;
float velocity,dist_record,keepmaxv_dist,keepmaxv_tim,dist_target,time_dist,timetoget,timetostopo,t1,t2,total_time=0;
float motion_time=1.6;
//float calculateVelocity(float flag,float realtime_distance,float Acceleration, float maxVelocity) 
//{

// if(!((super_control_Stack==Getting_motion2||super_control_Stack==Getting_motion2_B)&&motor_supestratum[2].total_angle==1&&realtime_distance==0))
//	{
//	 dist_target=realtime_distance ;
//	}
//	else  HAL_TIM_Base_Start_IT(&htim4);	
//	
//	Distance[0]=(maxVelocity*maxVelocity)/(Acceleration*2);//0-》maxv  maxv-》0的路程 
//	Distance[1]=dist_target-(2*Distance[0]);//匀速maxv的路程
//	dist_record=dist_target-realtime_distance;

//	t1=Acceleration/	Distance[1];//匀速时长
//	t2=Acceleration/Distance[0];//加减速时长
//	total_time=2*t2+t1;    //总时长
//	timetostopo=v/(dist_target-Distance[0]);
//	
//	if(total_time<motion_time)  
//	{
//				 ball_canget=true;
//	}

//	 if(dist_target>=300)  velocity=maxVelocity;
//	else  velocity=0;
//	if(cnt==timetostopo) velocity=0;
//  	
//		
		
//		
//	 if(dist_target>=300)  velocity=maxVelocity;
//	  else  velocity=0;
//		
//		
//    return velocity;
//}

 int str,flag_1, p=0;
void value_init(void)
{
	 for(int i;i<4;i++)		 angle.tar_angle[i]=0;
    Lif_L_Init=motor_supestratum[2].total_angle;
		Lif_R_Init=motor_supestratum[3].total_angle;

	//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&	
	  if(pinstate[1]==true)		
	 {		
		Str_L_Init=motor_supestratum[4].total_angle;	
		Str_R_Init=motor_supestratum[5].total_angle;
		Sinit_flag1=false;		
			str=0;			
		}
		else 
		{
			if(Sinit_flag1==true)
			{
				Str_L_Init=motor_supestratum[4].total_angle;
				Str_R_Init=motor_supestratum[5].total_angle;
		  	str=-300;			
				}	
		}
		//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&	
		if(pinstate[2]==1)
		{			
		  	Pinit_flag1=false;
		  	Pit_Init=motor_supestratum[6].total_angle;							
		    p=pit_ball_init;	
			
		}
		//如果还没碰到限位开关，就运动，一边运动（定值）一边记录当下编码值
		else 
		{	
      			
		  if(Pinit_flag1==true)	
			{
				Pit_Init=motor_supestratum[6].total_angle;		
		    p=-800;	
				}						
		}
		control_sup(up_ball_init,-up_ball_init,-str,str,p);
}



int init_flag=0;
GPIO_PinState res[4]={0};
int mark1=0,mark2=0,mark3=0,mark4=0;

void GET_InitPinstate()
{		
		res[0]=HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
		res[1]=HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);
		res[2]=HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14);
		res[3]=HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15);
	
  for(int i =0;i<4;i++)
	{
		if(res[i]==GPIO_PIN_SET)		pinstate[i]=0;
    else  pinstate[i]=1;
		
	}
	if(pinstate[3])
	{
//	 if( mark3<400)mark3++;
//		if(mark3>400)		
		ball_g=true;	
	}
	else //mark3=0;  
		ball_g=false;
	
}
double calculate_auto_tar_angle(double t_angle, double distance,double liqiujuli) {
    // 将角度从度转换为弧度	
	
	  t_angle=t_angle * M_PI / 180 ;
    double numerator = (sin(t_angle) * distance) - liqiujuli;
    double denominator = cos(t_angle) * distance;
   double auto_tar_anglel = atan(numerator / denominator);
	
	auto_tar_anglel=auto_tar_anglel*180/M_PI;
	
	  return auto_tar_anglel;
	
	
}