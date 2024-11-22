#include "mode_choic.h"


uint8_t control_current_State=None;
uint8_t control_current=M_None;
int int2=0;
void mode_judge()
{
 control_current_State=Reset;

	//下
  if(  sbus_struct.sbus_channels[5] == Remote_switch_Down) 
	{		
		
		//下上 无力
			if(sbus_struct.sbus_channels[6] == Remote_switch_Up  )
			{
			
			control_current_State=N_power;
		
			}
		
		  //下中 自动
			else if(sbus_struct.sbus_channels[6] == Remote_switch_Middle  )
	  	{
			
			control_current_State=Auto;
				
				//下中上初始化
				if(sbus_struct.sbus_channels[7] == Remote_switch_Up )
					{ 
						control_current=A_init;
						  int2=1;
					}
				//下中中   复位
					if(sbus_struct.sbus_channels[7] == Remote_switch_Middle )
					{ 
						control_current=A_reset;
						
					}
				//下中下    动
					else if(sbus_struct.sbus_channels[7] == Remote_switch_Down  )
					{
						control_current=A_power;
				
				  }		
			}
		//下下 手动
			else if ( sbus_struct.sbus_channels[6] == Remote_switch_Down  )
			{
				control_current_State=Manual;
			
			//下下上 有力
					if(sbus_struct.sbus_channels[7] == Remote_switch_Up )
					{
						control_current =M_power;
						
					}
					//下下中 上层
					if(sbus_struct.sbus_channels[7] == Remote_switch_Middle  )
						
					{
						control_current=M_sup;
					}
					//下下下 底盘
					else if(sbus_struct.sbus_channels[7] == Remote_switch_Down  )
						{
							control_current=M_chas;
					}			 
	   }	
   }
	//上
	else if(  sbus_struct.sbus_channels[5] == Remote_switch_Up) 
	{	
			control_current_State=N_power;
		   control_current=M_None;
	}
		
	if(control_current_State==N_power)
	{		
			   M_flag =1;
				 M_flag2=1;
	}
	


	
}


