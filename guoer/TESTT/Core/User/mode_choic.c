#include "mode_choic.h"


uint8_t control_current_State=None;
uint8_t control_current=M_None;
int int2=0;
void mode_judge()
{
 control_current_State=Reset;

	//��
  if(  sbus_struct.sbus_channels[5] == Remote_switch_Down) 
	{		
		
		//���� ����
			if(sbus_struct.sbus_channels[6] == Remote_switch_Up  )
			{
			
			control_current_State=N_power;
		
			}
		
		  //���� �Զ�
			else if(sbus_struct.sbus_channels[6] == Remote_switch_Middle  )
	  	{
			
			control_current_State=Auto;
				
				//�����ϳ�ʼ��
				if(sbus_struct.sbus_channels[7] == Remote_switch_Up )
					{ 
						control_current=A_init;
						  int2=1;
					}
				//������   ��λ
					if(sbus_struct.sbus_channels[7] == Remote_switch_Middle )
					{ 
						control_current=A_reset;
						
					}
				//������    ��
					else if(sbus_struct.sbus_channels[7] == Remote_switch_Down  )
					{
						control_current=A_power;
				
				  }		
			}
		//���� �ֶ�
			else if ( sbus_struct.sbus_channels[6] == Remote_switch_Down  )
			{
				control_current_State=Manual;
			
			//������ ����
					if(sbus_struct.sbus_channels[7] == Remote_switch_Up )
					{
						control_current =M_power;
						
					}
					//������ �ϲ�
					if(sbus_struct.sbus_channels[7] == Remote_switch_Middle  )
						
					{
						control_current=M_sup;
					}
					//������ ����
					else if(sbus_struct.sbus_channels[7] == Remote_switch_Down  )
						{
							control_current=M_chas;
					}			 
	   }	
   }
	//��
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


