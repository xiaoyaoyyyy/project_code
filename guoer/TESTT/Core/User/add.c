#include "add.h"
#include "auto.h"

int tar_arr[5]={0,0,0,0.0};
int target_limit[5]={0,0,0,0,0};
bool add_flag;
int flag_change[5],last_flag_change[5];

#define INCREAMENT 400
int ccccvT,ccccvR,ccccvT2,ccccvR2;
void sup_add_task()
{
		// ���� ERR �������ӵ� GPIOx_PINx ��
if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_SET) {
    // ����״̬����Ϊ�ߵ�ƽ�������շ����д���
   ccccvT++;
}
if (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_9) == GPIO_PIN_SET) {
    // ����״̬����Ϊ�ߵ�ƽ�������շ����д���
   ccccvR++;
}
if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) {
    // ����״̬����Ϊ�ߵ�ƽ�������շ����д���
   ccccvT2++;
}
if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET) {
    // ����״̬����Ϊ�ߵ�ƽ�������շ����д���
   ccccvR2++;
}
if(Pinit_flag1==true&&control_current==A_init)
{
	for(int i=0;i<5;i++)	
	{
    flag_change[i]=target_limit[i]	;		
  	if(	last_flag_change[i]!=flag_change[i])
			tar_arr[i]=0;		
	}
}
for (int i = 0; i < 5; i++)
	{
			if (tar_arr[i] < target_limit[i]) 
				{
					tar_arr[i] += INCREAMENT;
					
					if(i==3)tar_arr[i] += 160;
					if(i==4)tar_arr[i] += 8600;
					
					if (tar_arr[i] > target_limit[i])
						{
							tar_arr[i] = target_limit[i]; // ��ֹ����Ŀ��ֵ
						}

				}
				if (tar_arr[i] > target_limit[i]) 
				{
					tar_arr[i] -= INCREAMENT;
					if(i==2)tar_arr[i] -= 160;
					if(i==4)tar_arr[i] -= 8500;
					if (tar_arr[i] < target_limit[i])
						{
							tar_arr[i] = target_limit[i]; // ��ֹ����Ŀ��ֵ
						}
				}
    }
	for(int i=0;i<5;i++)	
	{
  last_flag_change[i]=target_limit[i];
	}
}
