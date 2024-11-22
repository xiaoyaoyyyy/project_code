#include "pid.h"
#include "niming.h"
#include "CAN_receive.h"
#include "chassis.h"
#include <stdlib.h>
#include <math.h>

extern PID_chassis_TypeDef target,output;
void PID_SET(PID_Typedef *PID, float target, float P, float I, float D)
{
  PID->target_val = target;   //���ó�ʼĿ��ֵ

  PID->Kp = P;//���ñ���ֵ
  PID->Ki = I;//���û���ֵ
  PID->Kd = D;//���΢��ֵ
}


void PID_Init(PID_Typedef *PID)
{
	PID->target_val = 0.0;
  PID->actual_val=0.0;
  PID->deviation_val = 0.0;
  PID->deviation_val_last = 0.0;
  PID->deviation_val_ll = 0.0;
  
	PID->err = 0.0;
  PID->last_err=0.0;
  PID->before_err = 0.0;
  PID->total_err = 0.0;
  PID->integral = 0.0;
	
  PID->Kp = 0.0;
  PID->Ki = 0.0;
  PID->Kd = 0.0;

}
float err;
float PID_incremental(PID_Typedef *PID, double target, float actual, float limit)
{	
	/*PID�������ֵ--PID_OUT; ��������ֵ--I_OUT; ��������ֵ--P_OUT; ΢������ֵ--D_OUT; ���ֶ�̬�޷�ֵ*/
	float increment_val=0, Proportion_val=0, Integral_val=0, Differential_val=0;
  //����Ŀ��ֵ��ʵ��ֵ�����
	 err=target-actual;
//	if(fabs(target-actual)>30)
//	{
	PID->target_val=target;
	PID->actual_val=actual;
	PID->deviation_val = (target - actual);
//	}
//	else PID->deviation_val =0;
  //PID�㷨ʵ��
	Proportion_val = PID->Kp * (PID->deviation_val - PID->deviation_val_last);	//��������: Kp * (��ǰ���ֵ - �ϴ����ֵ)
	Integral_val   = PID->Ki * PID->deviation_val;	//��������: Ki * ��ǰ���ֵ
	Differential_val = PID->Kd * (PID->deviation_val - 2*PID->deviation_val_last + PID->deviation_val_ll);	//΢������: Kd * ����ǰ���ֵ - 2*�ϴ����ֵ + ���ϴ����ֵ��
	
  increment_val = Proportion_val+ Integral_val + Differential_val; //P_OUT +I_OUT +D_OUT
  
  PID->ouput_val += increment_val;//�ۼӼ������ֵ
  PID->ouput_val = (PID->ouput_val <= -limit) ? -limit : (PID->ouput_val >=  limit) ?  limit : PID->ouput_val;	
  //�������
	PID->deviation_val_ll = PID->deviation_val_last;
	PID->deviation_val_last = PID->deviation_val;

	//���ص�ǰʵ��ֵ
	return PID-> ouput_val;
}


float PID_position(PID_Typedef * PID,float target1, float temp_val,  float confine, float limit)
 {
//float limit_siqu=70
	/*����Ŀ��ֵ��ʵ��ֵ�����*/
	 PID->target_val=target1;
	 PID->actual_val= temp_val;
	PID->err = target1 -temp_val;
	/*�����ۼ�*/ 
	PID-> total_err += PID->err;
	 
	/*�����޷�*/
	PID->total_err = PID->total_err >= confine ? confine:PID->total_err;
	PID->total_err = PID->total_err <= -confine ? -confine:PID->total_err;
	 
	/*λ��ʽPID��ʵ��*/ 
	PID->ouput_val = PID->Kp * PID->err 
									+ PID->Ki * PID-> total_err 
									+ PID->Kd *( PID->err-PID->last_err);
	      
 	/*���ֵ�޷�*/
	PID->ouput_val = PID->ouput_val >= limit ? limit:PID->ouput_val;
	PID->ouput_val = PID->ouput_val <= -limit ? -limit:PID->ouput_val;
//	 	
//	 if(abs(PID->err) <= abs(limit_siqu))
//	{
//	  PID->total_err=0;
//	  PID->ouput_val=0;
//	} 
//	 
//	 
//	printf("ʵ��=%d,Ŀ��=%d, ���=%d, KP = %d ,KI = %d , KD = %d\r\n",temp_val,target,PID->actual_val,);
	 
	PID->last_err=PID->err;
	return PID->ouput_val;
	
	// PID->actual_val=0;
 }


void pid_set(int16_t taget1,int16_t taget2,int a,int b)
{
		send_sensorData(0,
										taget1, 
		                taget2,		             
		                motor_chassis[a].total_angle,
										motor_chassis[b].speed_rpm,
										0);	
}


