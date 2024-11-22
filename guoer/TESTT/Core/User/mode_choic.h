#ifndef _MODE_CHOIS_H
#define _MODE_CHOIS_H
#include "stm32f4xx_hal.h"
#include "sbus.h"
#include "superstratum.h"

void mode_judge(void);
void mode_choic(void);
extern uint8_t control_current_State;
extern uint8_t control_current;
extern int init_flag,int2;

typedef enum{
	
	None=0,
	Reset,
	Auto,
	Manual,
  N_power,

}control_State_stack;//×´Ì¬»ú

typedef enum{
	
	M_None=0,
	M_power,
	M_sup,
	M_chas,
	A_reset,
	A_power,
	A_init,
	
}control_stack;
	

#endif
