#include "delay.h"

static u32 fac_us=0;		

void delay_init(u8 SYSCLK)
{
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	fac_us=SYSCLK;						 
}								    

void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;				
	ticks=nus*fac_us; 						
	told=SysTick->VAL;    
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;		
		}  
	};
}
 
void delay_ms(u16 nms)
{
	u32 i;
	for(i=0;i<nms;i++) delay_us(1000);
}

			 
