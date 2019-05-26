/*
 * FreeModbus Libary: LPC214X Port
 * Copyright (C) 2007 Tiago Prado Lone <tiago@maxwellbohr.com.br>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: porttimer.c,v 1.1 2007/04/24 23:15:18 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "sys.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"


//通用定时器中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
void TIM4_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=(1<<2);	          //TIM6时钟使能    
 	TIM4->ARR=arr;  	              //设定计数器自动重装值 
	TIM4->PSC=psc-1;  	            //预分频器设置
	TIM4->DIER|=(1<<0);             //允许更新中断				
	TIM4->CR1&=~(1<<0);             //禁止定时器6
  MY_NVIC_Init(1,3,TIM4_IRQn,2);  //抢占1，子优先级3，组2									 
}

//5ms
void Tim4_Clear_Enable(void)
{
		TIM4->CR1&=~(1<<0);     //禁用定时器4
    TIM4->CNT = 0;          //清零计数
	 	TIM4->ARR=50;  	        //设定计数器自动重装值 
	  TIM4->PSC=7200-1;  	    //预分频器设置
		TIM4->SR&=~(1<<0);      //清除中断标志位
	  TIM4->DIER|=(1<<0);     //允许更新中断				
		TIM4->CR1|=(1<<0);      //使能定时器6
}


void Tim4_Clear_Disable(void)
{
	  
    TIM4->CNT = 0;             //清零计数
		TIM4->SR&=~(1<<0);         //清除中断标志位
		TIM4->DIER &= ~(1<<0);     //禁止更新中断
		TIM4->CR1&=~(1<<0);        //禁用定时器6

}

/* ----------------------- Start implementation -----------------------------*/
//用户应该根据所使用的硬件初始化超时定时器，使之能产生中断时间为usTim1Timerout50us*50us的中断。
//返回值务必为TRUE
BOOL xMBPortTimersInit( USHORT usTim1Timerout50us )
{
	TIM4_Int_Init(50,7200-1);  //延时5ms
	return TRUE;
}


//功能 : 使能定时器功能
//描述 : 用户需要在此函数中清除中断标志位，清零定时器计数值，并重新使能定时器中断
void vMBPortTimersEnable()
{	
    Tim4_Clear_Enable();
}

//函数功能: 使能超时定时器
//描述: 用户需要在此函数中清除中断标志位、清零定时器计数器值，并关闭定时器中断
void vMBPortTimersDisable()
{
    Tim4_Clear_Disable();
}

//功能: 定时器中断函数
//描述: 此函数无需修改，只需在用户的定时器中断中调用此函数即可，同时，用户应在调用此函数后清除中断标志位
void TIMERExpiredISR(void)
{

  	//OSIntEnter();		//进入中断
	  //pxMBPortCBTimerExpired = xMBRTUTimerT35Expired;
    (void)pxMBPortCBTimerExpired();
    //OSIntExit();        //触发任务切换软中断

}

void TIM4_IRQHandler(void)
{
	
	if(TIM4->SR&0X0001)//溢出中断
	{
	    TIMERExpiredISR();
			TIM4->SR&=~(1<<0);   //清除中断标志位
	}
	
}


