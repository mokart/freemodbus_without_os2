#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "key.h"
//ALIENTEK Mini STM32开发板范例代码3
//串口实验   
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define REG_INPUT_START     1
#define REG_INPUT_NREGS     20
#define REG_HOLDING_START 	1
#define REG_HOLDING_NREGS 	10
/* ----------------------- Static variables ---------------------------------*/
static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegInputBuf[REG_INPUT_NREGS];               //REG_INPUT_NREGS 20字节
static USHORT   usRegHoldingStart = REG_HOLDING_START;
static USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];           //REG_HOLDING_NREGS 10字节
/* ----------------------- Start implementation -----------------------------*/


int main(void)
{

	Stm32_Clock_Init(9); //系统时钟设置
	delay_init(72);	     //延时初始化 
	uart_init(72,9600);	 //串口初始化为9600
	LED_Init();		  	 //初始化与LED连接的硬件接口    
  printf("uart test:\r\n");
	eMBInit(MB_RTU, 0x02, 0, 9600, MB_PAR_NONE);
	eMBEnable();
	
	while(1)
	{
		  //printf("main runing:\r\n");
		  LED0 = !LED0;
		  eMBPoll();
			delay_ms(1000);   
		
	}
}



/****************************************************************************
* 名	  称：eMBRegInputCB 
* 功    能：读取输入寄存器，对应功能码是 04 eMBFuncReadInputRegister
* 入口参数：pucRegBuffer: 数据缓存区，用于响应主机   
*						usAddress: 寄存器地址
*						usNRegs: 要读取的寄存器个数
* 出口参数：
* 注	  意：上位机发来的 帧格式是: SlaveAddr(1 Byte)+FuncCode(1 Byte)
*								+StartAddrHiByte(1 Byte)+StartAddrLoByte(1 Byte)
*								+LenAddrHiByte(1 Byte)+LenAddrLoByte(1 Byte)+
*								+CRCAddrHiByte(1 Byte)+CRCAddrLoByte(1 Byte)
*							3 区
****************************************************************************/
//功能: 输入寄存器读回函数。
//参数 : * pucRegBuffer为要添加到协议中的数据，usAddress为输入寄存器地址，usNRegs为要读取寄存器的个数。
//描述: 用户应根据要访问的寄存器地址usAddress将相应输入寄存器的值按顺序添加到pucRegBuffer中。
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ = ( UCHAR )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ = ( UCHAR )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

/****************************************************************************
* 名	  称：eMBRegHoldingCB 
* 功    能：对应功能码有：06 写保持寄存器 eMBFuncWriteHoldingRegister 
*						  16 写多个保持寄存器 eMBFuncWriteMultipleHoldingRegister
*						  03 读保持寄存器 eMBFuncReadHoldingRegister
*						  23 读写多个保持寄存器 eMBFuncReadWriteMultipleHoldingRegister
* 入口参数：pucRegBuffer: 数据缓存区，用于响应主机   
*						usAddress: 寄存器地址
*						usNRegs: 要读写的寄存器个数
*						eMode: 功能码
* 出口参数：
* 注	  意：4 区
****************************************************************************/
//功能 : 保持寄存器回函数。
//参数 : * pucRegBuffer为要协议中的数据，usAddress为输入寄存器地址，
//       usNRegs为访问寄存器的个数，eMode为访问类型（MB_REG_READ为读保持寄存器，MB_REG_WRITE为写保持寄存器）
//描述 : 用户应根据要访问的寄存器地址usAddress将相应输入寄存器的值按顺序添加到pucRegBuffer中，
//       或将协议中的数据根据要访问的寄存器地址usAddress放到相应保持寄存器中。
eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
	eMBErrorCode    eStatus = MB_ENOERR;
	int             iRegIndex;


	if((usAddress >= REG_HOLDING_START)&&\
		((usAddress+usNRegs) <= (REG_HOLDING_START + REG_HOLDING_NREGS)))
	{
		iRegIndex = (int)(usAddress - usRegHoldingStart);
		switch(eMode)
		{                                       
			case MB_REG_READ://读 MB_REG_READ = 0
       			 	while(usNRegs > 0)
					{
					*pucRegBuffer++ = (u8)(usRegHoldingBuf[iRegIndex] >> 8);            
					*pucRegBuffer++ = (u8)(usRegHoldingBuf[iRegIndex] & 0xFF); 
         			 iRegIndex++;
         			 usNRegs--;					
					}                            
        break;
			case MB_REG_WRITE://写 MB_REG_WRITE = 1
				while(usNRegs > 0)
				{         
					  usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
			          usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
			          iRegIndex++;
			          usNRegs--;
        }				
			}
	}
	else//错误
	{
		eStatus = MB_ENOREG;
	}	
	
	return eStatus;
}

/****************************************************************************
* 名	  称：eMBRegCoilsCB 
* 功    能：对应功能码有：01 读线圈 eMBFuncReadCoils
*													05 写线圈 eMBFuncWriteCoil
*													15 写多个线圈 eMBFuncWriteMultipleCoils
* 入口参数：pucRegBuffer: 数据缓存区，用于响应主机   
*						usAddress: 线圈地址
*						usNCoils: 要读写的线圈个数
*						eMode: 功能码
* 出口参数：
* 注	  意：如继电器 
*						0 区
****************************************************************************/
//功能 : 读写线圈回函数。
//参数 : * pucRegBuffer为要添加到协议中的数据，usAddress为线圈地址，usNCoils为要访问线圈的个数，
//       eMode为访问类型（MB_REG_READ为读线圈状态，MB_REG_WRITE为写线圈）。
//描述 : 用户应根据要访问的线圈地址usAddress将相应线圈的值按顺序添加到pucRegBuffer中，
//       或将协议中的数据根据要访问的线圈地址usAddress放到相应线圈中。
eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
	eMBErrorCode    eStatus = MB_ENOERR;
	int             iRegIndex;


	if((usAddress >= REG_HOLDING_START)&&\
		((usAddress+usNCoils) <= (REG_HOLDING_START + REG_HOLDING_NREGS)))
	{
		iRegIndex = (int)(usAddress - usRegHoldingStart);
		switch(eMode)
		{                                       
			case MB_REG_READ://读 MB_REG_READ = 0
        while(usNCoils > 0)
				{
 					*pucRegBuffer++ = (u8)(usRegHoldingBuf[iRegIndex] >> 8);            
 					*pucRegBuffer++ = (u8)(usRegHoldingBuf[iRegIndex] & 0xFF); 
          			iRegIndex++;
          			usNCoils--;					
				}                            
        break;
			case MB_REG_WRITE://写 MB_REG_WRITE = 1
				while(usNCoils > 0)
				{         
 					usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
           		usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
          			iRegIndex++;
          			usNCoils--;
        }				
			}
	}
	else//错误
	{
		eStatus = MB_ENOREG;
	}	
	
	return eStatus;
}
/****************************************************************************
* 名	  称：eMBRegDiscreteCB 
* 功    能：读取离散寄存器，对应功能码有：02 读离散寄存器 eMBFuncReadDiscreteInputs
* 入口参数：pucRegBuffer: 数据缓存区，用于响应主机   
*						usAddress: 寄存器地址
*						usNDiscrete: 要读取的寄存器个数
* 出口参数：
* 注	  意：1 区
****************************************************************************/
//功能 : 读离散线圈回函数
//参数 : * pucRegBuffer为要添加到协议中的数据，usAddress为线圈地址，usNDiscrete为要访问线圈的个数。
//
eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    ( void )pucRegBuffer;
    ( void )usAddress;
    ( void )usNDiscrete;
    return MB_ENOREG;
}













