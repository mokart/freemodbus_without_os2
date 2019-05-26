/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: mb.c,v 1.27 2007/02/18 23:45:41 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/

#include "mb.h"
#include "mbconfig.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbfunc.h"

#include "mbport.h"
#if MB_RTU_ENABLED == 1
#include "mbrtu.h"
#endif
#if MB_ASCII_ENABLED == 1
#include "mbascii.h"
#endif
#if MB_TCP_ENABLED == 1
#include "mbtcp.h"
#endif

#ifndef MB_PORT_HAS_CLOSE
#define MB_PORT_HAS_CLOSE 0
#endif

/* ----------------------- Static variables ---------------------------------*/
//本从机的地址
static UCHAR    ucMBAddress;

//static eMBMode  eMBCurrentMode;

static enum
{
    STATE_ENABLED,
    STATE_DISABLED,
    STATE_NOT_INITIALIZED
} eMBState = STATE_NOT_INITIALIZED;

/* Functions pointer which are initialized in eMBInit( ). Depending on the
 * mode (RTU or ASCII) the are set to the correct implementations.
 */
static peMBFrameSend peMBFrameSendCur;
static pvMBFrameStart pvMBFrameStartCur;
static pvMBFrameStop pvMBFrameStopCur;
static peMBFrameReceive peMBFrameReceiveCur;
static pvMBFrameClose pvMBFrameCloseCur;

/* Callback functions required by the porting layer. They are called when
 * an external event has happend which includes a timeout or the reception
 * or transmission of a character.
 */
BOOL( *pxMBFrameCBByteReceived ) ( void );
BOOL( *pxMBFrameCBTransmitterEmpty ) ( void );
BOOL( *pxMBPortCBTimerExpired ) ( void );

BOOL( *pxMBFrameCBReceiveFSMCur ) ( void );
BOOL( *pxMBFrameCBTransmitFSMCur ) ( void );

/* An array of Modbus functions handlers which associates Modbus function
 * codes with implementing functions.
 */
static xMBFunctionHandler xFuncHandlers[MB_FUNC_HANDLERS_MAX] = {
#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
    {MB_FUNC_OTHER_REPORT_SLAVEID, eMBFuncReportSlaveID},
#endif
#if MB_FUNC_READ_INPUT_ENABLED > 0
    {MB_FUNC_READ_INPUT_REGISTER, eMBFuncReadInputRegister},
#endif
#if MB_FUNC_READ_HOLDING_ENABLED > 0
    {MB_FUNC_READ_HOLDING_REGISTER, eMBFuncReadHoldingRegister},
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_REGISTERS, eMBFuncWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_REGISTER, eMBFuncWriteHoldingRegister},
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
    {MB_FUNC_READWRITE_MULTIPLE_REGISTERS, eMBFuncReadWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_READ_COILS_ENABLED > 0
    {MB_FUNC_READ_COILS, eMBFuncReadCoils},
#endif
#if MB_FUNC_WRITE_COIL_ENABLED > 0
    {MB_FUNC_WRITE_SINGLE_COIL, eMBFuncWriteCoil},
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_COILS, eMBFuncWriteMultipleCoils},
#endif
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
    {MB_FUNC_READ_DISCRETE_INPUTS, eMBFuncReadDiscreteInputs},
#endif
};

/* ----------------------- Start implementation -----------------------------*/
/*函数功能： 
*1:实现RTU模式和ASCALL模式的协议栈初始化; 
*2:完成协议栈核心函数指针的赋值，包括Modbus协议栈的使能和禁止、报文的接收和响应、3.5T定时器中断回调函数、串口发送和接收中断回调函数; 
*3:eMBRTUInit完成RTU模式下串口和3.5T定时器的初始化，需用户自己移植; 
*4:设置Modbus协议栈的模式eMBCurrentMode为MB_RTU，设置Modbus协议栈状态eMBState为STATE_DISABLED; 
*/  
eMBErrorCode
eMBInit( eMBMode eMode, UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity )
{
    //错误状态初始值
    eMBErrorCode    eStatus = MB_ENOERR;

    /* check preconditions 检查先决条件*/
	//验证从机地址
	//
    if( ( ucSlaveAddress == MB_ADDRESS_BROADCAST ) ||
        ( ucSlaveAddress < MB_ADDRESS_MIN ) || ( ucSlaveAddress > MB_ADDRESS_MAX ) )
    {
        eStatus = MB_EINVAL;
    }
    else
    {
        ucMBAddress = ucSlaveAddress;//设置本从机的地址

        switch ( eMode )
        {
#if MB_RTU_ENABLED > 0
        case MB_RTU:
            pvMBFrameStartCur = eMBRTUStart;   //RTU模式开始函数
            pvMBFrameStopCur = eMBRTUStop;     //RTU模式终止函数
            peMBFrameSendCur = eMBRTUSend;     //RTU回复帧信息组织函数
            peMBFrameReceiveCur = eMBRTUReceive;  //RTU接收数据帧信息提取函数
            pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBPortClose : NULL;  //关闭串口的发送使能和接收使能
            pxMBFrameCBByteReceived = xMBRTUReceiveFSM;//接收状态机函数，接收中断调用
            pxMBFrameCBTransmitterEmpty = xMBRTUTransmitFSM;//发送状态机函数，供发送中断调用
            pxMBPortCBTimerExpired = xMBRTUTimerT35Expired; //超时中断发生时候处理的事务，供中断超时调用

            eStatus = eMBRTUInit( ucMBAddress, ucPort, ulBaudRate, eParity );  //RTU串口初始化，和3.5T定时器初始化
            break;
#endif
#if MB_ASCII_ENABLED > 0
        case MB_ASCII:
            pvMBFrameStartCur = eMBASCIIStart;
            pvMBFrameStopCur = eMBASCIIStop;
            peMBFrameSendCur = eMBASCIISend;
            peMBFrameReceiveCur = eMBASCIIReceive;
            pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBPortClose : NULL;
            pxMBFrameCBByteReceived = xMBASCIIReceiveFSM;
            pxMBFrameCBTransmitterEmpty = xMBASCIITransmitFSM;
            pxMBPortCBTimerExpired = xMBASCIITimerT1SExpired;

            eStatus = eMBASCIIInit( ucMBAddress, ucPort, ulBaudRate, eParity );
            break;
#endif
        default:
            eStatus = MB_EINVAL;
        }

        if( eStatus == MB_ENOERR )
        {
            if( 0==xMBPortEventInit(  ) )
            {
                /* port dependent event module initalization failed. */
                eStatus = MB_EPORTERR;
            }
            else
            {
                //eMBCurrentMode = eMode;   //当前模式设置为RTU模式
                eMBState = STATE_DISABLED;  //MODBUS协议状态初始化，此处初始化为禁止
            }
        }
    }
    return eStatus;
}

#if MB_TCP_ENABLED > 0
eMBErrorCode
eMBTCPInit( USHORT ucTCPPort )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( ( eStatus = eMBTCPDoInit( ucTCPPort ) ) != MB_ENOERR )
    {
        eMBState = STATE_DISABLED;
    }
    else if( !xMBPortEventInit(  ) )
    {
        /* Port dependent event module initalization failed. */
        eStatus = MB_EPORTERR;
    }
    else
    {
        pvMBFrameStartCur = eMBTCPStart;
        pvMBFrameStopCur = eMBTCPStop;
        peMBFrameReceiveCur = eMBTCPReceive;
        peMBFrameSendCur = eMBTCPSend;
        pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBTCPPortClose : NULL;
        ucMBAddress = MB_TCP_PSEUDO_ADDRESS;
        eMBCurrentMode = MB_TCP;
        eMBState = STATE_DISABLED;
    }
    return eStatus;
}
#endif

eMBErrorCode
eMBRegisterCB( UCHAR ucFunctionCode, pxMBFunctionHandler pxHandler )
{
    int             i;
		//OS_CPU_SR cpu_sr=0;
    eMBErrorCode    eStatus;

    if( ( 0 < ucFunctionCode ) && ( ucFunctionCode <= 127 ) )
    {
        ENTER_CRITICAL_SECTION(  );
        if( pxHandler != NULL )
        {
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                if( ( xFuncHandlers[i].pxHandler == NULL ) ||
                    ( xFuncHandlers[i].pxHandler == pxHandler ) )
                {
                    xFuncHandlers[i].ucFunctionCode = ucFunctionCode;
                    xFuncHandlers[i].pxHandler = pxHandler;
                    break;
                }
            }
            eStatus = ( i != MB_FUNC_HANDLERS_MAX ) ? MB_ENOERR : MB_ENORES;
        }
        else
        {
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                if( xFuncHandlers[i].ucFunctionCode == ucFunctionCode )
                {
                    xFuncHandlers[i].ucFunctionCode = 0;
                    xFuncHandlers[i].pxHandler = NULL;
                    break;
                }
            }
            /* Remove can't fail. */
            eStatus = MB_ENOERR;
        }
        EXIT_CRITICAL_SECTION(  );
    }
    else
    {
        eStatus = MB_EINVAL;
    }
    return eStatus;
}


eMBErrorCode
eMBClose( void )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( eMBState == STATE_DISABLED )
    {
        if( pvMBFrameCloseCur != NULL )
        {
            pvMBFrameCloseCur(  );
        }
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

/*函数功能
*1：设置Modbus协议栈工作状态eMBState为STATE_ENABLED;
*2: 调用pvMBFrameStartCur()函数激活协议栈
*/
eMBErrorCode
eMBEnable( void )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( eMBState == STATE_DISABLED )
    {
        /* Activate the protocol stack. */
		//pvMBFrameStartCur = eMBRTUStart;
        pvMBFrameStartCur(  );
        eMBState = STATE_ENABLED;
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode
eMBDisable( void )
{
    eMBErrorCode    eStatus;

    if( eMBState == STATE_ENABLED )
    {
        pvMBFrameStopCur(  );
        eMBState = STATE_DISABLED;
        eStatus = MB_ENOERR;
    }
    else if( eMBState == STATE_DISABLED )
    {
        eStatus = MB_ENOERR;
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

//功能 : 轮询事件查询处理函数
//描述 : 用户需在主循环中调用此函数。对于使用操作系统的程序，应单独创建一个任务，使操作系统能周期调用此函数。
//功能 : 1.检查协议栈是否使能，eMBState初始值为STATE_NOT_INITIALIZED,在eMBInit()函数中被赋值为STATE_DISABLE,
//       在 eMBEnable 函数中被赋值为STATE_ENABLE;
//       2.轮询EV_FRAME_RECEIVED事件发生，若EV_FRAM_RECEIVED事件发生，接收一帧报文数据，上报EV_EXECUTE事件，
//       解析一帧报文，相应（发送）一帧数据给主机。

//     在第二阶段，从机接收到一帧完整的报文后，上报“接收到报文”事件，eMBPoll函数轮询，发现“接收到报文”事件
//     发生，调用peMBFrameReceiveCur函数，该函数指针在eMBInit被赋值eMBRTUReceive函数，最终调用eMBRTUReceive
//     函数，从ucRTUBuf中取得从机地址、PDU单元和PDU单元的长度，然后判断从机地址是否一致，若一致，上报
//     "报文解析事件"EV_EXECUTE,xMBPortEventPost( EV_EXECUTE );"报文解析事件"发生后，根据功能码，调用 
//      xFuncHandlers[i].pxHandler( ucMBFrame, &usLength ) 对报文进行解析，此过程全部在eMBPoll函数中执行；

eMBErrorCode eMBPoll( void )
{
    static UCHAR   *ucMBFrame;       //接收和发送报文数据缓存区
    static UCHAR    ucRcvAddress;    //modbus从机地址
    static UCHAR    ucFunctionCode;  //功能码
    static USHORT   usLength;        //报文长度
    static eMBException eException;  //错误码响应枚举

    int             i;
    eMBErrorCode    eStatus = MB_ENOERR;  //modbus协议栈错误码
    eMBEventType    eEvent;               //事件标志枚举

    /* Check if the protocol stack is ready. *///检查协议栈是否使能
    if( eMBState != STATE_ENABLED )
    {
        return MB_EILLSTATE;          //协议栈未使能，返回协议栈无效错误码
    }

    /* Check if there is a event available. If not return control to caller.
     * Otherwise we will handle the event. */

	//查询事件
    if( xMBPortEventGet( &eEvent ) == TRUE )   //查询哪个事件发生
    {
        switch ( eEvent )
        {
        case EV_READY:
            break;

        case EV_FRAME_RECEIVED:              //接收到一帧数据，此事件发生
					  //            peMBFrameReceiveCur = eMBRTUReceive;
            eStatus = peMBFrameReceiveCur( &ucRcvAddress, &ucMBFrame, &usLength );//查询接收结果，并将结果保存到ucMBFrame
            if( eStatus == MB_ENOERR )           //报文长度和CRC校验正确
            {
                /* Check if the frame is for us. If not ignore the frame. */
			    //判断接收到的报文数据是否可接受，如果是，处理报文数据
                if( ( ucRcvAddress == ucMBAddress ) || ( ucRcvAddress == MB_ADDRESS_BROADCAST ) )
                {
                    ( void )xMBPortEventPost( EV_EXECUTE );  //修改事件标志为EV_EXECUTE执行事件
                }
            }
            break;

        case EV_EXECUTE:                                //对接收到的报文进行处理数据
            ucFunctionCode = ucMBFrame[MB_PDU_FUNC_OFF];//获取PDU中的第一个字节为功能码
            eException = MB_EX_ILLEGAL_FUNCTION;        //赋值错误初始值为无效的功能码
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )//依次查询各个功能码
            {
                /* No more function handlers registered. Abort. */
			    //没有更多的功能处理寄存器
                if( xFuncHandlers[i].ucFunctionCode == 0 )
                {
                    break;
                }
                else if( xFuncHandlers[i].ucFunctionCode == ucFunctionCode )//相等，则说明本机支持此功能码
                {
                    eException = xFuncHandlers[i].pxHandler( ucMBFrame, &usLength );//执行相应的功能码的处理函数
                    break;
                }
            }

            /* If the request was not sent to the broadcast address we
             * return a reply. */
            if( ucRcvAddress != MB_ADDRESS_BROADCAST )
            {
                if( eException != MB_EX_NONE )     //接收到的报文有错误
                {
                    /* An exception occured. Build an error frame. */
                    usLength = 0;                                     //响应发送数据的首字节为从机地址
                    ucMBFrame[usLength++] = ( UCHAR )( ucFunctionCode | MB_FUNC_ERROR );//响应发送数据帧的第二个字节，功能码最高位置1
                    ucMBFrame[usLength++] = eException;               //响应发送数据帧的第三个字节为错误码标识
                }
				//解析完一帧完整的报文后，eMBPoll()函数中调用peMBFrameSendCur()函数进行响应，peMBFrameSendCur是
				//函数指针，最终会调用 eMBRTUSend() 函数发送响应；
                eStatus = peMBFrameSendCur( ucMBAddress, ucMBFrame, usLength );//modbus从机响应函数，发送响应给主机
            }
            break;

        case EV_FRAME_SENT:
            break;
        }
    }
    return MB_ENOERR;
}
