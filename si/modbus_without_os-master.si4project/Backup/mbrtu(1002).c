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
 * File: $Id: mbrtu.c,v 1.18 2007/09/12 10:15:56 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"
#include "usart.h"
/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbrtu.h"
#include "mbframe.h"

#include "mbcrc.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_SER_PDU_SIZE_MIN     4       /*!< Minimum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_MAX     256     /*!< Maximum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_CRC     2       /*!< Size of CRC field in PDU. */
#define MB_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */

// #define	DEBUG

/* ----------------------- Type definitions ---------------------------------*/
typedef enum
{
    STATE_RX_INIT,              /*!< Receiver is in initial state. */
    STATE_RX_IDLE,              /*!< Receiver is in idle state. */
    STATE_RX_RCV,               /*!< Frame is beeing received. */
    STATE_RX_ERROR              /*!< If the frame is invalid. */
} eMBRcvState;

typedef enum
{
    STATE_TX_IDLE,              /*!< Transmitter is in idle state. */
    STATE_TX_XMIT               /*!< Transmitter is in transfer state. */
} eMBSndState;

/* ----------------------- Static variables ---------------------------------*/
static volatile eMBSndState eSndState;

static volatile eMBRcvState eRcvState;//接收状态机全局标识

volatile UCHAR  ucRTUBuf[MB_SER_PDU_SIZE_MAX];//256 字节

static volatile UCHAR *pucSndBufferCur;
static volatile USHORT usSndBufferCount;

static volatile USHORT usRcvBufferPos;

/* ----------------------- Start implementation -----------------------------*/
//RTU模式的串口初始化函数和3.5T定时器初始化
//此函数中判断串行口初始化是否成功（通过判断串行口初始化函数的返回值实现。
//当然，查看返回值必然先调用该函数，从而完成端口初始化），如果成功，则根据波特率计算T35，初始化超时定时器。
eMBErrorCode
eMBRTUInit( UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity )
{
		//OS_CPU_SR cpu_sr=0;
    eMBErrorCode    eStatus = MB_ENOERR;
    ULONG           usTimerT35_50us;


    ( void )ucSlaveAddress;
    ENTER_CRITICAL_SECTION(  );

    /* Modbus RTU uses 8 Databits. */
    if( xMBPortSerialInit( ucPort, ulBaudRate, 8, eParity ) != TRUE )  //串口初始化
    {
        eStatus = MB_EPORTERR;
    }
    else
    {
        /* If baudrate > 19200 then we should use the fixed timer values
         * t35 = 1750us. Otherwise t35 must be 3.5 times the character time.
         */
        if( ulBaudRate > 19200 )
        {
            usTimerT35_50us = 35;       /* 1800us. */
        }
        else
        {
            /* The timer reload value for a character is given by:
             *
             * ChTimeValue = Ticks_per_1s / ( Baudrate / 11 )
             *             = 11 * Ticks_per_1s / Baudrate
             *             = 220000 / Baudrate
             * The reload for t3.5 is 1.5 times this value and similary
             * for t3.5.
             */
            usTimerT35_50us = ( 7UL * 220000UL ) / ( 2UL * ulBaudRate );
        }
        if( xMBPortTimersInit( ( USHORT ) usTimerT35_50us ) != TRUE )
        {
            eStatus = MB_EPORTERR;
        }
    }
    EXIT_CRITICAL_SECTION(  );

    return eStatus;
}

//功能 : RTU模式开始函数
//描述 : 函数主要功能是，将接收状态eRcvState设为STATE_RX_INIT（Receiver is in initial state），
//       使能串口的接收、关闭发送、使能定时器功能
void
eMBRTUStart( void )
{
//		OS_CPU_SR cpu_sr=0;
	
    #ifdef	DEBUG
	  printf("eMBRTUStart()\r\n");	
    #endif
    ENTER_CRITICAL_SECTION(  );
    /* Initially the receiver is in the state STATE_RX_INIT. we start
     * the timer and if no character is received within t3.5 we change
     * to STATE_RX_IDLE. This makes sure that we delay startup of the
     * modbus protocol stack until the bus is free.
     */
    eRcvState = STATE_RX_INIT;
	  //使能串口接收，禁止发送
    vMBPortSerialEnable( TRUE, FALSE );
	  //使能定时器功能
    vMBPortTimersEnable(  );

    EXIT_CRITICAL_SECTION(  );
}

//功能 : RTU模式终止函数
//描述 : 禁用串口接收、禁用串口发送、 禁用定时器
void
eMBRTUStop( void )
{
//		OS_CPU_SR cpu_sr=0;

#ifdef	DEBUG
	printf("eMBRTUStop()\r\n");	
#endif	
	
    ENTER_CRITICAL_SECTION(  );
    vMBPortSerialEnable( FALSE, FALSE );
    vMBPortTimersDisable(  );
    EXIT_CRITICAL_SECTION(  );
}

//功能 : RTU接收数据帧信息提取函数
//描述 : 将接收帧（存放于缓存）的地址指针赋给指针变量pucRcvAddress，将PDU编码首地址赋给指针* pucFrame，
//       将PDU长度地址赋给指针变量pusLength。使用指针访问缓存数组，而不是额外开辟缓存存放帧信息，大大减少了内存的开支。

//功能 : eMBPoll函数轮询到EV_FRAME_RECEIVED事件时，调用peMBFrameReceiveCur(),此函数是用户为函数指针peMBFrameReceiveCur()
//       的赋值。
//       此函数完成的功能：从一帧数据报文中，取得modbus从机地址给pucRcvAddress,PDU报文的长度给pusLength,PDU报文的首地址给
//       pucFrame,函数形参全部为地址传递
eMBErrorCode
eMBRTUReceive( UCHAR * pucRcvAddress, UCHAR ** pucFrame, USHORT * pusLength )
{
//	  OS_CPU_SR cpu_sr=0;
//    BOOL            xFrameReceived = FALSE;
    eMBErrorCode    eStatus = MB_ENOERR;

    #ifdef	DEBUG
	  printf("eMBRTUReceive()\r\n");	
    #endif	
	
    ENTER_CRITICAL_SECTION(  );
    assert( usRcvBufferPos < MB_SER_PDU_SIZE_MAX );  //断言宏，判断接收到的字节数<256，如果>256，终止程序

    /* Length and CRC check */
    if( ( usRcvBufferPos >= MB_SER_PDU_SIZE_MIN )
        && ( usMBCRC16( ( UCHAR * ) ucRTUBuf, usRcvBufferPos ) == 0 ) )
    {
        /* Save the address field. All frames are passed to the upper layed
         * and the decision if a frame is used is done there.
         */
        *pucRcvAddress = ucRTUBuf[MB_SER_PDU_ADDR_OFF];//取接收到的第一个字节，modbus从机地址

        /* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
         * size of address field and CRC checksum.
         */
        *pusLength = ( USHORT )( usRcvBufferPos - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC );   //减去3

        /* Return the start of the Modbus PDU to the caller. */
        *pucFrame = ( UCHAR * ) & ucRTUBuf[MB_SER_PDU_PDU_OFF];
       // xFrameReceived = TRUE;
    }
    else
    {
        eStatus = MB_EIO;
    }

    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

//功能 : RTU回复帧信息组织函数
//描述 : 函数的功能是，此函数首先使发送内容指针pucSndBufferCur指向pucFrame之前的一个地址，.
//       并将该地址内容填充为ucSlaveAddress，并使用直接访问方式向缓存数组ucRTUBuf的相应地址内存入CRC校验值。
//       注意，此函数中，对ucRTUBuf的访问既有间接方式（指针pucSndBufferCur与pucFrame），又有直接方式（直接向相应地址内写值），比较难理解。
//       回复帧组织完后，将发送状态eSndState设为STATE_TX_XMIT（Transmitter is in transfer state），并禁止接收使能发送。
//       发送一旦使能，就会进入发送中断，完成相应字符的发送。

//功能 : 对响应报文PDU前面加上从机地址
//       对响应报文PDU后加上CRC校验；
//       使能发送，启动传输
eMBErrorCode
eMBRTUSend( UCHAR ucSlaveAddress, const UCHAR * pucFrame, USHORT usLength )
{
//	OS_CPU_SR cpu_sr=0;
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          usCRC16;

#ifdef	DEBUG
	printf("eMBRTUSend()\r\n");	
#endif	
	
    ENTER_CRITICAL_SECTION(  );

    /* Check if the receiver is still in idle state. If not we where to
     * slow with processing the received frame and the master sent another
     * frame on the network. We have to abort sending the frame.
     */
     //检查接收器是否任然在空闲状态，如果不空闲我们推迟去处理接收
     //并且主机发送另一个帧。
     //我们必须终止发送当前帧。
    if( eRcvState == STATE_RX_IDLE )
    {
        /* First byte before the Modbus-PDU is the slave address. */
	    //首先在协议数据单元前面加上从机地址
        pucSndBufferCur = ( UCHAR * ) pucFrame - 1;
        usSndBufferCount = 1;

        /* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
		//现在拷贝Modbus的PDU到Modbus串行线PDU
        pucSndBufferCur[MB_SER_PDU_ADDR_OFF] = ucSlaveAddress;
        usSndBufferCount += usLength;

        /* Calculate CRC16 checksum for Modbus-Serial-Line-PDU. */
        usCRC16 = usMBCRC16( ( UCHAR * ) pucSndBufferCur, usSndBufferCount );
        ucRTUBuf[usSndBufferCount++] = ( UCHAR )( usCRC16 & 0xFF );
        ucRTUBuf[usSndBufferCount++] = ( UCHAR )( usCRC16 >> 8 );

        /* Activate the transmitter. */
		//发送状态
        eSndState = STATE_TX_XMIT;
        vMBPortSerialEnable( FALSE, TRUE );
    }
    else
    {
        eStatus = MB_EIO;
    }
    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

//功能 : 此函数描述了一个接收状态机，供接收中断调用
//描述 : 状态机中，首先完成串口接收寄存器读取，然后判断相应接收状态eRcvState，实现接收。
//       在STATE_RX_INIT状态，重置超时定时器，等待超时中断（超时中断会把eRcvState设为STATE_RX_IDLE）；
//       在STATE_RX_ERROR状态，同样会重置超时定时器等待超时中断；在STATE_RX_IDLE状态，会将接收字符个数置零，
//       同时向缓存数组ucRTUBuf中存入接收到的字符，跳入状态STATE_RX_RCV，并使重置超时定时器；
//       在STATE_RX_RCV状态，不断将接收到的字符存入缓存，并统计接收计数，重置超时定时器，接收计数大于帧最大长度时，会跳入STATE_RX_ERROR状态。
//       在任何一处发生超时中断，都会将状态eRcvState置为STATE_RX_IDLE。在接收过程（STATE_RX_RCV）中，发生超时中断，指示着一帧数据接收完成。
//功能 : 将接收到的数据存入ucRTUBuf[]中
//       usRcvBufferPos为全局变量，表示接收数据的个数
//       每接收到一个字节的数据，3.5T定时器清0
BOOL
xMBRTUReceiveFSM( void )
{
    BOOL            xTaskNeedSwitch = FALSE;
    UCHAR           ucByte;

    assert( eSndState == STATE_TX_IDLE );	     //确保没有数据在发送
	
    /* Always read the character. */
    ( void )xMBPortSerialGetByte( ( CHAR * ) & ucByte ); //从串口数据寄存器读取一个字节数据

    //根据不同的状态转移
    switch ( eRcvState )
    {
        /* If we have received a character in the init state we have to
         * wait until the frame is finished.
         */
    case STATE_RX_INIT:
        vMBPortTimersEnable( );      //开起3.5T定时器
        break;

        /* In the error state we wait until all characters in the
         * damaged frame are transmitted.
         */
    case STATE_RX_ERROR:
        vMBPortTimersEnable( );       //数据帧损坏，重启定时器，不保存串口接收的数据
        break;

        /* In the idle state we wait for a new character. If a character
         * is received the t1.5 and t3.5 timers are started and the
         * receiver is in the state STATE_RX_RECEIVCE.
         */
    case STATE_RX_IDLE:               //接收器空闲，开始接收，进入STATE_RX_RCV状态
        usRcvBufferPos = 0;
        ucRTUBuf[usRcvBufferPos++] = ucByte; //保存数据
        eRcvState = STATE_RX_RCV;

        /* Enable t3.5 timers. */
        vMBPortTimersEnable( );          //每次收到一个字节，都重启3.5T定时器
        break;

        /* We are currently receiving a frame. Reset the timer after
         * every character received. If more than the maximum possible
         * number of bytes in a modbus frame is received the frame is
         * ignored.
         */
    case STATE_RX_RCV:
        if( usRcvBufferPos < MB_SER_PDU_SIZE_MAX )
        {
            ucRTUBuf[usRcvBufferPos++] = ucByte;   //接收数据
        }
        else
        {
            eRcvState = STATE_RX_ERROR;  //一帧报文的字节数大于最大PDU长度，忽略超出的数据
        }
        vMBPortTimersEnable();             //每次收到一个字节，都重启3.5T定时器
        break;
    }

		
    return xTaskNeedSwitch;
}

//功能 : 此函数描述了一个发送状态机，供发送中断调用。
//描述 : 状态机中，判断相应发送状态eSndState，实现发送。在STATE_TX_IDLE状态，使能接收关闭发送；
//       在STATE_TX_XMIT状态，调用底层串口发送函数将缓存中的字符发送出去，并使发送指针加1，待发送字符数
//       减1，待发送数为0时，将向系统发送事件EV_FRAME_SENT（Frame sent），同时使能接收关闭发送，并转向STATE_TX_IDLE状态。
BOOL
xMBRTUTransmitFSM( void )
{
    BOOL            xNeedPoll = FALSE;

    assert( eRcvState == STATE_RX_IDLE );
#ifdef	DEBUG
	printf("xMBRTUTransmitFSM()\r\n");	
#endif
    switch ( eSndState )
    {
        /* We should not get a transmitter event if the transmitter is in
         * idle state.  */
    case STATE_TX_IDLE:
        /* enable receiver/disable transmitter. */
        vMBPortSerialEnable( TRUE, FALSE );
        break;

    case STATE_TX_XMIT:                  //发送器处于发送状态，在从机发送函数eMBRTUSend中赋值STATE_TX_XMIT
        /* check if we are finished. */
        if( usSndBufferCount != 0 )
        {
            //发送数据
            xMBPortSerialPutByte( ( CHAR )*pucSndBufferCur );
            pucSndBufferCur++;  /* next byte in sendbuffer. */
            usSndBufferCount--;
        }
        else
        {
            //传递任务，发送完成
            //协议栈事件状态赋值为EV_FRAM_SENT,发送完成事件，eMBPoll函数会对此事件进行处理
            xNeedPoll = xMBPortEventPost( EV_FRAME_SENT );
            /* Disable transmitter. This prevents another transmit buffer
             * empty interrupt. */
            vMBPortSerialEnable( TRUE, FALSE );//使能接收，禁止发送
            eSndState = STATE_TX_IDLE;  //发送器状态为空闲状态
        }
        break;
    }

    return xNeedPoll;
}

//功能 : 此函数描述了发生超时中断时应处理的事务，供超时中断调用。
//描述 : 通过判读接收状态eRcvState来决定要处理的事务，思想上有点像摩尔类型的FSM的输出逻辑。
//       若中断发生于STATE_RX_INIT，则向系统发送事件EV_READY（Startup finished）；
//       若中断发生于STATE_RX_RCV，则向系统发送事件EV_FRAME_RECEIVED（Frame received）；
//       若中断发生于STATE_RX_ERROR，则跳出，不执行。在每个执行分支结束后，均关闭超时定时器，并将eRcvState转为STATE_RX_IDLE。
//       当然，这儿不像FSM的输出逻辑。

//功能    从机接收完成一帧数据后，接收状态机eRcvState为STATE_RX_RCV;
//     上报“接收到报文”事件(EV_FRAME_RECEIVED)
//     禁止3.5T定时器，设置接收状态机eRcvState状态为STATE_RX_IDLE空闲
BOOL xMBRTUTimerT35Expired( void )
{
    BOOL            xNeedPoll = FALSE;
	
#ifdef	DEBUG
 	printf("xMBRTUTimerT35Expired()\r\n");	
#endif
	  
    switch ( eRcvState )
    {
        /* Timer t35 expired. Startup phase is finished. */
	    //上报modbus协议栈的事件状态给poll函数，EV_READY:初始化完成事件
    case STATE_RX_INIT:
        xNeedPoll = xMBPortEventPost( EV_READY );
        break;

        /* A frame was received and t35 expired. Notify the listener that
         * a new frame was received. */
    case STATE_RX_RCV:                          //一帧数据接收完成
        xNeedPoll = xMBPortEventPost( EV_FRAME_RECEIVED );   //上报协议栈事件，接收到一帧完整的数据
		    printf("post fra_rcvd\r\n");
        break;

        /* An error occured while receiving the frame. */
    case STATE_RX_ERROR:
        break;

        /* Function called in an illegal state. */
    default:
        assert( ( eRcvState == STATE_RX_INIT ) ||
                ( eRcvState == STATE_RX_RCV ) || ( eRcvState == STATE_RX_ERROR ) );
    }

    vMBPortTimersDisable(  );   //当接收到一帧数据后，禁止3.5T定时器，直到接收下一帧数据开始，开始计时
    eRcvState = STATE_RX_IDLE;  //处理完一帧数据，接收器状态为空闲
    //至此，从机接收到一帧完整的报文，存储在ucRTUBuf[MB_SER_PDU_SIZE_MAX]全局变量中，定时器禁止，接收机状态为空闲。

    return xNeedPoll;
}
