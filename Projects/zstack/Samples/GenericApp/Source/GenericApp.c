/**************************************************************************************************
  Filename:       GenericApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends "Hello World" to another "Generic"
  application every 15 seconds.  The application will also
  receive "Hello World" packets.

  The "Hello World" messages are sent/received as MSG type message.

  This applications doesn't have a profile, so it handles everything
  directly - itself.

  Key control:
    SW1:
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "GenericApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

/*********************************************************************
 * MACROS
 */
#define HAL_LED_OFF   1
#define HAL_LED_ON    0

// This is the max byte count per OTA message.
#if !defined( SERIAL_APP_TX_MAX )
#define SERIAL_APP_TX_MAX  80
#endif

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
  GENERICAPP_CLUSTERID
};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
  GENERICAPP_ENDPOINT,              //  int Endpoint;
  GENERICAPP_PROFID,                //  uint16 AppProfId[2];
  GENERICAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  GENERICAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  GENERICAPP_FLAGS,                 //  int   AppFlags:4;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList,  //  byte *pAppInClusterList;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // GenericApp_Init() is called.
devStates_t GenericApp_NwkState;


byte GenericApp_TransID;  // This is the unique message ID (counter)

afAddrType_t GenericApp_DstAddr;
byte RxBuf[SERIAL_APP_TX_MAX+1];
static uint8 SerialApp_TxLen;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void GenericApp_HandleKeys( byte shift, byte keys );
void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void GenericApp_SendTheMessage( void );
void GenericApp_SendTheMessage2(short cmd,char*data,char len);
void rxCB(uint8 port,uint8 event);
//进行校验
char getCrc(char*data,int len);
void uart_write_fmt(short cmd,char*data,char len);
//发送数据到终端
void GenericApp_SendTheMessageEnd(uint16 shortAddr, char*data,char len);

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void GenericApp_Init( byte task_id )
{
  halUARTCfg_t uartConfig;
  
  GenericApp_TaskID = task_id;
  GenericApp_NwkState = DEV_INIT;
  GenericApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  GenericApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  GenericApp_DstAddr.endPoint = 0;
  GenericApp_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &GenericApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( GenericApp_TaskID );
  
  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = HAL_UART_BR_115200;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 64;   // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = 128;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = 128;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = 6;    // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE; // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = rxCB;
  HalUARTOpen (0, &uartConfig); 
  
  // Update the display
#if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "GenericApp", HAL_LCD_LINE_1 );
#endif
    
  //ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
  //ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );
}

/*********************************************************************
 * @fn      GenericApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
UINT16 GenericApp_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          GenericApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;
          
        case KEY_CHANGE:
          GenericApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
            
          }
          break;

        case AF_INCOMING_MSG_CMD:
          GenericApp_MessageMSGCB( MSGpkt );
          //当是协调器时候
          //收到数据，原封不动的发送到串口
          if ( GenericApp_NwkState == DEV_ZB_COORD )
          {
            HalUARTWrite(0, MSGpkt->cmd.Data, MSGpkt->cmd.DataLength);
          }
          else if(GenericApp_NwkState==DEV_ROUTER||GenericApp_NwkState==DEV_END_DEVICE)
          {
            //当是终端或者路由器时候，处理数据
            //处理数据，如开灯，打开空调等。。
            HalUARTWrite(0, MSGpkt->cmd.Data, MSGpkt->cmd.DataLength);
            //uart_write_fmt(0, MSGpkt->cmd.Data, MSGpkt->cmd.DataLength);
          }
          
          
          break;

        case ZDO_STATE_CHANGE:
          GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( GenericApp_NwkState == DEV_ZB_COORD )
          {
            // Start sending "the" message in a regular interval.
            //osal_start_timerEx( GenericApp_TaskID, GENERICAPP_SEND_MSG_EVT,GENERICAPP_SEND_MSG_TIMEOUT );
            uart_write_fmt(0, "DEV_ZB_COORD\r\n", strlen("DEV_ZB_COORD\r\n"));
            char bf[100]={0};
            byte ieee[8]={0};
            uint16 netid= NLME_GetShortAddr();
            strcpy(ieee,NLME_GetExtAddr(),8);
            sprintf(bf,"%04X  %02X:%02X:%02X:%02X:%02X:%02X:%02X\r\n",netid,ieee[0],ieee[1],ieee[2],ieee[3],ieee[4],ieee[5],ieee[6],ieee[7]);
            //HalUARTWrite(0, bf, strlen(bf));
            uart_write_fmt(0, bf, strlen(bf));
          }
          else if(GenericApp_NwkState == DEV_ROUTER)
          {
            //当是路由器和终端时向协调器发送自己的短地址
            uart_write_fmt(0, "DEV_ROUTER\r\n", strlen("DEV_ROUTER\r\n"));
            char bf[100]={0};
            byte ieee[8]={0};
            uint16 netid= NLME_GetShortAddr();
            strcpy(ieee,NLME_GetExtAddr(),8);
            sprintf(bf,"%04X  %02X:%02X:%02X:%02X:%02X:%02X:%02X\r\n",netid,ieee[0],ieee[1],ieee[2],ieee[3],ieee[4],ieee[5],ieee[6],ieee[7]);
            //HalUARTWrite(0, bf, strlen(bf));
            uart_write_fmt(0, bf, strlen(bf));
            memset(bf,0,100);
            memcpy(bf,ieee,8);
            bf[8]=(netid&0xFF);
            bf[9]=((netid>>8)&0xFF);
            GenericApp_SendTheMessage2(1,bf,10);
            
          }
          else if(GenericApp_NwkState == DEV_END_DEVICE)
          {
            //当是路由器和终端时向协调器发送自己的短地址
            uart_write_fmt(0, "DEV_END_DEVICE\r\n", strlen("DEV_END_DEVICE\r\n"));
            char bf[100]={0};
            byte ieee[8]={0};
            uint16 netid= NLME_GetShortAddr();
            strcpy(ieee,NLME_GetExtAddr(),8);
            sprintf(bf,"%04X  %02X:%02X:%02X:%02X:%02X:%02X:%02X\r\n",netid,ieee[0],ieee[1],ieee[2],ieee[3],ieee[4],ieee[5],ieee[6],ieee[7]);
            //HalUARTWrite(0, bf, strlen(bf));
            uart_write_fmt(0, bf, strlen(bf));
            
            memset(bf,0,100);
            memcpy(bf,ieee,8);
            bf[8]=(netid&0xFF);
            bf[9]=((netid>>8)&0xFF);
            GenericApp_SendTheMessage2(1,bf,10);
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in GenericApp_Init()).
  if ( events & GENERICAPP_SEND_MSG_EVT )
  {
    // Send "the" message
    GenericApp_SendTheMessage();

    // Setup to send message again
    osal_start_timerEx( GenericApp_TaskID,
                        GENERICAPP_SEND_MSG_EVT,
                      GENERICAPP_SEND_MSG_TIMEOUT );

    // return unprocessed events
    return (events ^ GENERICAPP_SEND_MSG_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      GenericApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined(BLINK_LEDS)
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            GenericApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            GenericApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
          //  HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
 * @fn      GenericApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void GenericApp_HandleKeys( byte shift, byte keys )
{
  zAddrType_t dstAddr;
  
  // Shift is used to make each button/switch dual purpose.
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }

    if ( keys & HAL_KEY_SW_2 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate an End Device Bind Request for the mandatory endpoint
      dstAddr.addrMode = Addr16Bit;
      dstAddr.addr.shortAddr = 0x0000; // Coordinator
      ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(), 
                            GenericApp_epDesc.endPoint,
                            GENERICAPP_PROFID,
                            GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                            GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                            FALSE );
    }

    if ( keys & HAL_KEY_SW_3 )
    {
    }

    if ( keys & HAL_KEY_SW_4 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
      // Initiate a Match Description Request (Service Discovery)
      dstAddr.addrMode = AddrBroadcast;
      dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
      ZDP_MatchDescReq( &dstAddr, NWK_BROADCAST_SHORTADDR,
                        GENERICAPP_PROFID,
                        GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                        GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                        FALSE );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  byte data ;
     
  switch ( pkt->clusterId )
  {
    case GENERICAPP_CLUSTERID:
#if defined(ZDO_COORDINATOR)
      // "the" message
  #if defined( LCD_SUPPORTED )
        HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
  #elif defined( WIN32 )
        WPRINTSTR( pkt->cmd.Data );
  #endif

#else     
     data  = pkt->cmd.Data[0];    //osal_memcpy(&data, pkt->cmd.Data, 1);
     
     if(data == 0x10)
       HalLedSet(HAL_LED_1, HAL_LED_ON); 
     else if(data == 0x11)
       HalLedSet(HAL_LED_1, HAL_LED_OFF); 
     
     else if(data == 0x20)
       HalLedSet(HAL_LED_2, HAL_LED_ON); 
     else if(data == 0x21)
       HalLedSet(HAL_LED_2, HAL_LED_OFF); 
     
     else if(data == 0x30)
       HalLedSet(HAL_LED_3, HAL_LED_ON); 
     else if(data == 0x31)
       HalLedSet(HAL_LED_3, HAL_LED_OFF);         
      
#endif      
      break;
  }
}

/*********************************************************************
 * @fn      GenericApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
void GenericApp_SendTheMessage( void )
{
  afAddrType_t P2P_DstAddr;
  P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  P2P_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  P2P_DstAddr.addr.shortAddr = 0xFFFF; //终端短地址在LCD上有显示，此处换成终端短地址就可以点播了。

  if ( AF_DataRequest( &P2P_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       1,
                       RxBuf,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    // Error occurred in request to send.
  }
}

//发给协调器数据

void GenericApp_SendTheMessage2(short cmd, char*data,char len)
{
  char *temp=(char*)osal_mem_alloc(len+4);
  temp[0]=len;
  temp[1]=getCrc(data,len);
  temp[2]=(cmd&0xFF);
  temp[3]=((cmd>>8)&0xFF);
  memcpy(temp+4,data,len);
  
  afAddrType_t P2P_DstAddr;
  P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  P2P_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  P2P_DstAddr.addr.shortAddr = 0x00; //终端短地址在LCD上有显示，此处换成终端短地址就可以点播了。

  if ( AF_DataRequest( &P2P_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       len+4,
                       temp,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    // Error occurred in request to send.
  }
  osal_mem_free(temp);
}


void GenericApp_SendTheMessageEnd(uint16 shortAddr, char*data,char len)
{
  afAddrType_t P2P_DstAddr;
  P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  P2P_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  P2P_DstAddr.addr.shortAddr = shortAddr; //终端短地址在LCD上有显示，此处换成终端短地址就可以点播了。
  
  if ( AF_DataRequest( &P2P_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       len,
                       data,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    // Error occurred in request to send.
  }
  
}

static void rxCB(uint8 port,uint8 event)
{
  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) &&
#if SERIAL_APP_LOOPBACK
      (SerialApp_TxLen < SERIAL_APP_TX_MAX))
#else
      !SerialApp_TxLen)
#endif
  {
    SerialApp_TxLen = HalUARTRead(0, RxBuf, SERIAL_APP_TX_MAX);
   
    if (SerialApp_TxLen)
    {
      //当串口接收到数据时
      //HalUARTWrite(0, RxBuf, SerialApp_TxLen);
      //当是协调器时候
       
      if(GenericApp_NwkState == DEV_ZB_COORD)
      {
        unsigned char length=RxBuf[0];
        //当数据长度不符合要求时候默认为非法数据，不做任何处理
        if(SerialApp_TxLen!=length+4)
        {
          SerialApp_TxLen=0;
          return;
        }
        char crc=RxBuf[1];
        //当数据校验失败直接返回
        if(crc!=getCrc(RxBuf+4,length))
        {
          SerialApp_TxLen=0;
          return;
        }
        unsigned short cmd=(RxBuf[2]&0xFF)|((RxBuf[3]&0xFF)<<8);
        if(cmd==2)
        {
          //表示需要发往终端的控制包
          //前两个字节表示网络短地址
         uint16 address=(RxBuf[4]&0xFF)|((RxBuf[5]&0xFF)<<8);
         GenericApp_SendTheMessageEnd(address,RxBuf+6,length-2);
         uart_write_fmt(0, "send ok", 7);
        }
      }
      else
      {
        //当是终端和路由器时候
        uart_write_fmt(0,RxBuf,SerialApp_TxLen);
        GenericApp_SendTheMessage();
      }
       SerialApp_TxLen=0;
    }
  }
}


char getCrc(char*data,int len)
{
  int i=0;
  char result=0;
  for(i=0;i<len;i++)
  {
    result+=data[i];
  }
  return result;
}

void uart_write_fmt(short cmd,char*dt,char len)
{
  char* data=(char*)osal_mem_alloc(len+4);
  data[0]=len;
  data[1]=getCrc(dt,len);
  data[2]=(cmd&0xFF);
  data[3]=((cmd>>8)&0xFF);
  memcpy(data+4,dt,len);
  HalUARTWrite(0,data,len+4);
  osal_mem_free(data);
}
/*********************************************************************
*********************************************************************/
