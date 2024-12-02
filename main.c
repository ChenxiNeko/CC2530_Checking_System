#include <iocc2530.h>
#include "clock2.h"
#include "hal_mcu.h"
#include "hal_assert.h"
#include "hal_rf.h"
#include "basic_rf.h"
#include "led.h"
#include "clock.h"
#include "delay.h"
#include "touch.h"
#include "uart1.h"
#include "oled.h"
#include "key.h"
#include "string.h"
#include "vibration.h"
#include <stdio.h>

#define RF_CHANNEL            12                                // 2.4 GHz RF channel
#define PAN_ID                0x0638
#define SEND_ADDR             0x2530
#define RECV_ADDR             0x2520
#define COMMAND               0x10
#define COMMANDV             0x11

#define NODE_TYPE             0                                 //0:���սڵ㣬��0�����ͽڵ�

void io_init(void)
{
  P1SEL &= ~0x04;
  P1DIR &= ~0x04;
}

static basicRfCfg_t basicRfConfig;
int ledstatus = 0;

void rfSendData(void)
{
  u8 D2_Status = OFF;
  char Vibration_status = 0;
  uint8 pTxData[] = {COMMAND};
  uint8 pTxData2[] = {COMMANDV};
  xtal_init();                                                  //ϵͳʱ�ӳ�ʼ��     
  led_init();                                                   //LED��ʼ��
  touch_init();                                                 //������������ʼ��
  uart1_init(0x00,0x00);                                        //���ڳ�ʼ��
  vibration_init();                                             //�𶯴�������ʼ��
  uart1_send_string("Touch Experiment:\r\n");
  unsigned int send_count = 0;
  
  while(TRUE)
  {
    if(get_touch_status() == 1){                                //��⵽����
      D2_Status = 1 - D2_Status;
      D2 = D2_Status;                                           //����D2��
      uart1_send_string("touch detected!\r\n");                 //���ڴ�ӡ��ʾ��Ϣ
      basicRfSendPacket(RECV_ADDR, pTxData, sizeof pTxData); 
      
    }

    if(get_vibration_status() == 1){                            //��⵽��
      if(Vibration_status == 0){                                //�𶯴�����״̬�����ı�
        uart1_send_string("Vibration!\r\n");                     //���ڴ�ӡ��ʾ��Ϣ
        basicRfSendPacket(RECV_ADDR, pTxData2, sizeof pTxData2); 
        Vibration_status = 1;                                   //�����𶯴�����״̬
        while(TRUE) {
          D2 = !D2;
          delay_ms(200);
        }
      }
      send_count = 0;                                                //������0
    }
    else{                                                       //û�м�⵽��
      send_count ++;                                                 //��������
      if(send_count > 50000)                                         //�ж��Ƿ�ֹͣ��
      {
        send_count =  0;                                             //����0
        if(Vibration_status == 1){                              //�𶯴�����״̬�����ı�
          uart1_send_string("No Vibration!\r\n");                //���ڴ�ӡ��ʾ��Ϣ
          Vibration_status = 0;                                 //�����𶯴�����״̬       
        } 
      }    
    }
    halMcuWaitMs(50);
  }
}


void rfRecvData(void) {
  xtal_init();                                                  //ϵͳʱ�ӳ�ʼ�� 
  OLED_Init();                                                  //OLED��ʼ��
  uart1_init(0x00,0x00);                                        //���ڳ�ʼ��
  OLED_ShowCHineseString(0,0,0,8);
  delay_s(1);
  OLED_ShowCHineseString(0,2,8,7);
  OLED_ShowCHineseString(0,4,16,4);
  delay_s(2);

  OLED_Clear();
  OLED_ShowCHineseString(0,0,49,5);
  OLED_ShowCHineseString(0,2,54,5);
  OLED_ShowCHineseString(0,4,42,7);
  OLED_ShowString(0,6,"0",16);
  uint8 pRxData[64];
  unsigned int count = 0;
  int rlen;
    basicRfReceiveOn();           
    // Main loop
    while (TRUE) {
        while(!basicRfPacketIsReady());
    rlen = basicRfReceive(pRxData, sizeof pRxData, NULL);
    if(rlen > 0 && pRxData[0] == COMMAND) {
      count++;
      OLED_Clear();
      OLED_ShowCHineseString(0,0,49,5);
      OLED_ShowCHineseString(0,2,54,5);
      OLED_ShowCHineseString(0,4,42,7);
      int icount = count % 10;
      int tcount = (count % 100) / 10 ;
      int hcount = count / 100;
      if(hcount == 0){
        if(tcount == 0){
          unsigned char i1[2];
          sprintf((char *)i1,"%d",count);
          OLED_ShowString(0,6,i1,16);
        }
        else {
          unsigned char i2[3];
          sprintf((char *)i2,"%d%d",tcount,icount);
          OLED_ShowString(0,6,i2,16);
        }
        }
      else {
        unsigned char i3[4];
        sprintf((char *)i3, "%d%d%d", hcount, tcount, icount);
        OLED_ShowString(0, 6, i3, 16);
      }
    }
    
    if(rlen > 0 && pRxData[0] == COMMANDV) {
      OLED_Clear();
      OLED_ShowCHineseString(0,0,59,6);
      OLED_ShowCHineseString(0,2,59,6);
      OLED_ShowCHineseString(0,4,59,6);
      OLED_ShowCHineseString(0,6,59,6);
    }
}
}

void main(void)
{
    halMcuInit();
    io_init();       
    if (FAILED == halRfInit()) {
        HAL_ASSERT(FALSE);
    }

    // Config basicRF
    basicRfConfig.panId = PAN_ID;
    basicRfConfig.channel = RF_CHANNEL;
    basicRfConfig.ackRequest = TRUE;
#ifdef SECURITY_CCM
    basicRfConfig.securityKey = key;
#endif

    
    // Initialize BasicRF
#if NODE_TYPE
    basicRfConfig.myAddr = SEND_ADDR;
#else
    basicRfConfig.myAddr = RECV_ADDR; 
#endif
    
    if(basicRfInit(&basicRfConfig)==FAILED) {
      HAL_ASSERT(FALSE);
    }
#if NODE_TYPE
  rfSendData();
#else
  rfRecvData();
#endif
}
