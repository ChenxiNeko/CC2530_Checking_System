/*********************************************************************************************
* 文件：touch.c
* 作者：Lixm 2017.10.17
* 说明：触摸驱动代码
* 修改：
* 注释：
*********************************************************************************************/

/*********************************************************************************************
* 头文件
*********************************************************************************************/
#include "touch.h"

/*********************************************************************************************
* 名称：touch_init()
* 功能：触摸传感器初始化
* 参数：无
* 返回：无
* 修改：
* 注释：
*********************************************************************************************/
void touch_init(void)
{
  P0SEL &= ~0x01;                                               //配置管脚为通用IO模式
  P0DIR &= ~0x01;                                               //配置控制管脚为输入模式
}

/*********************************************************************************************
* 名称：unsigned char get_touch_status(void)
* 功能：获取触摸传感器状态
* 参数：无
* 返回：无
* 修改：
* 注释：
*********************************************************************************************/
unsigned char get_touch_status(void)
{
  static unsigned char touch_status = 0;
  if(P0_0){                                                      //检测io口电平
    if(touch_status == 0){
      touch_status = 1;
      return 1;
    }
    else
      return 0;
  }
  else{
    if(touch_status == 1){
      touch_status = 0;
      return 1;
    }
    else
      return 0;
  }
}