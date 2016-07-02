/*********************************************************************
						LPC之I2C.h接口针对应用的实现
此实现针对单个I2c接口的LPC芯片
**********************************************************************/

/**********************************************************************
                        内部配置
**********************************************************************/

//#define FAST_MODE_PLUS   //I2C快速模式支持
#define _I2C_BAUD     100000   //I2C通讯速率,HZ为单位

/**********************************************************************
                        相关实现
**********************************************************************/
#include  "I2C.h"
#include "IOCtrl.h"

#include  <string.h>

struct _I2c I2c;

//-----------------------------LPC2103的I2C初始化程序-----------------------
//初始化管脚定义等
void I2C_Init(void)
{
  //1.初始化电源控制及时钟
  //LPC_SYSCON->PDRUNCFG &= ~XXX_PD;    //禁止掉电
  LPC_SYSCON->PRESETCTRL |= I2C_RST_N; //集中复位
  LPC_SYSCON->SYSAHBCLKCTRL |= I2C_ACC;//开启时钟

  //2.初始化管脚配置 (IOCtrl.h里实现)
  CfgI2C_IO();
  
  //3.初始化其它
  LPC_I2C->CONCLR = I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC | I2CONCLR_I2ENC;

  /*--- Reset registers ---*/
  #if FAST_MODE_PLUS
    LPC_I2C->SCLL   = I2SCLL_HS_SCLL;
    LPC_I2C->SCLH   = I2SCLH_HS_SCLH;
  #else
    LPC_I2C->SCLL   = I2SCLL_SCLL;
    LPC_I2C->SCLH   = I2SCLH_SCLH;
  #endif

  //4.初始化变量及模块
  memset(&I2c,0,sizeof(struct _I2c));
  I2cDev_Init(&I2c.I2cDev[0],LPC_I2C,SystemAHBFrequency, _I2C_BAUD);

  //5.最后允许中断
  NVIC_EnableIRQ(I2C_IRQn); //开启中断
  LPC_I2C->CONSET = I2CONSET_I2EN;
}

//-----------------------------系统I2C0中断处理程序实现-----------------------
//只在初始化中断中使用
void I2C0_IRQHandler(void)
{
  I2cDev_IRQ(&I2c.I2cDev[0]);
}
