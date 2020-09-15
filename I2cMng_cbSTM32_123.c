/***********************************************************************

       通讯管理器-在STM32中，使用3个I2C且I2cId排序为1,2,3时的实现
此模块实现硬件与应用相关，这里为示例!!
***********************************************************************/
#include "I2cMng.h"
#include "IoCtrl.h"
#include <string.h>

/******************************************************************************
		                        回调函数-硬件相关实现
******************************************************************************/

//---------------------------硬件ID到I2cId的查找表实现------------------------
const unsigned char I2cMng_cbHwId2I2cId[] = {0, 1, 2};

//---------------------------挂接的底层硬件指针实现-----------------------------
void* const I2cMng_cbHw[] = {I2C1, I2C2, I2C3};

//---------------------------------中断配置实现-------------------------------- 
//下标为I2cId
static void _CfgInt1(void)
{
  NVIC_Config(1,1, I2C1_EV_IRQn);
}
static void _CfgInt2(void)
{
  NVIC_Config(1,2,I2C2_EV_IRQn);
}
static void _CfgInt3(void)
{
  NVIC_Config(1,3,I2C3_EV_IRQn);
}

const I2cCfgInt_t I2cMng_cbCfgInt[] = {
  _CfgInt1,
  _CfgInt2,
  _CfgInt3,  
};

//STM32专用配置时钟函数,在I2cDev_STM32里实现
extern void I2cDev_CfgClk(I2C_TypeDef *pHw, unsigned int Baudrate); //波特率

//-----------------------底层硬件参数更新接口实现-------------------------------
void I2cMng_cbUpdateCfg(unsigned char I2cId)
{
  I2cDev_CfgClk((I2C_TypeDef *)I2cMng_cbHw[I2cId], 
                I2cMng.I2c[I2cId].Info.BuadK * 1000);//重配置时钟
}

/*****************************************************************************
		                     USART1硬件中断
*****************************************************************************/

//----------------------------- I2C1事件硬件中断-------------------------------
void I2C1_EV_IRQHandler(void)
{
  I2cDev_IRQ(I2cMng_pGetI2cDev(0));
}
void I2C1_ER_IRQHandler(void)
{
  I2cDev_IRQ(I2cMng_pGetI2cDev(0));
}

//-----------------------------I2C2事件硬件中断-------------------------------
void I2C2_EV_IRQHandler(void)
{
  I2cDev_IRQ(I2cMng_pGetI2cDev(1));
}

void I2C2_ER_IRQHandler(void)
{
  I2cDev_IRQ(I2cMng_pGetI2cDev(1));
}

//----------------------------I2C3事件硬件中断-------------------------------
void I2C3_EV_IRQHandler(void)
{
  I2cDev_IRQ(I2cMng_pGetI2cDev(2));
}

void I2C3_ER_IRQHandler(void)
{
  I2cDev_IRQ(I2cMng_pGetI2cDev(2));
}


