/***********************************************************************

       ͨѶ������-��STM32�У�ʹ��3��I2C��I2cId����Ϊ1,2,3ʱ��ʵ��
��ģ��ʵ��Ӳ����Ӧ����أ�����Ϊʾ��!!
***********************************************************************/
#include "I2cMng.h"
#include "IoCtrl.h"
#include <string.h>

/******************************************************************************
		                        �ص�����-Ӳ�����ʵ��
******************************************************************************/

//---------------------------Ӳ��ID��I2cId�Ĳ��ұ�ʵ��------------------------
const unsigned char I2cMng_cbHwId2I2cId[] = {0, 1, 2};

//---------------------------�ҽӵĵײ�Ӳ��ָ��ʵ��-----------------------------
void* const I2cMng_cbHw[] = {I2C1, I2C2, I2C3};

//---------------------------------�ж�����ʵ��-------------------------------- 
//�±�ΪI2cId
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

//STM32ר������ʱ�Ӻ���,��I2cDev_STM32��ʵ��
extern void I2cDev_CfgClk(I2C_TypeDef *pHw, unsigned int Baudrate); //������

//-----------------------�ײ�Ӳ���������½ӿ�ʵ��-------------------------------
void I2cMng_cbUpdateCfg(unsigned char I2cId)
{
  I2cDev_CfgClk((I2C_TypeDef *)I2cMng_cbHw[I2cId], 
                I2cMng.I2c[I2cId].Info.BuadK * 1000);//������ʱ��
}

/*****************************************************************************
		                     USART1Ӳ���ж�
*****************************************************************************/

//----------------------------- I2C1�¼�Ӳ���ж�-------------------------------
void I2C1_EV_IRQHandler(void)
{
  I2cDev_IRQ(I2cMng_pGetI2cDev(0));
}
void I2C1_ER_IRQHandler(void)
{
  I2cDev_IRQ(I2cMng_pGetI2cDev(0));
}

//-----------------------------I2C2�¼�Ӳ���ж�-------------------------------
void I2C2_EV_IRQHandler(void)
{
  I2cDev_IRQ(I2cMng_pGetI2cDev(1));
}

void I2C2_ER_IRQHandler(void)
{
  I2cDev_IRQ(I2cMng_pGetI2cDev(1));
}

//----------------------------I2C3�¼�Ӳ���ж�-------------------------------
void I2C3_EV_IRQHandler(void)
{
  I2cDev_IRQ(I2cMng_pGetI2cDev(2));
}

void I2C3_ER_IRQHandler(void)
{
  I2cDev_IRQ(I2cMng_pGetI2cDev(2));
}


