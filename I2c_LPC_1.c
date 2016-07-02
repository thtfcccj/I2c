/*********************************************************************
						LPC֮I2C.h�ӿ����Ӧ�õ�ʵ��
��ʵ����Ե���I2c�ӿڵ�LPCоƬ
**********************************************************************/

/**********************************************************************
                        �ڲ�����
**********************************************************************/

//#define FAST_MODE_PLUS   //I2C����ģʽ֧��
#define _I2C_BAUD     100000   //I2CͨѶ����,HZΪ��λ

/**********************************************************************
                        ���ʵ��
**********************************************************************/
#include  "I2C.h"
#include "IOCtrl.h"

#include  <string.h>

struct _I2c I2c;

//-----------------------------LPC2103��I2C��ʼ������-----------------------
//��ʼ���ܽŶ����
void I2C_Init(void)
{
  //1.��ʼ����Դ���Ƽ�ʱ��
  //LPC_SYSCON->PDRUNCFG &= ~XXX_PD;    //��ֹ����
  LPC_SYSCON->PRESETCTRL |= I2C_RST_N; //���и�λ
  LPC_SYSCON->SYSAHBCLKCTRL |= I2C_ACC;//����ʱ��

  //2.��ʼ���ܽ����� (IOCtrl.h��ʵ��)
  CfgI2C_IO();
  
  //3.��ʼ������
  LPC_I2C->CONCLR = I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC | I2CONCLR_I2ENC;

  /*--- Reset registers ---*/
  #if FAST_MODE_PLUS
    LPC_I2C->SCLL   = I2SCLL_HS_SCLL;
    LPC_I2C->SCLH   = I2SCLH_HS_SCLH;
  #else
    LPC_I2C->SCLL   = I2SCLL_SCLL;
    LPC_I2C->SCLH   = I2SCLH_SCLH;
  #endif

  //4.��ʼ��������ģ��
  memset(&I2c,0,sizeof(struct _I2c));
  I2cDev_Init(&I2c.I2cDev[0],LPC_I2C,SystemAHBFrequency, _I2C_BAUD);

  //5.��������ж�
  NVIC_EnableIRQ(I2C_IRQn); //�����ж�
  LPC_I2C->CONSET = I2CONSET_I2EN;
}

//-----------------------------ϵͳI2C0�жϴ������ʵ��-----------------------
//ֻ�ڳ�ʼ���ж���ʹ��
void I2C0_IRQHandler(void)
{
  I2cDev_IRQ(&I2c.I2cDev[0]);
}
