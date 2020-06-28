/***********************************************************************

                   I2c������ʵ��
��ģ�鸺�����ײ�Ӳ�����ṩ����ӿ�
��ģ��Ӧ�������
***********************************************************************/

#include "I2cMng.h"
#include "Eeprom.h"
#include "InfoBase.h"
#include "IoCtrl.h"
#include <string.h>

/***********************************************************************
		                     �����볣��
***********************************************************************/

//Ĭ��I2c����Ϣ�ṹ
static struct _I2cInfo _DefaultInfo = {
  100,        //�����ʣ�KΪ��λ
};

struct _I2cMng I2cMng; //ʵ��

/***********************************************************************
		                      ��غ���ʵ��
***********************************************************************/

//---------------------------I2c��ʼ������---------------------------------
static void  _I2cInit(signed char IsInited,
                         unsigned char I2cId)
{
  struct _I2c *pI2c = I2cMng_pGetI2c(I2cId);
  
  if(!IsInited){//��ʼ��ΪĬ��
    memcpy(&pI2c->Info, &_DefaultInfo, sizeof(struct _I2cInfo));
    if(!IsInited){//δ��ʼ��ʱ�ű���
      Eeprom_Wr(I2cMng_GetInfoBase(I2cId),
                &pI2c->Info,
                sizeof(struct _I2cInfo));
    }
  }
  else{
    Eeprom_Rd(I2cMng_GetInfoBase(I2cId),
              &pI2c->Info,
              sizeof(struct _I2cInfo));
  }
  I2cMng_cbUpdateCfg(I2cId);  //�ײ�Ӳ�����ó�ʼ��
  I2cDev_Init(I2cMng_pGetI2cDev(I2cId), //�ײ�Ӳ��ͨѶ��ʼ��
              I2cMng_cbHw[I2cId],
              SYS_MHZ,
              I2cMng.I2c[I2cId].Info.BuadK * 1000); 
  I2cMng_cbCfgInt[I2cId]();  //��������ж�
}

//--------------------------------��ʼ������---------------------------------
//�β�0x55ר���ڱ�ʾװ��Ĭ������
void  I2cMng_Init(signed char IsInited)
{
  CfgI2c(); //IO��
  //���ײ�ֱ��ʼ��
  for(unsigned char I2cId = 0; I2cId < I2C_COUNT; I2cId++){
    _I2cInit(IsInited, I2cId);
  }
}

//---------------------����I2cInfo��Ա��EEPROM����----------------------------
//���ô˺���ǰ���������,�����ڴ����ݱ�����EEPROM
void I2cMng_SaveInfo(unsigned char I2cId,
                       unsigned char StructOffset, //struct_offset()�õ�ƫ��
                       unsigned char Len)
{
  Eeprom_Wr(I2cMng_GetInfoBase(I2cId) + StructOffset,
              (unsigned char*)(I2cMng_pGetInfo(I2cId)) + StructOffset,Len);
}
//---------------------��EEPROM�ж�ȡ��������----------------------------
//�˺������ڶ�ȡ�޸�ϵͳ������������ڴ�����Ĳ���,���޸��ڴ�ֵ
unsigned char I2cMng_GetInfoFromEeprom(unsigned char I2cId,
                                         unsigned char StructOffset) //struct_offset()�õ�ƫ��
{
  unsigned char Data;
  Eeprom_Rd(I2cMng_GetInfoBase(I2cId) + StructOffset, &Data, 1);
  return Data;
}

//---------------------����I2cInfo��Ա��EEPROM����----------------------------
//�˺���ֱ���޸�EEPROM,���޸��ڴ�ֵ����I2cMng_GetInfoFromEeprom()����ʹ��
void I2cMng_SaveInfoToEeprom(unsigned char Data,
                               unsigned char I2cId,
                               unsigned char StructOffset) //struct_offset()�õ�ƫ��
{
  Eeprom_Wr(I2cMng_GetInfoBase(I2cId) + StructOffset, &Data, 1);
}

//---------------------------�ײ�Ӳ��ָ�뵽I2cId-----------------------------
//��ֵ��ʾδ�ҵ�
signed char I2cMng_Dev2I2cId(const struct _I2cDev *pDev)
{
  for(unsigned char I2cId = 0; I2cId < I2C_COUNT; I2cId++){
    if(pDev == I2cMng_pGetI2cDev(I2cId)) return I2cId;
  }
  return -1;//δ�ҵ�
}


