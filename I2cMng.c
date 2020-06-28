/***********************************************************************

                   I2c管理器实现
此模块负责管理底层硬件并提供软件接口
此模块应用无相关
***********************************************************************/

#include "I2cMng.h"
#include "Eeprom.h"
#include "InfoBase.h"
#include "IoCtrl.h"
#include <string.h>

/***********************************************************************
		                     变量与常明
***********************************************************************/

//默认I2c的信息结构
static struct _I2cInfo _DefaultInfo = {
  100,        //波特率，K为单位
};

struct _I2cMng I2cMng; //实例

/***********************************************************************
		                      相关函数实现
***********************************************************************/

//---------------------------I2c初始化函数---------------------------------
static void  _I2cInit(signed char IsInited,
                         unsigned char I2cId)
{
  struct _I2c *pI2c = I2cMng_pGetI2c(I2cId);
  
  if(!IsInited){//初始化为默认
    memcpy(&pI2c->Info, &_DefaultInfo, sizeof(struct _I2cInfo));
    if(!IsInited){//未初始化时才保存
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
  I2cMng_cbUpdateCfg(I2cId);  //底层硬件配置初始化
  I2cDev_Init(I2cMng_pGetI2cDev(I2cId), //底层硬件通讯初始化
              I2cMng_cbHw[I2cId],
              SYS_MHZ,
              I2cMng.I2c[I2cId].Info.BuadK * 1000); 
  I2cMng_cbCfgInt[I2cId]();  //最后配置中断
}

//--------------------------------初始化函数---------------------------------
//形参0x55专用于表示装载默认设置
void  I2cMng_Init(signed char IsInited)
{
  CfgI2c(); //IO口
  //各底层分别初始化
  for(unsigned char I2cId = 0; I2cId < I2C_COUNT; I2cId++){
    _I2cInit(IsInited, I2cId);
  }
}

//---------------------保存I2cInfo成员至EEPROM函数----------------------------
//调用此函数前请更新数据,并将内存数据保存至EEPROM
void I2cMng_SaveInfo(unsigned char I2cId,
                       unsigned char StructOffset, //struct_offset()得到偏移
                       unsigned char Len)
{
  Eeprom_Wr(I2cMng_GetInfoBase(I2cId) + StructOffset,
              (unsigned char*)(I2cMng_pGetInfo(I2cId)) + StructOffset,Len);
}
//---------------------从EEPROM中读取参数函数----------------------------
//此函数用于读取修改系统参数后会引起内存问题的参数,不修改内存值
unsigned char I2cMng_GetInfoFromEeprom(unsigned char I2cId,
                                         unsigned char StructOffset) //struct_offset()得到偏移
{
  unsigned char Data;
  Eeprom_Rd(I2cMng_GetInfoBase(I2cId) + StructOffset, &Data, 1);
  return Data;
}

//---------------------保存I2cInfo成员至EEPROM函数----------------------------
//此函数直接修改EEPROM,不修改内存值，与I2cMng_GetInfoFromEeprom()配套使用
void I2cMng_SaveInfoToEeprom(unsigned char Data,
                               unsigned char I2cId,
                               unsigned char StructOffset) //struct_offset()得到偏移
{
  Eeprom_Wr(I2cMng_GetInfoBase(I2cId) + StructOffset, &Data, 1);
}

//---------------------------底层硬件指针到I2cId-----------------------------
//负值表示未找到
signed char I2cMng_Dev2I2cId(const struct _I2cDev *pDev)
{
  for(unsigned char I2cId = 0; I2cId < I2C_COUNT; I2cId++){
    if(pDev == I2cMng_pGetI2cDev(I2cId)) return I2cId;
  }
  return -1;//未找到
}


