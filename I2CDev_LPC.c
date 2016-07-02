/*********************************************************************

		通用I2c主机设备驱动程序接口-在LPC中的实现

*********************************************************************/

#include "I2cDev.h"
#include "LPC12xx.h"
#include "LPC12xxbit.h"

#include <string.h>

//------------------------------I2c设备初始化函数----------------------
//此函数将始化设备结构，并将挂接的I2c硬件初始化
//注：此函数不负责配置除I2c硬件外的其它部分，如引脚，中断，时钟等
void I2cDev_Init(I2cDev_t *pI2cDev,        //未初始化的设备指针
                 void *pI2cHw,             //挂接的I2c硬件
                 unsigned int Mck,         //主时钟
                 unsigned int Baudrate)   //设定的波特率
{
  //初始化结构
  memset(pI2cDev,0,sizeof(I2cDev_t));
  pI2cDev->pI2cHw = pI2cHw; //硬件挂接
  LPC_I2C_TypeDef *pHw = (LPC_I2C_TypeDef*)pI2cHw;

  //计算并配置I2c波特率,配置I2c占空比为50%
  Baudrate = (Mck / Baudrate) >> 1;
  //先强行停止I2c总线
  pHw->CONCLR = LPC_I2C_EN | LPC_I2C_SI | LPC_I2C_STA | LPC_I2C_AA;
  pHw->SCLH = Baudrate;
  pHw->SCLL = Baudrate;
}

//-----------------------------I2c读写数据启动函数-------------------------
////返回是否成功 0成功,非0失败
//当状态机正在运行期间访问时，将返回错识
signed char I2cDev_ReStart(I2cDev_t *pI2cDev, //设备指针
                           I2cData_t *pData)
{
  enum eI2cStatus_t eStatus = pI2cDev->eStatus;
  LPC_I2C_TypeDef *pHw = (LPC_I2C_TypeDef *)(pI2cDev->pI2cHw);

  //I2c进行过程中禁止访问
  if((eStatus > eI2cIdie) && (eStatus < eI2cDone)) return -1;

  pHw->CONCLR = LPC_I2C_SI | LPC_I2C_STA | LPC_I2C_AA;  //先停止
  //初始化结构相关：
  pI2cDev->Index = 0;
  pI2cDev->ErrTimer = pData->Flag & I2C_WAIT_OV_MASK;
  pI2cDev->pData = pData;
  pI2cDev->eStatus = eI2cRdy; //准备状态
  //启动I2c开始工作:
  pHw->CONSET = LPC_I2C_EN;//先使能I2c
  pHw->CONSET = LPC_I2C_STA | LPC_I2C_AA | LPC_I2C_EN;

  return 0;
}

//-----------------------------I2c强制复位函数-------------------------
//停止并强制I2c复位
void I2cDev_Reset(I2cDev_t *pI2cDev)
{


}

//-----------------------------I2c中断处理程序-------------------------
//将此函数放入中断处理程序中
void I2cDev_IRQ(I2cDev_t *pI2cDev)
{
  LPC_I2C_TypeDef *pHw = (LPC_I2C_TypeDef *)(pI2cDev->pI2cHw);
  I2cData_t *pData = pI2cDev->pData;
  unsigned int StatusHw = pHw->STAT;//读硬件状态位
  enum eI2cStatus_t eStatus = pI2cDev->eStatus;//读状态机

  unsigned char Index;

  pI2cDev->ErrTimer = pData->Flag & I2C_WAIT_OV_MASK;//定时器复位



  switch(StatusHw){
  //======================发送器件地址=========================
  case  0x08://己发送起始条件,发送器件地址
    if(eStatus == eI2cRdy){   //写器件地址,读写均为先读I2C
      pHw->DAT = pData->SlvAdr << 1;
      pHw->CONCLR = LPC_I2C_SI + LPC_I2C_STA;
      eStatus = eI2cWrCmd;
    }
    else eStatus = eI2cErrState;   //状态机错误
    break;
  case  0x10://读数据时重启动总线后，发送器件地址
    if((!pI2cDev->Index) && (eStatus == eI2cRd)){
      pHw->DAT = (pData->SlvAdr << 1) | 0x01;//I2c的读标志
      pHw->CONCLR = LPC_I2C_SI + LPC_I2C_STA;
    }
    else eStatus = eI2cErrState;   //状态机错误
    break;
  //=======================发送命令或发送数据======================
  case 0x18://已发送器件地址和标志并接收到应答
  case 0x28://已发送器件地址标志,子地址或数据，并已接收应答
    Index = pI2cDev->Index;    //命令或数据发送个数
    if(eStatus == eI2cWrCmd){   //写命令阶段
      if(Index < pData->CmdSize){
        pHw->DAT = *(pData->pData + Index);
        pHw->CONCLR = LPC_I2C_SI + LPC_I2C_STA;
        pI2cDev->Index++;
        break;
      }
      else{//命令发送完成:
        pI2cDev->Index = Index = 0;
        if(pData->Flag & I2C_CMD_RD){//读数据，重新启动总线
          pHw->CONSET = LPC_I2C_STA;
          pHw->CONCLR = LPC_I2C_STO + LPC_I2C_SI;
          eStatus = eI2cRd;
          break;
        }
        else eStatus = eI2cWr;//开始发送第一个数据
      }
    }
    if(eStatus == eI2cWr){    //写数据阶段
      if(Index < pData->DataSize){
        pHw->DAT = *(pData->pData + Index);
        pHw->CONCLR = LPC_I2C_SI + LPC_I2C_STA;
        pI2cDev->Index++;
        break;
      }
      else{ //数据写完成,结束总线
        pHw->CONSET = LPC_I2C_STO;
        pHw->CONCLR = LPC_I2C_SI + LPC_I2C_STA;
        eStatus = eI2cDone;   //写完成
      }
      break;
    }
    else eStatus = eI2cErrState;   //状态机错误
    break;
  //===========================接收数据=========================
  case 0x50://数据已接收到且有应答信号
    Index = pI2cDev->Index;    //命令或数据发送个数
    if((eStatus == eI2cRd) && (Index < pData->DataSize)){//接收中,没有接收完成
      *(pData->pData + Index) = pHw->DAT;//读取数据
      pI2cDev->Index++;
      //(此后与0x40处理同)
    }
    else{
      eStatus = eI2cErrState;   //状态机错误
      break;
    }
    //发送完器件地址，并已接收到应答信号
  case 0x40:
    if((pI2cDev->Index + 1) >= pData->DataSize){//接收最后一字节，接收后不产生应答
      pHw->CONCLR = LPC_I2C_SI + LPC_I2C_STA + LPC_I2C_AA;
      eStatus = eI2cRdLast;
    }
    else{ //还有多个字节:  继续接收数据并发送应答信号
      pHw->CONSET = LPC_I2C_AA;
      pHw->CONCLR = LPC_I2C_SI + LPC_I2C_STA;
    }
    break;
  case  0x58: //数据已接收到,但无应答信号:表示已读到最后一个数据,结束总线
    if(eStatus == eI2cRdLast){//最后一字节时
      *(pData->pData + pI2cDev->Index) = pHw->DAT;//读取数据
      pI2cDev->Index++;
      pHw->CONSET = LPC_I2C_STO;
      pHw->CONCLR = LPC_I2C_SI + LPC_I2C_STA;
      eStatus = eI2cDone;   //读完成
    }
    else eStatus = eI2cErrState; //状态机错误
    break;
  //================================故障处理===========================
  //器件地址位发送后未回应
  //case  0x20 uI2cStat_tmp = 0x38; //(与0x38处理同)
  //数据位发送后未回应
  //case  0x30: uI2cStat_tmp = 0x38; //(与0x38处理同)
  //接收模式时地址已发出,但无回应
  //case  0x48: uI2cStat_tmp = 0x38; //(与0x38处理同)
  //发送数据时地址已发出,但无回应
  //case  0x38:
  default:		//其它状态均认为是器件出错
    eStatus = eI2cErrState;   //状态机错误
    break;
  }

  //若I2C状态出错,则强行结束I2c总线
  if(eStatus == eI2cErrState){
    pHw->CONSET = LPC_I2C_STO;
    pHw->CONCLR = LPC_I2C_SI + LPC_I2C_STA;
  }
  pI2cDev->eStatus = eStatus;

  //;              // 中断处理结束
}

//--------------------------------I2c任务函数--------------------------
//将此任务放入系统TICK中
void I2cDev_Task(I2cDev_t *pI2cDev)
{
  LPC_I2C_TypeDef *pHw = (LPC_I2C_TypeDef *)(pI2cDev->pI2cHw);
  
  if(pI2cDev->ErrTimer) pI2cDev->ErrTimer--;
  if(!pI2cDev->ErrTimer){//定时时间到，强行结束I2c总线
    pHw->CONSET = LPC_I2C_STO;
    pHw->CONCLR = LPC_I2C_EN | LPC_I2C_SI + LPC_I2C_STA;
    pI2cDev->eStatus = eI2cErrOV; //等待超时
  }
}

//--------------------------------I2c是否结束函数--------------------------
//返回非0表示结束，否则需继续等待
signed char I2cDev_IsEnd(const I2cDev_t *pI2cDev)
{
  if((pI2cDev->eStatus > eI2cIdie) &&
     (pI2cDev->eStatus < eI2cDone)) return 0;
  return 1;

}


