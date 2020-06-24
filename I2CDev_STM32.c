/*********************************************************************

		通用I2c主机设备驱动程序接口-在STM32中的实现

*********************************************************************/

#include "I2cDev.h"
#include "CMSIS.h"

#include <string.h>

#ifndef I2C_DEV_MHZ//I2C时钟频率MHZ,范围2~50
  #define I2C_DEV_MHZ    25    
#endif

#ifdef SUPPORT_I2C_DEV_CB      //支持回调时空回调防止出错
  static void _CallBackNull(I2cDev_t *pI2cDev,unsigned char NotifyFlag){}
#endif


//---------------------------------配置时钟实现--------------------------
//STM32专用
void I2cDev_CfgClk(I2C_TypeDef *pHw,
                   unsigned int Baudrate)   //设定的波特率
{
  //先停止I2c总线才能配置
  pHw->CR1 = I2C_CR1_SWRST;  
  //时钟，打开数据缓冲区中断与事件中断(暂不用DMA,故障中断有查询方式)
  pHw->CR2 = I2C_DEV_MHZ | I2C_CR2_ITEVTEN |  I2C_CR2_ITBUFEN; 
  //计算并配置I2c波特率,50%占空比,Sm mode or SMBus
  unsigned char Baud = (unsigned long)(I2C_DEV_MHZ / 2)  / Baudrate;
  pHw->CCR = Baud;
  //配置上升沿与滤波
  Baud /= 100;
  pHw->TRISE = Baud;//上升沿,回环模式用得到
  pHw->FLTR = Baud;//开启数据滤波，多少个时间周期有效
  pHw->CR1 = I2C_CR1_PE; //允许外围(开启I2C)
}

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
  #ifdef SUPPORT_I2C_DEV_CB      //支持回调时空回调防止出错
    pI2cDev->CallBack = _CallBackNull;
  #endif
  
  I2C_TypeDef *pHw = (I2C_TypeDef*)pI2cHw;
  I2cDev_CfgClk(pHw, Baudrate);
}

//-----------------------------I2c读写数据启动函数-------------------------
////返回是否成功 0成功,非0失败
//当状态机正在运行期间访问时，将返回错识
signed char I2cDev_ReStart(I2cDev_t *pI2cDev, //设备指针
                           I2cData_t *pData)
{
  enum eI2cStatus_t eStatus = pI2cDev->eStatus;
  I2C_TypeDef *pHw = (I2C_TypeDef *)(pI2cDev->pI2cHw);

  //I2c进行过程中禁止访问
  if((eStatus > eI2cIdie) && (eStatus < eI2cDone)) return -1;

  pHw->CR1 = I2C_CR1_SWRST;  //先停止
  //初始化结构相关：
  pI2cDev->Index = 0;
  pI2cDev->ErrTimer = pData->Flag & I2C_WAIT_OV_MASK;
  pI2cDev->pData = pData;
  pI2cDev->eStatus = eI2cRdy; //准备状态
  //启动I2c开始工作
  pHw->OAR1 = pData->SlvAdr << 1; //7bit模式，地址写入  
  pHw->CR1 |=  I2C_CR1_PE| I2C_CR1_START; //允许并开始启动(成功后进中断)

  return 0;
}

//-----------------------------I2c强制复位函数-------------------------
//停止并强制I2c复位
void I2cDev_Reset(I2cDev_t *pI2cDev)
{
  I2C_TypeDef *pHw = (I2C_TypeDef *)(pI2cDev->pI2cHw);
  pHw->CR1 = I2C_CR1_SWRST;
  pI2cDev->eStatus = eI2cIdie;
}

//-----------------------------I2c中断处理程序-------------------------
//将此函数放入中断处理程序中
void I2cDev_IRQ(I2cDev_t *pI2cDev)
{
  I2C_TypeDef *pHw = (I2C_TypeDef *)(pI2cDev->pI2cHw);
  I2cData_t *pData = pI2cDev->pData;
  unsigned short HwSR1 = pHw->SR1;//读硬件状态位
  
  enum eI2cStatus_t eStatus = pI2cDev->eStatus;//读状态机
  unsigned char Index = pI2cDev->Index;    //命令或数据发送个数
  pI2cDev->ErrTimer = pData->Flag & I2C_WAIT_OV_MASK;//定时器复位

  //========================发送启动或从机地址=================================
  //己发送起始位,发送器件地址
  if(HwSR1 & I2C_SR1_SB){
    volatile unsigned short HwSR2 = pHw->SR2;//读SR2清除I2C_SR1_SB标志
    //写器件地址,读写均为先读I2C
    if(eStatus == eI2cRdy){
      if((pData->CmdSize == 0) && (pData->Flag & I2C_CMD_RD)){//无指令读时(此硬件支持)
        pHw->DR = (pData->SlvAdr << 1) | 1;//最低位写1表示主机接收状态,写启动发送
        pHw->CR1 |= I2C_CR1_ACK;//接收数据时预置应答允许
      }
      else{//有指令，或无指令发送时
        pHw->DR = (pData->SlvAdr << 1) | 0; //最低位写0表示主机发送状态,写启动发送
        eStatus = eI2cWrCmd;
      }
    }
    //读数据时重启动总线后，发送器件地址
    else if((eStatus == eI2cRd) && (Index == 0)){
      pHw->DR = (pData->SlvAdr << 1) | 0x01;//最低位写1表示主机接收状态,写启动发送
      pHw->CR1 |= I2C_CR1_ACK;//接收数据时预置应答允许
    }
    else eStatus = eI2cErrState;   //状态机错误
    goto _IrqEnd; //直接结束
  }
  //已发送器件地址和标志并接收到应答
  if(HwSR1 & I2C_SR1_ADDR){
    if((eStatus == eI2cWrCmd) && (Index == 0)){
      HwSR1 = I2C_SR1_TXE; //此条件转至发送第一个数   
    }
    else if((eStatus == eI2cRd) && (Index == 0))
      HwSR1 = I2C_SR1_RXNE; //此条件转至接收第一个数
    else{
      eStatus = eI2cErrState;   //状态机错误
      goto _IrqEnd;          //直接结束
    }
    //这里可继续
  }  
  //============================发送命令或发送数据===========================
  //已发送完一组数据
  if(HwSR1 & I2C_SR1_TXE){
    if(eStatus == eI2cWrCmd){   //写命令阶段
      if(Index < pData->CmdSize){
        pHw->DR = *(pData->pData + Index); //写启动发送
        pI2cDev->Index++;
      }
      else{//命令发送完成:
        pI2cDev->Index = 0;
        if(pData->Flag & I2C_CMD_RD){//读数据，重新启动总线
          pHw->CR1 |=  I2C_CR1_START; //重启总线
          eStatus = eI2cRd;
        }
        else eStatus = eI2cWr;//写数据，开始发送第一个数据
      }
    }
    if(eStatus == eI2cWr){    //写数据阶段,(无else)
      if(Index < pData->DataSize){//未完成
        pHw->DR = *(pData->pData + Index); //写启动发送
        pI2cDev->Index++;
      }
      else{ //数据写完成,结束总线
        pHw->CR1 = I2C_CR1_STOP;
        eStatus = eI2cDone;   //写完成
        #ifdef SUPPORT_I2C_DEV_CB      //完成通报
          if(pData->Flag & I2C_CB_FINAL) pI2cDev->CallBack(pI2cDev, I2C_CB_FINAL);
        #endif
      }
    }
    else eStatus = eI2cErrState;   //状态机错误
    goto _IrqEnd; //直接结束
  }
  //===================================接收数据=================================
  if(HwSR1 & I2C_SR1_RXNE){//数据已接收到了
    if(eStatus == eI2cRd){//读数据过程中
      if(Index < pData->DataSize){//接收中,没有接收完成
        *(pData->pData + Index) = pHw->DR;//读取数据并清中断
        pI2cDev->Index++;
        #ifdef SUPPORT_I2C_DEV_CB      //接收数据通报
          if(pData->Flag & I2C_CB_RCV) pI2cDev->CallBack(pI2cDev, I2C_CB_RCV);
        #endif
        //准备接收最后一个数了
        if(pI2cDev->Index >= (pData->DataSize - 1)){
          pHw->CR1 &= ~I2C_CR1_ACK;//取消应答标志，以让从机不发数了
          pHw->CR1 = I2C_CR1_STOP; //发送停止位
          eStatus = eI2cRdLast;
        }
      }
      else eStatus = eI2cErrState;//长度超限
    }
    else if(eStatus == eI2cRd){//读最后一个数据，此时总线已结束
      if(Index == (pData->DataSize - 1)){
        *(pData->pData + Index) = pHw->DR;//读取数据并清中断
        eStatus = eI2cDone;   //读完成
        #ifdef SUPPORT_I2C_DEV_CB      //完成通报
          if(pData->Flag & I2C_CB_FINAL) pI2cDev->CallBack(pI2cDev, I2C_CB_FINAL);
        #endif
      }
      else eStatus = eI2cErrState;//位置不对
    }
    else eStatus = eI2cErrState;//状态机错误
    goto _IrqEnd; //直接结束
  }
  eStatus = eI2cErrState;//状态机错误

_IrqEnd: //结束处理
  //若I2C状态出错,则强行结束I2c总线
  if(eStatus == eI2cErrState){
    pHw->CR1 = I2C_CR1_SWRST;
    #ifdef SUPPORT_I2C_DEV_CB      //状态错误通报
      if(pData->Flag & I2C_CB_ERR) pI2cDev->CallBack(pI2cDev, I2C_CB_ERR);
    #endif
  }
  pI2cDev->eStatus = eStatus;

  // 中断处理结束
}

//--------------------------------I2c任务函数--------------------------
//将此任务放入系统TICK中
void I2cDev_Task(I2cDev_t *pI2cDev)
{
  I2C_TypeDef *pHw = (I2C_TypeDef *)(pI2cDev->pI2cHw);
  
  if(pI2cDev->ErrTimer) pI2cDev->ErrTimer--;
  if(!pI2cDev->ErrTimer){//定时时间到，强行结束I2c总线
    pHw->CR1 = I2C_CR1_SWRST;
    pI2cDev->eStatus = eI2cErrOV; //等待超时错误
    #ifdef SUPPORT_I2C_DEV_CB      //超时回调
      if(pI2cDev->pData->Flag & I2C_CB_OV) pI2cDev->CallBack(pI2cDev, I2C_CB_OV);
    #endif
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


