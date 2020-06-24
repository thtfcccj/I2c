/*********************************************************************

		ͨ��I2c�����豸��������ӿ�-��STM32�е�ʵ��

*********************************************************************/

#include "I2cDev.h"
#include "CMSIS.h"

#include <string.h>

#ifndef I2C_DEV_MHZ//I2Cʱ��Ƶ��MHZ,��Χ2~50
  #define I2C_DEV_MHZ    25    
#endif

#ifdef SUPPORT_I2C_DEV_CB      //֧�ֻص�ʱ�ջص���ֹ����
  static void _CallBackNull(I2cDev_t *pI2cDev,unsigned char NotifyFlag){}
#endif


//---------------------------------����ʱ��ʵ��--------------------------
//STM32ר��
void I2cDev_CfgClk(I2C_TypeDef *pHw,
                   unsigned int Baudrate)   //�趨�Ĳ�����
{
  //��ֹͣI2c���߲�������
  pHw->CR1 = I2C_CR1_SWRST;  
  //ʱ�ӣ������ݻ������ж����¼��ж�(�ݲ���DMA,�����ж��в�ѯ��ʽ)
  pHw->CR2 = I2C_DEV_MHZ | I2C_CR2_ITEVTEN |  I2C_CR2_ITBUFEN; 
  //���㲢����I2c������,50%ռ�ձ�,Sm mode or SMBus
  unsigned char Baud = (unsigned long)(I2C_DEV_MHZ / 2)  / Baudrate;
  pHw->CCR = Baud;
  //�������������˲�
  Baud /= 100;
  pHw->TRISE = Baud;//������,�ػ�ģʽ�õõ�
  pHw->FLTR = Baud;//���������˲������ٸ�ʱ��������Ч
  pHw->CR1 = I2C_CR1_PE; //������Χ(����I2C)
}

//------------------------------I2c�豸��ʼ������----------------------
//�˺�����ʼ���豸�ṹ�������ҽӵ�I2cӲ����ʼ��
//ע���˺������������ó�I2cӲ������������֣������ţ��жϣ�ʱ�ӵ�
void I2cDev_Init(I2cDev_t *pI2cDev,        //δ��ʼ�����豸ָ��
                 void *pI2cHw,             //�ҽӵ�I2cӲ��
                 unsigned int Mck,         //��ʱ��
                 unsigned int Baudrate)   //�趨�Ĳ�����
{
  //��ʼ���ṹ
  memset(pI2cDev,0,sizeof(I2cDev_t));
  pI2cDev->pI2cHw = pI2cHw; //Ӳ���ҽ�
  #ifdef SUPPORT_I2C_DEV_CB      //֧�ֻص�ʱ�ջص���ֹ����
    pI2cDev->CallBack = _CallBackNull;
  #endif
  
  I2C_TypeDef *pHw = (I2C_TypeDef*)pI2cHw;
  I2cDev_CfgClk(pHw, Baudrate);
}

//-----------------------------I2c��д������������-------------------------
////�����Ƿ�ɹ� 0�ɹ�,��0ʧ��
//��״̬�����������ڼ����ʱ�������ش�ʶ
signed char I2cDev_ReStart(I2cDev_t *pI2cDev, //�豸ָ��
                           I2cData_t *pData)
{
  enum eI2cStatus_t eStatus = pI2cDev->eStatus;
  I2C_TypeDef *pHw = (I2C_TypeDef *)(pI2cDev->pI2cHw);

  //I2c���й����н�ֹ����
  if((eStatus > eI2cIdie) && (eStatus < eI2cDone)) return -1;

  pHw->CR1 = I2C_CR1_SWRST;  //��ֹͣ
  //��ʼ���ṹ��أ�
  pI2cDev->Index = 0;
  pI2cDev->ErrTimer = pData->Flag & I2C_WAIT_OV_MASK;
  pI2cDev->pData = pData;
  pI2cDev->eStatus = eI2cRdy; //׼��״̬
  //����I2c��ʼ����
  pHw->OAR1 = pData->SlvAdr << 1; //7bitģʽ����ַд��  
  pHw->CR1 |=  I2C_CR1_PE| I2C_CR1_START; //������ʼ����(�ɹ�����ж�)

  return 0;
}

//-----------------------------I2cǿ�Ƹ�λ����-------------------------
//ֹͣ��ǿ��I2c��λ
void I2cDev_Reset(I2cDev_t *pI2cDev)
{
  I2C_TypeDef *pHw = (I2C_TypeDef *)(pI2cDev->pI2cHw);
  pHw->CR1 = I2C_CR1_SWRST;
  pI2cDev->eStatus = eI2cIdie;
}

//-----------------------------I2c�жϴ������-------------------------
//���˺��������жϴ��������
void I2cDev_IRQ(I2cDev_t *pI2cDev)
{
  I2C_TypeDef *pHw = (I2C_TypeDef *)(pI2cDev->pI2cHw);
  I2cData_t *pData = pI2cDev->pData;
  unsigned short HwSR1 = pHw->SR1;//��Ӳ��״̬λ
  
  enum eI2cStatus_t eStatus = pI2cDev->eStatus;//��״̬��
  unsigned char Index = pI2cDev->Index;    //��������ݷ��͸���
  pI2cDev->ErrTimer = pData->Flag & I2C_WAIT_OV_MASK;//��ʱ����λ

  //========================����������ӻ���ַ=================================
  //��������ʼλ,����������ַ
  if(HwSR1 & I2C_SR1_SB){
    volatile unsigned short HwSR2 = pHw->SR2;//��SR2���I2C_SR1_SB��־
    //д������ַ,��д��Ϊ�ȶ�I2C
    if(eStatus == eI2cRdy){
      if((pData->CmdSize == 0) && (pData->Flag & I2C_CMD_RD)){//��ָ���ʱ(��Ӳ��֧��)
        pHw->DR = (pData->SlvAdr << 1) | 1;//���λд1��ʾ��������״̬,д��������
        pHw->CR1 |= I2C_CR1_ACK;//��������ʱԤ��Ӧ������
      }
      else{//��ָ�����ָ���ʱ
        pHw->DR = (pData->SlvAdr << 1) | 0; //���λд0��ʾ��������״̬,д��������
        eStatus = eI2cWrCmd;
      }
    }
    //������ʱ���������ߺ󣬷���������ַ
    else if((eStatus == eI2cRd) && (Index == 0)){
      pHw->DR = (pData->SlvAdr << 1) | 0x01;//���λд1��ʾ��������״̬,д��������
      pHw->CR1 |= I2C_CR1_ACK;//��������ʱԤ��Ӧ������
    }
    else eStatus = eI2cErrState;   //״̬������
    goto _IrqEnd; //ֱ�ӽ���
  }
  //�ѷ���������ַ�ͱ�־�����յ�Ӧ��
  if(HwSR1 & I2C_SR1_ADDR){
    if((eStatus == eI2cWrCmd) && (Index == 0)){
      HwSR1 = I2C_SR1_TXE; //������ת�����͵�һ����   
    }
    else if((eStatus == eI2cRd) && (Index == 0))
      HwSR1 = I2C_SR1_RXNE; //������ת�����յ�һ����
    else{
      eStatus = eI2cErrState;   //״̬������
      goto _IrqEnd;          //ֱ�ӽ���
    }
    //����ɼ���
  }  
  //============================���������������===========================
  //�ѷ�����һ������
  if(HwSR1 & I2C_SR1_TXE){
    if(eStatus == eI2cWrCmd){   //д����׶�
      if(Index < pData->CmdSize){
        pHw->DR = *(pData->pData + Index); //д��������
        pI2cDev->Index++;
      }
      else{//��������:
        pI2cDev->Index = 0;
        if(pData->Flag & I2C_CMD_RD){//�����ݣ�������������
          pHw->CR1 |=  I2C_CR1_START; //��������
          eStatus = eI2cRd;
        }
        else eStatus = eI2cWr;//д���ݣ���ʼ���͵�һ������
      }
    }
    if(eStatus == eI2cWr){    //д���ݽ׶�,(��else)
      if(Index < pData->DataSize){//δ���
        pHw->DR = *(pData->pData + Index); //д��������
        pI2cDev->Index++;
      }
      else{ //����д���,��������
        pHw->CR1 = I2C_CR1_STOP;
        eStatus = eI2cDone;   //д���
        #ifdef SUPPORT_I2C_DEV_CB      //���ͨ��
          if(pData->Flag & I2C_CB_FINAL) pI2cDev->CallBack(pI2cDev, I2C_CB_FINAL);
        #endif
      }
    }
    else eStatus = eI2cErrState;   //״̬������
    goto _IrqEnd; //ֱ�ӽ���
  }
  //===================================��������=================================
  if(HwSR1 & I2C_SR1_RXNE){//�����ѽ��յ���
    if(eStatus == eI2cRd){//�����ݹ�����
      if(Index < pData->DataSize){//������,û�н������
        *(pData->pData + Index) = pHw->DR;//��ȡ���ݲ����ж�
        pI2cDev->Index++;
        #ifdef SUPPORT_I2C_DEV_CB      //��������ͨ��
          if(pData->Flag & I2C_CB_RCV) pI2cDev->CallBack(pI2cDev, I2C_CB_RCV);
        #endif
        //׼���������һ������
        if(pI2cDev->Index >= (pData->DataSize - 1)){
          pHw->CR1 &= ~I2C_CR1_ACK;//ȡ��Ӧ���־�����ôӻ���������
          pHw->CR1 = I2C_CR1_STOP; //����ֹͣλ
          eStatus = eI2cRdLast;
        }
      }
      else eStatus = eI2cErrState;//���ȳ���
    }
    else if(eStatus == eI2cRd){//�����һ�����ݣ���ʱ�����ѽ���
      if(Index == (pData->DataSize - 1)){
        *(pData->pData + Index) = pHw->DR;//��ȡ���ݲ����ж�
        eStatus = eI2cDone;   //�����
        #ifdef SUPPORT_I2C_DEV_CB      //���ͨ��
          if(pData->Flag & I2C_CB_FINAL) pI2cDev->CallBack(pI2cDev, I2C_CB_FINAL);
        #endif
      }
      else eStatus = eI2cErrState;//λ�ò���
    }
    else eStatus = eI2cErrState;//״̬������
    goto _IrqEnd; //ֱ�ӽ���
  }
  eStatus = eI2cErrState;//״̬������

_IrqEnd: //��������
  //��I2C״̬����,��ǿ�н���I2c����
  if(eStatus == eI2cErrState){
    pHw->CR1 = I2C_CR1_SWRST;
    #ifdef SUPPORT_I2C_DEV_CB      //״̬����ͨ��
      if(pData->Flag & I2C_CB_ERR) pI2cDev->CallBack(pI2cDev, I2C_CB_ERR);
    #endif
  }
  pI2cDev->eStatus = eStatus;

  // �жϴ������
}

//--------------------------------I2c������--------------------------
//�����������ϵͳTICK��
void I2cDev_Task(I2cDev_t *pI2cDev)
{
  I2C_TypeDef *pHw = (I2C_TypeDef *)(pI2cDev->pI2cHw);
  
  if(pI2cDev->ErrTimer) pI2cDev->ErrTimer--;
  if(!pI2cDev->ErrTimer){//��ʱʱ�䵽��ǿ�н���I2c����
    pHw->CR1 = I2C_CR1_SWRST;
    pI2cDev->eStatus = eI2cErrOV; //�ȴ���ʱ����
    #ifdef SUPPORT_I2C_DEV_CB      //��ʱ�ص�
      if(pI2cDev->pData->Flag & I2C_CB_OV) pI2cDev->CallBack(pI2cDev, I2C_CB_OV);
    #endif
  }
}

//--------------------------------I2c�Ƿ��������--------------------------
//���ط�0��ʾ����������������ȴ�
signed char I2cDev_IsEnd(const I2cDev_t *pI2cDev)
{
  if((pI2cDev->eStatus > eI2cIdie) &&
     (pI2cDev->eStatus < eI2cDone)) return 0;
  return 1;
}


