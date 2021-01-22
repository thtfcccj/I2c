/*********************************************************************

		ͨ��I2c�����豸��������ӿ�-��STM32�е�ʵ��

*********************************************************************/

#include "I2cDev.h"
#include "CMSIS.h"

#include <string.h>

#ifndef I2C_DEV_MHZ//I2Cʱ��Ƶ��MHZ ȫ����APB1, Ϊ4��Ƶ
  #define I2C_DEV_MHZ   (SYS_MHZ / 4)   
  #define I2C_DEV_HZ    ((unsigned long)SYS_MHZ * 1000000 / 4)   
#endif

#ifdef SUPPORT_I2C_DEV_CB      //֧�ֻص�ʱ�ջص���ֹ����
  static void _CallBackNull(I2cDev_t *pI2cDev,unsigned char NotifyFlag){}
#endif


//---------------------------------����ʱ��ʵ��--------------------------
//STM32ר��
void I2cDev_CfgClk(I2C_TypeDef *pHw,
                   unsigned int Baudrate)   //�趨�Ĳ�����
{
  pHw->CR1 = I2C_CR1_SWRST; //��λ��ͬʱ�������������
  pHw->CR1 = 0;  
  //ʱ�ӣ������ݻ������ж����¼��ж�(�ݲ���DMA,�����ж��в�ѯ��ʽ)
  pHw->CR2 = I2C_DEV_MHZ | I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN |  I2C_CR2_ITERREN; 
  //���㲢����I2c������,50%ռ�ձ�,Sm mode or SMBus
  unsigned short Baud = (I2C_DEV_HZ / 2)  / Baudrate;
  if(Baud > 4095) Baud = 4095;//11λ
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
  //I2cDev_Reset(pI2cDev);//��λӲ��
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

  //��ʼ���ṹ��أ�
  pI2cDev->Index = 0;
  pI2cDev->ErrTimer = pData->Flag & I2C_WAIT_OV_MASK;
  pI2cDev->pData = pData;
  pI2cDev->eStatus = eI2cRdy; //׼��״̬
  //����I2c��ʼ����
  pHw->OAR1 = pData->SlvAdr << 1; //7bitģʽ����ַд��  
  pHw->CR1 |=  I2C_CR1_PE | I2C_CR1_START; //������ʼ����(�ɹ�����ж�)

  return 0;
}

//-----------------------------I2cǿ�Ƹ�λ����-------------------------
//ֹͣ��ǿ��I2c��λ
void I2cDev_Reset(I2cDev_t *pI2cDev)
{
  //��λӲ��
  I2C_TypeDef *pHw = (I2C_TypeDef *)(pI2cDev->pI2cHw);
  
  unsigned short CCR = pHw->CCR;//��λǰ��ס���������õ�
  pHw->CR1 = I2C_CR1_SWRST; //��λ��ͬʱ�������������
  pHw->CR1 = 0;  
  //ʱ�ӣ������ݻ������ж����¼��ж�(�ݲ���DMA,�����ж��в�ѯ��ʽ)
  pHw->CR2 = I2C_DEV_MHZ | I2C_CR2_ITEVTEN |  I2C_CR2_ITBUFEN | I2C_CR2_ITERREN; 
  pHw->CCR = CCR;
  CCR /= 100;
  pHw->TRISE = CCR;//������,�ػ�ģʽ�õõ�
  pHw->FLTR = CCR;//���������˲������ٸ�ʱ��������Ч
  pHw->CR1 = I2C_CR1_PE; //������Χ(����I2C)

  pI2cDev->eStatus = eI2cIdie;
}

unsigned long _BusErrCount = 0;  //���߹��ϼ���
unsigned long _StateErrCount = 0;//״̬���ϼ���
unsigned long _HwSR1ErrCount = 0;//״̬���ϼ���

//pI2cDev->eStatus == eI2cRdʱ�� pI2cDev->Index��������ʱ�ڲ�״̬��:
#define   _RD_WAIT_RESTART   0  //��0xff���У� �ȴ�������
#define   _RD_WAIT_ADR_END   0  //��0xfe����) �ȴ���ַ���ͽ���


static volatile unsigned short HwSR2;
//-----------------------------I2c�жϴ������-------------------------
//���˺��������жϴ��������
void I2cDev_IRQ(I2cDev_t *pI2cDev)
{

  I2C_TypeDef *pHw = (I2C_TypeDef *)(pI2cDev->pI2cHw);
  I2cData_t *pData = pI2cDev->pData;
  unsigned short HwSR1 = pHw->SR1;//��Ӳ��״̬λ
  HwSR2 = pHw->SR2;//��SR2���I2C_SR1_SB��־

  enum eI2cStatus_t eStatus = pI2cDev->eStatus;//��״̬��
  //��ɻ�δ׼���ý�ֹ����
  if((eStatus <= eI2cIdie) || (eStatus >= eI2cDone)) return ;
    
  unsigned char Index = pI2cDev->Index;    //��������ݷ��͸���
  pI2cDev->ErrTimer = pData->Flag & I2C_WAIT_OV_MASK;//��ʱ����λ

  //======================================���ϴ���=====================================
  //             ���ߴ���      �ٲö�ʧ        ��Ӧ��       ����/Ƿ��      PEC���մ���       ��ʱ����SCLΪ�ͳ�25ms)         
  if(HwSR1 & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF |  I2C_SR1_OVR | I2C_SR1_PECERR | I2C_SR1_TIMEOUT)){
    pHw->SR1 = 0;//ȫ�����(ֻ���⼸λ�ǿ�д��ֻ��Ϊ0��)
    eStatus = eI2cErrState;
    _BusErrCount++;
    goto _IrqEnd; //ֱ�ӽ���
  }
  
  //========================��������=========================================
  //EV5: ��������ʼλ,������������������ַ
  if((HwSR1 & I2C_SR1_SB) || (HwSR1 == 0)){//������ʱ����ȡ�ڶ���Ϊ0
    if(HwSR1 == 0) _HwSR1ErrCount++;
    //д������ַ,��д��Ϊ�ȶ�I2C
    if(eStatus == eI2cRdy){
      if((pData->CmdSize == 0) && (pData->Flag & I2C_CMD_RD)){//��ָ���ʱ(��Ӳ��֧��)
        eStatus = eI2cRd;//ֱ�Ӷ�
        pHw->DR = (pData->SlvAdr << 1) | 1;//���λд1��ʾ��������״̬,д��������
      }
      else{//��ָ�����ָ���ʱ
        eStatus = eI2cWrCmd;        
        pHw->DR = (pData->SlvAdr << 1) | 0; //���λд0��ʾ��������״̬,д��������
      }
    }
    //������ʱ���������ߺ󣬷���������ַ
    else if((eStatus == eI2cRd) && (Index == _RD_WAIT_RESTART)){
      pHw->DR = (pData->SlvAdr << 1) | 1;//���λд1��ʾ��������״̬,д��������
      pI2cDev->Index = _RD_WAIT_ADR_END;//ת�յ�ַ״̬
    }
    else eStatus = eI2cErrState;   //״̬������
    goto _IrqEnd; //ֱ�ӽ���
  }

  //========================����״̬ʱ����ַ�������===========================
  //��EV6(ADDR=1)
  if((HwSR1 & (I2C_SR1_ADDR | I2C_SR1_TXE)) == I2C_SR1_ADDR){
    if((eStatus == eI2cRd) && (Index == _RD_WAIT_ADR_END)){//��ʼ��ʱ
      pI2cDev->Index = 0;//�׸���
      pHw->CR1 |= I2C_CR1_ACK;//��������ʱԤ��Ӧ������
    }
    else  //״̬������
      eStatus = eI2cErrState;   //״̬������
    goto _IrqEnd;          //ֱ�ӽ���
  }
  
  //===================��������������ѷ�����(ֹͣ�ź��ѷ���)===================
  //��EV8-2: TxE=1, BTF = 1,
  if((HwSR1 & (I2C_SR1_TXE | I2C_SR1_BTF)) == (I2C_SR1_TXE | I2C_SR1_BTF)){
    if((eStatus == eI2cRd) && (Index == 0)){//��ʼ��ʱ
      pHw->CR1 &= ~ I2C_CR1_PE;//�ȹر�(���꣬STM32��֧���������źţ���������������ʧ��)
      pI2cDev->Index = _RD_WAIT_RESTART;//������
      pHw->CR1 |=  I2C_CR1_PE; //��������
      pHw->CR1 |=  I2C_CR1_START; //������ʼ����(�ɹ�����ж�)
    }
    else if((eStatus == eI2cWr) && (Index >= pData->DataSize)){//д���ʱ
      eStatus = eI2cDone;   //���
      #ifdef SUPPORT_I2C_DEV_CB      //���ͨ��
        if(pData->Flag & I2C_CB_FINAL) pI2cDev->CallBack(pI2cDev, I2C_CB_FINAL);
      #endif
    }
    else eStatus = eI2cErrState;   //״̬������
    goto _IrqEnd;          //ֱ�ӽ���
  }
  //============================���������������===========================
  //EV8: ���ͻ���Ϊ��(���ܻ�δ����)������ǰ׼����һ��
  if(HwSR1 & I2C_SR1_TXE){
    if(eStatus == eI2cRd) return;//�쳣����
    if(eStatus == eI2cWrCmd){   //д����׶�
      if(Index < pData->CmdSize){
        pHw->DR = *(pData->pCmd + Index); //д��������
        pI2cDev->Index++;
        goto _IrqEnd; //ֱ�ӽ���
      }
      else{//��������:
        pI2cDev->Index = 0;
        if(pData->Flag & I2C_CMD_RD){//��׼׼��������
          pHw->CR1 = I2C_CR1_STOP; //����ֹͣ�ź�
          eStatus = eI2cRd;//����д�����ˣ��û��������ݵ��������
          goto _IrqEnd; //ֱ�ӽ���
        }
        else{//д���ݣ���ʼ���͵�һ������
          eStatus = eI2cWr;
          Index = 0;
          //������д��
        }
      }
    }
    if(eStatus == eI2cWr){    //д���ݽ׶�,(��else)
      if(Index < pData->DataSize){//δ���
        pHw->DR = *(pData->pData + Index); //д��������
        pI2cDev->Index++;
      }
      else{ //����д���,��������
        pHw->CR1 = I2C_CR1_STOP; //����ֹͣ�ź�
      }
    }
    else eStatus = eI2cErrState;   //״̬������
    goto _IrqEnd; //ֱ�ӽ���
  }
  //===================================��������=================================
  //��EV7: 
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
          eStatus = eI2cRdLast;//�ȴ�����
        }
        else pHw->CR1 |= I2C_CR1_ACK;//����Ӧ��
      }
    }
    else if(eStatus == eI2cRdLast){//�������
      *(pData->pData + Index) = pHw->DR;//��ȡ���һ������
      pHw->CR1 = I2C_CR1_STOP; //ֹͣλֹͣͨѶ
      pHw->CR1 &= ~I2C_CR1_PE;//�ر�I2C
      eStatus = eI2cDone;   //���
      #ifdef SUPPORT_I2C_DEV_CB      //���ͨ��
        if(pData->Flag & I2C_CB_FINAL) pI2cDev->CallBack(pI2cDev, I2C_CB_FINAL);
      #endif
    }
    else eStatus = eI2cErrState;//״̬������
    goto _IrqEnd; //ֱ�ӽ���
  }
  _StateErrCount++;  
  eStatus = eI2cErrState;//״̬������

_IrqEnd: //��������
  //��I2C״̬����,��ǿ�н���I2c����
  if(eStatus == eI2cErrState){
    pHw->CR1 = I2C_CR1_STOP; //ֹͣλֹͣͨѶ
    pHw->CR1 &= ~I2C_CR1_PE;//�ر�I2C
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
  if(!pI2cDev->ErrTimer) return;
  pI2cDev->ErrTimer--;
  if(!pI2cDev->ErrTimer){//��ʱʱ�䵽��ǿ�н���I2c����
    I2cDev_Reset(pI2cDev);//�ȸ�λ
    pI2cDev->eStatus = eI2cErrOV; //�õȴ���ʱ����
    #ifdef SUPPORT_I2C_DEV_CB      //��ʱ�ص�
      if(pI2cDev->pData->Flag & I2C_CB_OV) pI2cDev->CallBack(pI2cDev, I2C_CB_OV);
    #endif
  }
  else if((pI2cDev->eStatus == eI2cRd) && //��ʼ��ʱ����ʧ������
          (pI2cDev->Index == _RD_WAIT_RESTART)){
    I2C_TypeDef *pHw = (I2C_TypeDef *)(pI2cDev->pI2cHw);
    pHw->CR1 &= ~ I2C_CR1_PE;//�ȹر�(���꣬STM32��֧���������ź�)
    pHw->CR1 |=  I2C_CR1_PE; //��������
    pHw->CR1 |=  I2C_CR1_START; //������ʼ����(�ɹ�����ж�)
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


