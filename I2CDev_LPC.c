/*********************************************************************

		ͨ��I2c�����豸��������ӿ�-��LPC�е�ʵ��

*********************************************************************/

#include "I2cDev.h"
#include "LPC12xx.h"
#include "LPC12xxbit.h"

#include <string.h>

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
  LPC_I2C_TypeDef *pHw = (LPC_I2C_TypeDef*)pI2cHw;

  //���㲢����I2c������,����I2cռ�ձ�Ϊ50%
  Baudrate = (Mck / Baudrate) >> 1;
  //��ǿ��ֹͣI2c����
  pHw->CONCLR = LPC_I2C_EN | LPC_I2C_SI | LPC_I2C_STA | LPC_I2C_AA;
  pHw->SCLH = Baudrate;
  pHw->SCLL = Baudrate;
}

//-----------------------------I2c��д������������-------------------------
////�����Ƿ�ɹ� 0�ɹ�,��0ʧ��
//��״̬�����������ڼ����ʱ�������ش�ʶ
signed char I2cDev_ReStart(I2cDev_t *pI2cDev, //�豸ָ��
                           I2cData_t *pData)
{
  enum eI2cStatus_t eStatus = pI2cDev->eStatus;
  LPC_I2C_TypeDef *pHw = (LPC_I2C_TypeDef *)(pI2cDev->pI2cHw);

  //I2c���й����н�ֹ����
  if((eStatus > eI2cIdie) && (eStatus < eI2cDone)) return -1;

  pHw->CONCLR = LPC_I2C_SI | LPC_I2C_STA | LPC_I2C_AA;  //��ֹͣ
  //��ʼ���ṹ��أ�
  pI2cDev->Index = 0;
  pI2cDev->ErrTimer = pData->Flag & I2C_WAIT_OV_MASK;
  pI2cDev->pData = pData;
  pI2cDev->eStatus = eI2cRdy; //׼��״̬
  //����I2c��ʼ����:
  pHw->CONSET = LPC_I2C_EN;//��ʹ��I2c
  pHw->CONSET = LPC_I2C_STA | LPC_I2C_AA | LPC_I2C_EN;

  return 0;
}

//-----------------------------I2cǿ�Ƹ�λ����-------------------------
//ֹͣ��ǿ��I2c��λ
void I2cDev_Reset(I2cDev_t *pI2cDev)
{


}

//-----------------------------I2c�жϴ������-------------------------
//���˺��������жϴ��������
void I2cDev_IRQ(I2cDev_t *pI2cDev)
{
  LPC_I2C_TypeDef *pHw = (LPC_I2C_TypeDef *)(pI2cDev->pI2cHw);
  I2cData_t *pData = pI2cDev->pData;
  unsigned int StatusHw = pHw->STAT;//��Ӳ��״̬λ
  enum eI2cStatus_t eStatus = pI2cDev->eStatus;//��״̬��

  unsigned char Index;

  pI2cDev->ErrTimer = pData->Flag & I2C_WAIT_OV_MASK;//��ʱ����λ



  switch(StatusHw){
  //======================����������ַ=========================
  case  0x08://��������ʼ����,����������ַ
    if(eStatus == eI2cRdy){   //д������ַ,��д��Ϊ�ȶ�I2C
      pHw->DAT = pData->SlvAdr << 1;
      pHw->CONCLR = LPC_I2C_SI + LPC_I2C_STA;
      eStatus = eI2cWrCmd;
    }
    else eStatus = eI2cErrState;   //״̬������
    break;
  case  0x10://������ʱ���������ߺ󣬷���������ַ
    if((!pI2cDev->Index) && (eStatus == eI2cRd)){
      pHw->DAT = (pData->SlvAdr << 1) | 0x01;//I2c�Ķ���־
      pHw->CONCLR = LPC_I2C_SI + LPC_I2C_STA;
    }
    else eStatus = eI2cErrState;   //״̬������
    break;
  //=======================���������������======================
  case 0x18://�ѷ���������ַ�ͱ�־�����յ�Ӧ��
  case 0x28://�ѷ���������ַ��־,�ӵ�ַ�����ݣ����ѽ���Ӧ��
    Index = pI2cDev->Index;    //��������ݷ��͸���
    if(eStatus == eI2cWrCmd){   //д����׶�
      if(Index < pData->CmdSize){
        pHw->DAT = *(pData->pData + Index);
        pHw->CONCLR = LPC_I2C_SI + LPC_I2C_STA;
        pI2cDev->Index++;
        break;
      }
      else{//��������:
        pI2cDev->Index = Index = 0;
        if(pData->Flag & I2C_CMD_RD){//�����ݣ�������������
          pHw->CONSET = LPC_I2C_STA;
          pHw->CONCLR = LPC_I2C_STO + LPC_I2C_SI;
          eStatus = eI2cRd;
          break;
        }
        else eStatus = eI2cWr;//��ʼ���͵�һ������
      }
    }
    if(eStatus == eI2cWr){    //д���ݽ׶�
      if(Index < pData->DataSize){
        pHw->DAT = *(pData->pData + Index);
        pHw->CONCLR = LPC_I2C_SI + LPC_I2C_STA;
        pI2cDev->Index++;
        break;
      }
      else{ //����д���,��������
        pHw->CONSET = LPC_I2C_STO;
        pHw->CONCLR = LPC_I2C_SI + LPC_I2C_STA;
        eStatus = eI2cDone;   //д���
      }
      break;
    }
    else eStatus = eI2cErrState;   //״̬������
    break;
  //===========================��������=========================
  case 0x50://�����ѽ��յ�����Ӧ���ź�
    Index = pI2cDev->Index;    //��������ݷ��͸���
    if((eStatus == eI2cRd) && (Index < pData->DataSize)){//������,û�н������
      *(pData->pData + Index) = pHw->DAT;//��ȡ����
      pI2cDev->Index++;
      //(�˺���0x40����ͬ)
    }
    else{
      eStatus = eI2cErrState;   //״̬������
      break;
    }
    //������������ַ�����ѽ��յ�Ӧ���ź�
  case 0x40:
    if((pI2cDev->Index + 1) >= pData->DataSize){//�������һ�ֽڣ����պ󲻲���Ӧ��
      pHw->CONCLR = LPC_I2C_SI + LPC_I2C_STA + LPC_I2C_AA;
      eStatus = eI2cRdLast;
    }
    else{ //���ж���ֽ�:  �����������ݲ�����Ӧ���ź�
      pHw->CONSET = LPC_I2C_AA;
      pHw->CONCLR = LPC_I2C_SI + LPC_I2C_STA;
    }
    break;
  case  0x58: //�����ѽ��յ�,����Ӧ���ź�:��ʾ�Ѷ������һ������,��������
    if(eStatus == eI2cRdLast){//���һ�ֽ�ʱ
      *(pData->pData + pI2cDev->Index) = pHw->DAT;//��ȡ����
      pI2cDev->Index++;
      pHw->CONSET = LPC_I2C_STO;
      pHw->CONCLR = LPC_I2C_SI + LPC_I2C_STA;
      eStatus = eI2cDone;   //�����
    }
    else eStatus = eI2cErrState; //״̬������
    break;
  //================================���ϴ���===========================
  //������ַλ���ͺ�δ��Ӧ
  //case  0x20 uI2cStat_tmp = 0x38; //(��0x38����ͬ)
  //����λ���ͺ�δ��Ӧ
  //case  0x30: uI2cStat_tmp = 0x38; //(��0x38����ͬ)
  //����ģʽʱ��ַ�ѷ���,���޻�Ӧ
  //case  0x48: uI2cStat_tmp = 0x38; //(��0x38����ͬ)
  //��������ʱ��ַ�ѷ���,���޻�Ӧ
  //case  0x38:
  default:		//����״̬����Ϊ����������
    eStatus = eI2cErrState;   //״̬������
    break;
  }

  //��I2C״̬����,��ǿ�н���I2c����
  if(eStatus == eI2cErrState){
    pHw->CONSET = LPC_I2C_STO;
    pHw->CONCLR = LPC_I2C_SI + LPC_I2C_STA;
  }
  pI2cDev->eStatus = eStatus;

  //;              // �жϴ������
}

//--------------------------------I2c������--------------------------
//�����������ϵͳTICK��
void I2cDev_Task(I2cDev_t *pI2cDev)
{
  LPC_I2C_TypeDef *pHw = (LPC_I2C_TypeDef *)(pI2cDev->pI2cHw);
  
  if(pI2cDev->ErrTimer) pI2cDev->ErrTimer--;
  if(!pI2cDev->ErrTimer){//��ʱʱ�䵽��ǿ�н���I2c����
    pHw->CONSET = LPC_I2C_STO;
    pHw->CONCLR = LPC_I2C_EN | LPC_I2C_SI + LPC_I2C_STA;
    pI2cDev->eStatus = eI2cErrOV; //�ȴ���ʱ
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


