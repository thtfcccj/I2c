/*********************************************************************

			        ͨ��I2c�����豸��������ӿ�
 �˽ӿ�֧�ֶ�·I2c�豸����֧��I2c�������������豸��ͨѶ
*********************************************************************/
#ifndef __I2C_DEV_H
#define __I2C_DEV_H

/**********************************************************************
                          ���˵��
**********************************************************************/
//1.Ϊ����ʹ��,I2c���ݶ���Ϊר��,�ӻ���I2cSlave ����ΪI2cSlaveData_t
//2.Ϊ��ʡ���ݴ洢�ռ�,�ṹ�ڲ�����λ�ý������Ż�(����4�ֽڶ���)

/**********************************************************************
                        ��ؽṹ
**********************************************************************/

//I2c���ݶ���
typedef struct _I2cData{
  unsigned char SlvAdr;         //�ӻ���ַ,0-127
  unsigned char CmdSize;        //�����ִ�С,����Ϊ0
  unsigned char DataSize;       //�����ִ�С,>= 1;
  unsigned char Flag;           //��ر�־�֣�������

  const unsigned char *pCmd;    //������
  unsigned char *pData;         //����
}I2cData_t;

//���У���ر�־�ֶ���Ϊ��
#define   I2C_WAIT_OV_MASK  0x3f   //I2c�ȴ�ʱ�䣬0-64TICK
#define   I2C_CMD_RD        0x80      //����־
#define   I2C_CMD_WR        0x00      //д��־

//I2c���߹���״̬������
enum eI2cStatus_t
{
  //I2cֹͣ�ڼ�:
  eI2cIdie     = 0,     //���߿���
  //I2c�����ڼ�:
  eI2cRdy      = 1,     //I2c׼��״̬
  eI2cWrCmd    = 2,     //д����״̬
  eI2cWr       = 3,     //д����״̬
  eI2cRd       = 4,     //������״̬,�ȴ��ӻ�����,����ӻ�Ӧ��!
  eI2cRdLast   = 5,     //�����һ����,����ӻ�����Ӧ���źţ���׼������
  //I2c�����ڼ�:
  eI2cDone     = 6,     //��д�������,�ȴ����ݴ���
  eI2cErrState = 7,     //I2c״̬������
  eI2cErrOV    = 8      //I2c�ȴ���ʱ����
};

//I2c�豸����
typedef struct _I2cDev_t{
  void *pI2cHw;                            //�ҽӵ�I2cӲ���豸ָ��
  I2cData_t  *pData;                        //����ͨ����I2c�ӻ��豸������
  volatile enum eI2cStatus_t eStatus;      //״̬��,�ⲿֻ��
  unsigned char Index;                     //���ڱ�ʶ���ڴ�����һλ
  unsigned char ErrTimer;                  //����ʱ��
}I2cDev_t;

/**********************************************************************
                        ��غ���,���ⲿʹ��
**********************************************************************/

//------------------------------I2c�豸��ʼ������----------------------
//�˺�����ʼ���豸�ṹ�������ҽӵ�I2cӲ����ʼ��
//ע���˺������������ó�I2cӲ������������֣������ţ��жϣ�ʱ�ӵ�
void I2cDev_Init(I2cDev_t *pI2cDev,        //δ��ʼ�����豸ָ��
                 void *pI2cHw,             //�ҽӵ�I2cӲ��
                 unsigned int Mck,         //��ʱ��
                 unsigned int Baudrate);   //�趨�Ĳ�����

//-----------------------------I2c��д������������-------------------------
////�����Ƿ�ɹ� 0�ɹ�,��0ʧ��
//��״̬�����������ڼ����ʱ�������ش�ʶ
signed char I2cDev_ReStart(I2cDev_t *pI2cDev, //�豸ָ��
                           I2cData_t *pData);

//-----------------------------I2cǿ�Ƹ�λ����-------------------------
//ֹͣ��ǿ��I2c��λ
void I2cDev_Reset(I2cDev_t *pI2cDev);

//-----------------------------I2c�жϴ�������-------------------------
//���˺��������жϴ���������
void I2cDev_IRQ(I2cDev_t *pI2cDev);

//--------------------------------I2c������--------------------------
//�����������ϵͳTICK��
void I2cDev_Task(I2cDev_t *pI2cDev);

//--------------------------------I2c�Ƿ��������--------------------------
//���ط�0��ʾ����������������ȴ�
signed char I2cDev_IsEnd(const I2cDev_t *pI2cDev);

//--------------------------I2c�õ�״̬����------------------------------
//enum eI2cStatus_t I2cDev_eGetSatate(const I2cDev_t *pI2cDev);
#define I2cDev_eGetSatate(pI2cDev) ((pI2cDev)->eStatus)

#endif //#define __I2C_DEV_H