/*********************************************************************

			        通用I2c主机设备驱动程序接口
 此接口支持多路I2c设备，并支持I2c总线上与多个从设备的通讯
*********************************************************************/
#ifndef __I2C_DEV_H
#define __I2C_DEV_H

/**********************************************************************
                          相关说明
**********************************************************************/
//1.为方便使用,I2c数据定义为专用,从机在I2cSlave 定义为I2cSlaveData_t
//2.为节省数据存储空间,结构内部数据位置进行了优化(尽量4字节对齐)

/**********************************************************************
                        相关结构
**********************************************************************/

//I2c数据定义
typedef struct _I2cData{
  unsigned char SlvAdr;         //从机地址,0-127
  unsigned char CmdSize;        //命令字大小,可以为0
  unsigned char DataSize;       //数据字大小,>= 1;
  unsigned char Flag;           //相关标志字，见定义

  const unsigned char *pCmd;    //命令字
  unsigned char *pData;         //数据
}I2cData_t;

//其中，相关标志字定义为：
#define   I2C_WAIT_OV_MASK  0x3f   //I2c等待时间，0-64TICK
#define   I2C_CMD_RD        0x80      //读标志
#define   I2C_CMD_WR        0x00      //写标志

//I2c总线工作状态机定义
enum eI2cStatus_t
{
  //I2c停止期间:
  eI2cIdie     = 0,     //总线空闲
  //I2c运行期间:
  eI2cRdy      = 1,     //I2c准备状态
  eI2cWrCmd    = 2,     //写命令状态
  eI2cWr       = 3,     //写数据状态
  eI2cRd       = 4,     //读数据状态,等待从机数据,并向从机应答!
  eI2cRdLast   = 5,     //读最后一个数,不向从机返回应答信号，并准备结束
  //I2c结束期间:
  eI2cDone     = 6,     //读写数据完成,等待数据处理
  eI2cErrState = 7,     //I2c状态机错误
  eI2cErrOV    = 8      //I2c等待超时错误
};

//I2c设备定义
typedef struct _I2cDev_t{
  void *pI2cHw;                            //挂接的I2c硬件设备指针
  I2cData_t  *pData;                        //正在通读的I2c从机设备或命令
  volatile enum eI2cStatus_t eStatus;      //状态机,外部只读
  unsigned char Index;                     //用于标识正在处理那一位
  unsigned char ErrTimer;                  //错误定时器
}I2cDev_t;

/**********************************************************************
                        相关函数,供外部使用
**********************************************************************/

//------------------------------I2c设备初始化函数----------------------
//此函数将始化设备结构，并将挂接的I2c硬件初始化
//注：此函数不负责配置除I2c硬件外的其它部分，如引脚，中断，时钟等
void I2cDev_Init(I2cDev_t *pI2cDev,        //未初始化的设备指针
                 void *pI2cHw,             //挂接的I2c硬件
                 unsigned int Mck,         //主时钟
                 unsigned int Baudrate);   //设定的波特率

//-----------------------------I2c读写数据启动函数-------------------------
////返回是否成功 0成功,非0失败
//当状态机正在运行期间访问时，将返回错识
signed char I2cDev_ReStart(I2cDev_t *pI2cDev, //设备指针
                           I2cData_t *pData);

//-----------------------------I2c强制复位函数-------------------------
//停止并强制I2c复位
void I2cDev_Reset(I2cDev_t *pI2cDev);

//-----------------------------I2c中断处理程序-------------------------
//将此函数放入中断处理程序中
void I2cDev_IRQ(I2cDev_t *pI2cDev);

//--------------------------------I2c任务函数--------------------------
//将此任务放入系统TICK中
void I2cDev_Task(I2cDev_t *pI2cDev);

//--------------------------------I2c是否结束函数--------------------------
//返回非0表示结束，否则需继续等待
signed char I2cDev_IsEnd(const I2cDev_t *pI2cDev);

//--------------------------I2c得到状态函数------------------------------
//enum eI2cStatus_t I2cDev_eGetSatate(const I2cDev_t *pI2cDev);
#define I2cDev_eGetSatate(pI2cDev) ((pI2cDev)->eStatus)

#endif //#define __I2C_DEV_H
