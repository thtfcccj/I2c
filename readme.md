﻿##嵌入式系统通用驱动程序接口及其实现-I2C(主机)驱动程序

* 此接口为具体项目中: 需要I2C通讯时，提供统一的操作函数。

####软件结构说明:
此接口含两大部分
* I2C设备驱动程序部分：
  + **I2C通用操作接口:**  即对外接口调用文件:**I2cDev.h** ，其它软件部分操作I2C时,**包含此文件即可**。
  + **I2C操作接口的各种实现:** 以*I2cDev(下横线_)I2C硬件名称(或载体)(下横线_)专用编译环境(可选,严重不建议代码与编译环境相关连)*命名, 与具体使用的I2C硬件的接口有关，即有各种不同的实现。但一种硬件仅实现一次，项目中需要那个加入那个即可，具有通用性(*实现时需尽量排除编译器影响，使一个I2C驱动的实现能在各种编译器里运行*)。
* I2C设备实例化部分：
  + **I2C通用实例化接口:**  即对外接口调用文件:**I2c.h** ，其它软件部分需要具体I2c设备时,**包含此文件即可**。
  + **I2C操作接口的各种实现:** 以*I2c(下横线_)I2c硬件名称(或载体)(下横线_)专用编译环境(可选,严重不建议代码与编译环境相关连)*命名, 与具体使用的I2C硬件有关，即有各种不同的实现。但一种硬件仅实现一次，项目中需要那个加入那个即可，具有通用性(*实现时需尽量排除编译器影响，使一个I2C驱动的实现能在各种编译器里运行*)。

####使用说明：
* I2cDev部分：
 + 1.根据项目嵌入式硬件不同，将I2cDev.h和**与项目对应的**的具体实现文件。增加到开发环境中。
 + 2.(此步可选) 查看具体实现文件文件内，是否有“编译选项”部分，若有，根据“编译选项”提示，在系统预编译头文件中增加相应配置。**为保证I2cDev实现文件的通用性，严禁对此实现做任何的改动**(BUG或提升功能与性能除外），若有不适用性，请复制后，包含复制文件进行操作。

* I2C设备实例化部分：基础实现： I2c.h及其实现文件
 + 1.根据项目嵌入式硬件不同，将I2c.h和**与项目对应的**的具体实现文件。增加到开发环境中。
 + 2.(此步可选) 查看I2c项目内实现文件文件内，是否有“编译选项”部分，若有，根据“编译选项”提示，在系统预编译头文件中增加相应配置。**为保证I2c.h通用性，严禁对此实现做任何的改动**(BUG或提升功能与性能除外），若有不适用性，请复制后，包含复制文件进行操作。

* I2C设备实例化部分：管理器实现： I2cMng.h及其实现与应用相关回调文件
    与基础实现不同的是，这里实现了波特率等参数的EEPROM读写与配置
   根据项目应用不同，将I2cMng.h和**与项目对应的**的具体回调实现文件(可放在其它地方)。增加到开发环境中。

####目录结构组织：
* **小型项目时**: 即不区分组件层，放在“项目源文件目录\I2c”下，内部不再有子目录
* **大中型项目时**: 区分组件层，放在“项目源文件目录\components\I2c”下，若项目很多，且同一项目也有较多实现时，可将具体实现文件放在此目录“项目名称”目录下，以实现分类存放。

-------------------------------------------------------------------------------

##开源项目说明
* 为各类单片机提供模板支持,**欢迎大家增加对各类嵌入式硬件的操作模板**,以让更多人使用
* 版权声明: ...ch这世道，说了也等于白说，总之以下点：
 + **源代码部分：** 可以自由使用，源代码中，也不需做任何版权声明。
 + **分享时：** 为防止碎化片，请注明出处，以利于开源项目的推广。
 + **关于fork：**  这个欢迎(但为防止碎化片化，请不要分支或单独推广)。更欢迎为此开源项目直接贡献代码。 

##此开源项目对应的教程
* 视频在分享平台：http://thtfcccj.56.com
* 与视频同步输入的文字,在http://blog.csdn.net/thtfcccj
* 同步的开源项目，则在代码托管平台：https://github.com/thtfcccj

##此开源项目对应的两个件教学视频：
* 嵌入式系统通用驱动程序接口及其实现2-EERPOM操作标准化
* 嵌入式系统通用驱动程序接口及其实现n-EERPOM存取位置标准化










