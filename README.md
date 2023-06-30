# Control_Frame
#### 注：此文档为临时文档，只对部分内容进行说明，供水下组使用，后续会进行完善（应该是暑假）
## 如何控制一个电机？
### 1. 电机实例化
此部分涉及到的文件有：devices下Motor、CommuType、tasks下ChassisTask。
#### 1.1 Motor
Motor类中存放电机的控制方式（ctrlType)和所需的pid（speedpid、anglepid）,并负责所有电机的中断处理，**需在定时器中断中调用MotorsHandle函数**。
控制方式分为三种：DIRECT（电机内部PID控制）、SPEED_Single（速度控制）、POSITION_Double（位置控制）。  
FOUR_Motor_4010为脉塔4010多电机类，继承自Motor类和CAN类。使用此控制方式需提前使用上位机将电机的多电机模式打开。实例化后只需调用SetTarAngle函数即可控制电机转动到指定角度。
#### 1.2 CommuType
CAN类中存放canID和can通道（CAN1orCAN2），并负责can消息包的发送，**需在定时器中断中调用CANPackageSend函数**。
#### 1.3 ChassisTask
此文件负责电机具体的实例化,实例化前，需要准备的初始化参数有MotorInit、CommuInit，其中MotorInit又包括pid参数，需提前设置。  
由于当前数据来源为遥控器，此处ChassisHandle函数作为中断处理函数，**需在定时器中断中调用**，用于处理遥控器数据并调用每个电机的SetTargetAngle函数。
### 2、接收上位机数据
此部分涉及文件ManiControl。
#### 2.1 ManiControl
使用时主要关注GetData函数，此函数用于处理接收到的数据。  
目前程序中接收数据（BUFF_SIZE)最多为28位，可根据需要上调。若已知接收数据长度，可更改
CONTROL_LENGTH为对应长度，更加保险，若接收不定长数据，可注释相应部分，后续再进行校验。


