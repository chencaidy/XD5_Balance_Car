#“恩智浦” 杯智能汽车大赛 - 新电五队托管项目

 `天津科技大学` `新电社` `新电五队` `平衡组` `摄像头`

### 硬件详情：

- 车模：E
- 主控：MK64FX512VLL12
- 电机驱动：IR2104S + LR7843
- 摄像头：野火 OV7725
- 陀螺仪：Invensense MPU9250
- 编码器：欧姆龙 E6A2-CW3C

### 编译环境：

- MCUXpresso with KSDK 2.2（请确保软件版本相同，防止出现意外情况）

### 工作区结构：

- `[CMSIS]` -- ARM Cortex 微控制器软件接口标准
- `[drivers]` -- Kinetis SDK 外设驱动
- `[freertos]` -- FreeRTOS 内核
- `[middleware]` -- Kinetis 中间件
- `[source]` -- 用户代码编辑区
- `[startup]` -- Kinetis 启动代码
- `[systemview]` -- SEGGER SystemView 内核监视器
- `[utilities]` -- Kinetis 调试控制台


### [source]结构：

- `[./]` -- 应用文件（算法、应用层等）
- `[board]` -- 板级时钟，引脚复用配置
- `[drivers]` -- 板载外设驱动文件
- `[eMPL]` -- Invensense 姿态处理库
