# 本仓库旨在将传感器驱动移植到STM32上（C语言）
## 1. TOF激光测距——VL53L0X
<div align=left><img src="https://github.com/Potatotatotato/STM32-SensorDevicePorting/blob/master/TOF_LASER_VL53L0X/VL53L0X.jpg" width = 300></div>  

> VL53L0X是新一代飞行时间（ToF）激光测距模块（不同于传统技术），采用目前市场上最小的封装，无论目标反射率如何，都能提供精确的距离测量。它可以测量2m的绝对距离，为测距性能等级设定了新的基准，为各种新应用打开了大门。
> 
> VL53L0X集成了一个领先的SPAD阵列（单光子雪崩二极管），并内嵌ST的第二代FlightSense™专利技术。
> 
> VL53L0X的940nm VCSEL发射器（垂直腔面发射激光器）完全不为人眼所见，加上内置的物理红外滤光片，使其测距距离更长，对环境光的免疫性更强，对盖片的光学串扰具有更好的稳定性。
