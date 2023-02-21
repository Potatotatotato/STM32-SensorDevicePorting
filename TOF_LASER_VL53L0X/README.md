# VL53L0驱动移植
<div align=center><img src="https://github.com/Potatotatotato/STM32-SensorDevicePorting/blob/master/TOF_LASER_VL53L0X/VL53L0X.jpg"></div>

## 移植概况
将Arduino例程移植到STM32F407上
## 总结
#### 语法问题
1. keil中c文件不支持`bool`类型
2. 变量声明必须放在函数的最前面
#### cpp中类的优势
1. 防止方法与其他文件重复。按照c语言的语法，要么通过static来防止函数重复，要么给函数加上特定的标识符（如前缀）
2. 变量的归属更加明确。 
