# VL53L0驱动移植
<div align=left><img src="https://github.com/Potatotatotato/STM32-SensorDevicePorting/blob/master/TOF_LASER_VL53L0X/VL53L0X.jpg" width = 300></div>

## 移植概况
将Arduino例程移植到STM32F407上
## 总结
#### 语法问题
1. keil中c文件不支持`bool`类型，我把所有`bool`替换为了`uint8_t`。
2. 变量声明必须放在函数的最前面。
3. keil的c文件中函数变量无法赋默认值？
```c
uint8_t VL530LX_init(uint8_t io_2v8, uint16_t timeout = 0); //wrong
```
```c
uint8_t VL530LX_init(uint8_t io_2v8, uint16_t timeout); //ok
```
#### cpp中类的优势
1. 防止方法与其他文件重复。按照c语言的语法，要么通过static限制函数作用域，从而防止函数重复；要么给函数加上特定的标识符（如前缀）。
2. 变量的归属更加明确。 
3. 函数可以重载。
