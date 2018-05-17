# 1.下載TI CC2650 SDK：ver2.1.1

	http://www.ti.com/tool/ble-stack
	
# 2.開啟範例程式HIDEmuKbd做修改
## 2-1. 修改Board.h

	http://processors.wiki.ti.com/index.php/CC2650_LaunchPad_User%27s_Guide_for_Bluetooth_Smart
	
## 2-2. 加入滑鼠PROFILE
	
	hidkbdservice.c(.h) -> hidkbmservice.c(.h)
	
## 2-3. 新增UART功能
	
	http://processors.wiki.ti.com/index.php/CC26xx_Adding_basic_printf_over_uart_with_TI-RTOS
	
## 2-4. 新增I2C功能並讀取MPU9250
參考SensorTag Sample Code，將I2C Driver引入，並讀取MPU9250。

## 2-5. 引入AHRS演算法

	https://github.com/kriswiner/MPU9250/blob/master/MPU9250BasicAHRS.ino
	
此範例程式將MPU9250操作在BYPASS MODE，但手邊的MPU9250內的AK8963在100KHz下無法讀取，故修改為I2C Master Mode由MPU9250去讀取AK8963的資料。
