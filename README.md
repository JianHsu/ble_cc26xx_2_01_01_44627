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

## 2-6.新增PWM控制RGB LED
	
	ti\tirtos_cc13xx_cc26xx_2_21_00_06\products\tidrivers_cc13xx_cc26xx_2_21_00_04\packages\ti\drivers\
	ti\tirtos_cc13xx_cc26xx_2_21_00_06\products\tidrivers_cc13xx_cc26xx_2_21_00_04\packages\ti\drivers\pwm
	ti\tirtos_cc13xx_cc26xx_2_21_00_06\products\tidrivers_cc13xx_cc26xx_2_21_00_04\packages\ti\drivers\timer
將裡面的PWM.c(.h)、PWMTimerCC26XX.c(.h)、GPTimerCC26XX.c(.h)複製到
	
	ti\tirtos_simplelink_2_13_00_06\packages\ti\drivers

再將6個檔案include到專案中，並且參考Sample Code，將PWM Driver 引入。
