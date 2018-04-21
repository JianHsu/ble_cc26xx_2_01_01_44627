/*
 * mpu6500.c
 *
 *  Created on: Jul 8, 2017
 *      Author: Henorvell
 */

#include "mpu6500.h"

#include <stdint.h>

int MPU6500_Init(unsigned int lpf)
{
    unsigned int default_filter = 1;

    //选择mpu6500内部数字低筒滤波器带宽
    //不开启内部低通滤波，陀螺仪采样率 8MHz
    //  开启内部低通滤波，陀螺仪采样率 1MHz
    //加速度计采样率1MHz
    switch(lpf)
    {
    case 5:
        default_filter = MPU6500_DLPF_BW_5;
        break;
    case 10:
        default_filter = MPU6500_DLPF_BW_10;
        break;
    case 20:
        default_filter = MPU6500_DLPF_BW_20;
        break;
    case 42:
        default_filter = MPU6500_DLPF_BW_42;
        break;
    case 98:
        default_filter = MPU6500_DLPF_BW_98;
        break;
    case 188:
        default_filter = MPU6500_DLPF_BW_188;
        break;
    case 256:
        default_filter = MPU6500_DLPF_BW_256;
        break;
    default:
        default_filter = MPU6500_DLPF_BW_42;
        break;
    }

    //delay_ms(200);

    /*//设备复位
//  IIC_Write_1Byte(MPU6500_SLAVE_ADDRESS,MPU6500_RA_PWR_MGMT_1, 0x80);

    //这里使用的Delay()只能在初始化阶段使用，任务调度中使用这种Delay()，会卡死整个调度
    MPU6500_setSleepEnabled(0); //进入工作状态
    delay_ms(10);
    MPU6500_setClockSource(MPU6500_CLOCK_PLL_ZGYRO);    //设置时钟  0x6b   0x03
                                                        //时钟源选择，MPU6500_CLOCK_INTERNAL表示内部8M晶振
    delay_ms(10);
    MPU6500_set_SMPLRT_DIV(1000);  //1000hz
    delay_ms(10);
    MPU6500_setFullScaleGyroRange(MPU6500_GYRO_FS_2000);//陀螺仪最大量程 +-2000度每秒
    delay_ms(10);
    MPU6500_setFullScaleAccelRange(MPU6500_ACCEL_FS_8); //加速度度最大量程 +-8G
    delay_ms(10);
    MPU6500_setDLPF(default_filter);  //42hz
    delay_ms(10);
    MPU6500_setI2CMasterModeEnabled(0);  //不让MPU6500 控制AUXI2C
    delay_ms(10);
    MPU6500_setI2CBypassEnabled(1);  //主控制器的I2C与    MPU6500的AUXI2C  直通。控制器可以直接访问HMC5883L
    delay_ms(10);*/

    return 0;
}

//读取MPU6500输出寄存器数值
void MPU6500_Read(MPU6500_STRUCT * mpu6500)
{
    //IIC_Read_nByte(MPU6500_SLAVE_ADDRESS,MPU6500_RA_ACCEL_XOUT_H,14,mpu6500->mpu6500_buffer);

    /*拼接buffer原始数据*/
    mpu6500->Acc_I16.x = ((((int16_t)mpu6500->mpu6500_buffer[0]) << 8) | mpu6500->mpu6500_buffer[1]) ;
    mpu6500->Acc_I16.y = ((((int16_t)mpu6500->mpu6500_buffer[2]) << 8) | mpu6500->mpu6500_buffer[3]) ;
    mpu6500->Acc_I16.z = ((((int16_t)mpu6500->mpu6500_buffer[4]) << 8) | mpu6500->mpu6500_buffer[5]) ;

    mpu6500->Gyro_I16.x = ((((int16_t)mpu6500->mpu6500_buffer[8]) << 8) | mpu6500->mpu6500_buffer[9]) ;
    mpu6500->Gyro_I16.y = ((((int16_t)mpu6500->mpu6500_buffer[10]) << 8) | mpu6500->mpu6500_buffer[11]) ;
    mpu6500->Gyro_I16.z = ((((int16_t)mpu6500->mpu6500_buffer[12]) << 8) | mpu6500->mpu6500_buffer[13]) ;

    mpu6500->Tempreature = ((((int16_t)mpu6500->mpu6500_buffer[6]) << 8) | mpu6500->mpu6500_buffer[7]);

}
