/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** ============================================================================
 *  @file       sensortag_i2c.c
 *
 *  @brief      Simple interface to the TI-RTOS driver. Also manages switching
 *              between I2C-buses.
 *
 *  ============================================================================
 */

/* -----------------------------------------------------------------------------
*  Includes
* ------------------------------------------------------------------------------
*/
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#include "math.h"
#include "myi2c.h"
#include "mpu9250.h"
/* -----------------------------------------------------------------------------
*  Constants
* ------------------------------------------------------------------------------
*/
#define SELF_TEST_X_GYRO    0x00
#define SELF_TEST_Y_GYRO    0x01
#define SELF_TEST_Z_GYRO    0x02
#define SELF_TEST_X_ACCEL   0x0D
#define SELF_TEST_Y_ACCEL   0x0E
#define SELF_TEST_Z_ACCEL   0x0F

#define SELF_TEST_A         0x10

#define XG_OFFSET_H         0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L         0x14
#define YG_OFFSET_H         0x15
#define YG_OFFSET_L         0x16
#define ZG_OFFSET_H         0x17
#define ZG_OFFSET_L         0x18
#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define ACCEL_CONFIG2       0x1D
#define LP_ACCEL_ODR        0x1E
#define WOM_THR             0x1F

#define MOT_DUR             0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR            0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR           0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN             0x23
#define I2C_MST_CTRL        0x24
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV1_ADDR       0x28
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A
#define I2C_SLV2_ADDR       0x2B
#define I2C_SLV2_REG        0x2C
#define I2C_SLV2_CTRL       0x2D
#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30
#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define DMP_INT_STATUS      0x39  // Check DMP interrupt
#define INT_STATUS          0x3A
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54
#define EXT_SENS_DATA_12    0x55
#define EXT_SENS_DATA_13    0x56
#define EXT_SENS_DATA_14    0x57
#define EXT_SENS_DATA_15    0x58
#define EXT_SENS_DATA_16    0x59
#define EXT_SENS_DATA_17    0x5A
#define EXT_SENS_DATA_18    0x5B
#define EXT_SENS_DATA_19    0x5C
#define EXT_SENS_DATA_20    0x5D
#define EXT_SENS_DATA_21    0x5E
#define EXT_SENS_DATA_22    0x5F
#define EXT_SENS_DATA_23    0x60
#define MOT_DETECT_STATUS   0x61
#define I2C_SLV0_DO         0x63
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_DO         0x66
#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET   0x68
#define MOT_DETECT_CTRL     0x69
#define USER_CTRL           0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1          0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2          0x6C
#define DMP_BANK            0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT          0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG             0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1           0x70
#define DMP_REG_2           0x71
#define FIFO_COUNTH         0x72
#define FIFO_COUNTL         0x73
#define FIFO_R_W            0x74
#define WHO_AM_I            0x75 // Should return 0x71
#define XA_OFFSET_H         0x77
#define XA_OFFSET_L         0x78
#define YA_OFFSET_H         0x7A
#define YA_OFFSET_L         0x7B
#define ZA_OFFSET_H         0x7D
#define ZA_OFFSET_L         0x7E

// Magnetometer registers
#define MAG_WHO_AM_I        0x00  // Should return 0x48
#define MAG_INFO            0x01
#define MAG_ST1             0x02  // Data ready status: bit 0
#define MAG_XOUT_L          0x03  // Data array
#define MAG_XOUT_H          0x04
#define MAG_YOUT_L          0x05
#define MAG_YOUT_H          0x06
#define MAG_ZOUT_L          0x07
#define MAG_ZOUT_H          0x08
#define MAG_ST2             0x09  // Overflow(bit 3), read err(bit 2)
#define MAG_CNTL1           0x0A  // Mode bits 3:0, resolution bit 4
#define MAG_CNTL2           0x0B  // System reset, bit 0
#define MAG_ASTC            0x0C  // Self test control
#define MAG_I2CDIS          0x0F  // I2C disable
#define MAG_ASAX            0x10  // x-axis sensitivity adjustment
#define MAG_ASAY            0x11  // y-axis sensitivity adjustment
#define MAG_ASAZ            0x12  // z-axis sensitivity adjustment

#define MAG_DEVICE_ID                 0x48

#define DELAY_MS(i)      (Task_sleep(((i) * 1000) / Clock_tickPeriod))
#define DELAY_US(i)      (Task_sleep(((i) * 1) / Clock_tickPeriod))

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)

//**float GyroMeasError = M_PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
//**float GyroMeasDrift = M_PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
//**float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
//**float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f
#define PI M_PI
/* -----------------------------------------------------------------------------
*  Public Variables
* ------------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
*  Private Variables
* ------------------------------------------------------------------------------
*/
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
uint32_t Now = 0;        // used to calculate integration interval


/* -----------------------------------------------------------------------------
*  Public Functions
* ------------------------------------------------------------------------------
*/

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

float getMres(int16_t rawData) {
    float v;
    switch (Mscale)
    {
        // Possible magnetometer scales (and their register bit settings) are:
        // 14 bit resolution (0) and 16 bit resolution (1)
        case MFS_14BITS:
            v = rawData * 10.0 * 4912.0 / 8190.0; // Proper scale to return milliGauss
        case MFS_16BITS:
            v = rawData * 10.0 * 4912.0 / 32760.0; // Proper scale to return milliGauss
    }
    return v;
}

float getGres(int16_t rawData) {
    float v;

    switch (Gscale)
    {
        // Possible gyro scales (and their register bit settings) are:
        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case GFS_250DPS:
            v = (rawData * 1.0) * 250 / 32768;
            break;
        case GFS_500DPS:
            v = (rawData * 1.0) * 500 / 32768;
            break;
        case GFS_1000DPS:
            v = (rawData * 1.0) * 1000 / 32768;
            break;
        case GFS_2000DPS:
            v = (rawData * 1.0) * 2000 / 32768;
            break;
    }
    return v;
}

float getAres(int16_t rawData) {
    float v;

    switch (Ascale)
    {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case AFS_2G:
            v = (rawData * 1.0) * 2 / 32768;
            break;
        case AFS_4G:
            v = (rawData * 1.0) * 4 / 32768;
            break;
        case AFS_8G:
            v = (rawData * 1.0) * 8 / 32768;
            break;
        case AFS_16G:
            v = (rawData * 1.0) * 16 / 32768;
            break;
    }
    return v;
}


void readAccelData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z accel register data stored here
    SensorI2C_readReg(MPU9250_ADDRESS, ACCEL_XOUT_H, &rawData[0], 6);  // Read the six raw data registers into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void readGyroData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    SensorI2C_readReg(MPU9250_ADDRESS, GYRO_XOUT_H, &rawData[0], 6);  // Read the six raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readMagData(int16_t * destination)
{
    uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    uint8_t st1;

    SensorI2C_readReg(AK8963_ADDRESS, MAG_ST1, &st1, 1);
    if(st1 & 0x01) { // wait for magnetometer data ready bit to be set
        SensorI2C_readReg(AK8963_ADDRESS, MAG_XOUT_L, &rawData[0], 7);  // Read the six raw data and ST2 registers sequentially into data array
        uint8_t c = rawData[6]; // End data read by reading ST2 register
        if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
            destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
            destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
            destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
        }
    }
}

int16_t readTempData()
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here
    SensorI2C_readReg(MPU9250_ADDRESS, TEMP_OUT_H, &rawData[0], 2);  // Read the two raw data registers sequentially into data array
    return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

void initAK8963(float * destination)
{

    // First extract the factory calibration for each magnetometer axis
    uint8_t rawData[3];  // x/y/z gyro calibration data stored here
    SensorI2C_writeReg(AK8963_ADDRESS, MAG_CNTL1, 0x00); // Power down magnetometer
    DELAY_MS(50);
    SensorI2C_writeReg(AK8963_ADDRESS, MAG_CNTL1, 0x0F); // Enter Fuse ROM access mode
    DELAY_MS(50);
    SensorI2C_readReg(AK8963_ADDRESS, MAG_ASAX, &rawData[0], 3);  // Read the x-, y-, and z-axis calibration values
    destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
    destination[1] =  (float)(rawData[1] - 128)/256. + 1.;
    destination[2] =  (float)(rawData[2] - 128)/256. + 1.;
    SensorI2C_writeReg(AK8963_ADDRESS, MAG_CNTL1, 0x00); // Power down magnetometer
    DELAY_MS(50);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates

    SensorI2C_writeReg(AK8963_ADDRESS, MAG_CNTL1, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
    DELAY_MS(50);
}

void magcalMPU9250(float * dest1, float * dest2) {
    uint16_t ii = 0, jj=0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0};
    int16_t mag_max[3] = {0x8000, 0x8000, 0x8000}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};

    System_printf("Mag Calibration: Wave device in a figure eight until done!");
    DELAY_MS(1000);

    if(Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
    if(Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms

    for(ii = 0; ii < sample_count; ii++) {
        readMagData(mag_temp);  // Read the mag data
        for (jj = 0; jj < 3; jj++) {
            if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
            if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
        }
        if(Mmode == 0x02) DELAY_MS(135);  // at 8 Hz ODR, new mag data is available every 125 ms
        if(Mmode == 0x06) DELAY_MS(12);  // at 100 Hz ODR, new mag data is available every 10 ms
    }

    System_printf("mag x min/max:%d, %d\r\n", mag_max[0], mag_min[0]);
    System_printf("mag y min/max:%d, %d\r\n", mag_max[1], mag_min[1]);
    System_printf("mag z min/max:%d, %d\r\n", mag_max[2], mag_min[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    dest1[0] = getMres(mag_bias[0]) * dest2[0];
    dest1[1] = getMres(mag_bias[1]) * dest2[1];
    dest1[2] = getMres(mag_bias[2]) * dest2[2];
    /*
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] ¡V mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] ¡V mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] ¡V mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;
    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
    */
    System_printf("Mag bias:%f, %f, %f\r\n", dest1[0], dest1[1], dest1[2]);
}
void initMPU9250(void)
{
    uint8_t c;
    // wake up device
    SensorI2C_writeReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    DELAY_MS(100); // Wait for all registers to reset

    // get stable time source
    SensorI2C_writeReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
    DELAY_MS(200);

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
    // minimum DELAY_MS time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    SensorI2C_writeReg(MPU9250_ADDRESS, CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    SensorI2C_writeReg(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                    // determined inset in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    SensorI2C_readReg(MPU9250_ADDRESS, GYRO_CONFIG, &c, 1); // get current GYRO_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x02; // Clear Fchoice bits [1:0]
    c = c & ~0x18; // Clear GFS bits [4:3]
    c = c | Gscale << 3; // Set full scale range for the gyro
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    SensorI2C_writeReg(MPU9250_ADDRESS, GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    SensorI2C_readReg(MPU9250_ADDRESS, ACCEL_CONFIG, &c, 1); // get current ACCEL_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | Ascale << 3; // Set full scale range for the accelerometer
    SensorI2C_writeReg(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    SensorI2C_readReg(MPU9250_ADDRESS, ACCEL_CONFIG2, &c, 1); // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    SensorI2C_writeReg(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    SensorI2C_writeReg(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
    SensorI2C_writeReg(MPU9250_ADDRESS, INT_ENABLE, 0x00);  // Disable data ready (bit 0) interrupt
    DELAY_MS(100);

}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * dest1, float * dest2)
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device
    SensorI2C_writeReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    DELAY_MS(100);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
    SensorI2C_writeReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    SensorI2C_writeReg(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
    DELAY_MS(200);

    // Configure device for bias calculation
    SensorI2C_writeReg(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
    SensorI2C_writeReg(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
    SensorI2C_writeReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
    SensorI2C_writeReg(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
    SensorI2C_writeReg(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    SensorI2C_writeReg(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
    DELAY_MS(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    SensorI2C_writeReg(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    SensorI2C_writeReg(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    SensorI2C_writeReg(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    SensorI2C_writeReg(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    float  gyrosensitivity  = 1.0 / getGres(1);   // = 131 LSB/degrees/sec
    float  accelsensitivity = 1.0 / getAres(1);  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    SensorI2C_writeReg(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
    SensorI2C_writeReg(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    DELAY_MS(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    SensorI2C_writeReg(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    SensorI2C_readReg(MPU9250_ADDRESS, FIFO_COUNTH, &data[0], 2); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        SensorI2C_readReg(MPU9250_ADDRESS, FIFO_R_W, &data[0], 12); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accelsensitivity;}

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    // Push gyro biases to hardware registers
    SensorI2C_writeReg(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
    SensorI2C_writeReg(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
    SensorI2C_writeReg(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
    SensorI2C_writeReg(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
    SensorI2C_writeReg(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
    SensorI2C_writeReg(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

    // Output scaled gyro biases for display in the main program
    dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
    dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    SensorI2C_readReg(MPU9250_ADDRESS, XA_OFFSET_H, &data[0], 2); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    SensorI2C_readReg(MPU9250_ADDRESS, YA_OFFSET_H, &data[0], 2);
    accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    SensorI2C_readReg(MPU9250_ADDRESS, ZA_OFFSET_H, &data[0], 2);
    accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for(ii = 0; ii < 3; ii++) {
        if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Apparently this is not working for the acceleration biases in the MPU-9250
    // Are we handling the temperature correction bit properly?
    // Push accelerometer biases to hardware registers

    SensorI2C_writeReg(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
    SensorI2C_writeReg(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
    SensorI2C_writeReg(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
    SensorI2C_writeReg(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
    SensorI2C_writeReg(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
    SensorI2C_writeReg(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

    // Output scaled accelerometer biases for display in the main program
    dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
    dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
    dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
    uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
    uint8_t selfTest[6];
    int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
    float factoryTrim[6];
    uint8_t FS = 0;
    uint8_t ii;

    SensorI2C_writeReg(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
    SensorI2C_writeReg(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    SensorI2C_writeReg(MPU9250_ADDRESS, GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
    SensorI2C_writeReg(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    SensorI2C_writeReg(MPU9250_ADDRESS, ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

    for(ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

        SensorI2C_readReg(MPU9250_ADDRESS, ACCEL_XOUT_H, &rawData[0], 6);        // Read the six raw data registers into data array
        aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
        aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

        SensorI2C_readReg(MPU9250_ADDRESS, GYRO_XOUT_H, &rawData[0], 6);       // Read the six raw data registers sequentially into data array
        gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
        gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
    }

    for (ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
        aAvg[ii] /= 200;
        gAvg[ii] /= 200;
    }

    // Configure the accelerometer for self-test
    SensorI2C_writeReg(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
    SensorI2C_writeReg(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    DELAY_MS(25);  // DELAY_MS a while to let the device stabilize

    for(ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer

        SensorI2C_readReg(MPU9250_ADDRESS, ACCEL_XOUT_H, &rawData[0], 6);  // Read the six raw data registers into data array
        aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
        aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

        SensorI2C_readReg(MPU9250_ADDRESS, GYRO_XOUT_H, &rawData[0], 6);  // Read the six raw data registers sequentially into data array
        gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
        gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
    }

    for (ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
        aSTAvg[ii] /= 200;
        gSTAvg[ii] /= 200;
    }

    // Configure the gyro and accelerometer for normal operation
    SensorI2C_writeReg(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
    SensorI2C_writeReg(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
    DELAY_MS(25);  // DELAY_MS a while to let the device stabilize

    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    SensorI2C_readReg(MPU9250_ADDRESS, SELF_TEST_X_ACCEL, &selfTest[0], 3); // X-axis accel self-test results
    SensorI2C_readReg(MPU9250_ADDRESS, SELF_TEST_X_GYRO, &selfTest[3], 3);  // X-axis gyro self-test results


    // Retrieve factory self-test value from self-test code reads
    factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
    factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
    factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
    factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
    factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
    factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get percent, must multiply by 100
    for (ii = 0; ii < 3; ii++) {
        destination[ii]   = 100.0*((float)(aSTAvg[ii] - aAvg[ii]))/factoryTrim[ii];   // Report percent differences
        destination[ii+3] = 100.0*((float)(gSTAvg[ii] - gAvg[ii]))/factoryTrim[ii+3]; // Report percent differences
    }
}

bool GetMpuIntStatus(void)
{
    uint8_t val;
    SensorI2C_readReg(MPU9250_ADDRESS, INT_STATUS, &val, 1);

    return val & 0x01;
}
/*
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;
    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;
    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;
    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;
    // Reference direction of Earth¡¦s magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;

    hx = mx * q1q1 ¡V _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 ¡V mx * q3q3 ¡V mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 ¡V _2q1mz * q2 + _2q2mx * q3 ¡V my * q2q2 + my * q3q3 + _2q3 * mz * q4 ¡V my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 ¡V mz * q2q2 + _2q3 * my * q4 ¡V mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;
    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 ¡V _2q1q3 ¡V ax) + _2q2 * (2.0f * q1q2 + _2q3q4 ¡V ay) ¡V _2bz * q3 * (_2bx * (0.5f ¡V q3q3 ¡V q4q4) + _2bz * (q2q4 ¡V q1q3) ¡V mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 ¡V q1q4) + _2bz * (q1q2 + q3q4) ¡V my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f ¡V q2q2 ¡V q3q3) ¡V mz);
    s2 = _2q4 * (2.0f * q2q4 ¡V _2q1q3 ¡V ax) + _2q1 * (2.0f * q1q2 + _2q3q4 ¡V ay) ¡V 4.0f * q2 * (1.0f ¡V 2.0f * q2q2 ¡V 2.0f * q3q3 ¡V az) + _2bz * q4 * (_2bx * (0.5f ¡V q3q3 ¡V q4q4) + _2bz * (q2q4 ¡V q1q3) ¡V mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 ¡V q1q4) + _2bz * (q1q2 + q3q4) ¡V my) + (_2bx * q4 ¡V _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f ¡V q2q2 ¡V q3q3) ¡V mz);
    s3 = -_2q1 * (2.0f * q2q4 ¡V _2q1q3 ¡V ax) + _2q4 * (2.0f * q1q2 + _2q3q4 ¡V ay) ¡V 4.0f * q3 * (1.0f ¡V 2.0f * q2q2 ¡V 2.0f * q3q3 ¡V az) + (-_4bx * q3 ¡V _2bz * q1) * (_2bx * (0.5f ¡V q3q3 ¡V q4q4) + _2bz * (q2q4 ¡V q1q3) ¡V mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 ¡V q1q4) + _2bz * (q1q2 + q3q4) ¡V my) + (_2bx * q1 ¡V _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f ¡V q2q2 ¡V q3q3) ¡V mz);
    s4 = _2q2 * (2.0f * q2q4 ¡V _2q1q3 ¡V ax) + _2q3 * (2.0f * q1q2 + _2q3q4 ¡V ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f ¡V q3q3 ¡V q4q4) + _2bz * (q2q4 ¡V q1q3) ¡V mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 ¡V q1q4) + _2bz * (q1q2 + q3q4) ¡V my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f ¡V q2q2 ¡V q3q3) ¡V mz);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;
    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx ¡V q3 * gy ¡V q4 * gz) ¡V beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz ¡V q4 * gy) ¡V beta * s2;
    qDot3 = 0.5f * (q1 * gy ¡V q2 * gz + q4 * gx) ¡V beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy ¡V q3 * gx) ¡V beta * s4;
    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}
*/
/*
// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones.
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;
    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;
    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;
    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;
    // Reference direction of Earth¡¦s magnetic field
    hx = 2.0f * mx * (0.5f ¡V q3q3 ¡V q4q4) + 2.0f * my * (q2q3 ¡V q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f ¡V q2q2 ¡V q4q4) + 2.0f * mz * (q3q4 ¡V q1q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 ¡V q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f ¡V q2q2 ¡V q3q3);
    // Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 ¡V q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 ¡V q2q2 ¡V q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f ¡V q3q3 ¡V q4q4) + 2.0f * bz * (q2q4 ¡V q1q3);
    wy = 2.0f * bx * (q2q3 ¡V q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f ¡V q2q2 ¡V q3q3);
    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz ¡V az * vy) + (my * wz ¡V mz * wy);
    ey = (az * vx ¡V ax * vz) + (mz * wx ¡V mx * wz);
    ez = (ax * vy ¡V ay * vx) + (mx * wy ¡V my * wx);
    if (Ki > 0.0f)
    {
        eInt[0] += ex;      // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
    }
    else
    {
        eInt[0] = 0.0f;     // prevent integral wind up
        eInt[1] = 0.0f;
        eInt[2] = 0.0f;
    }
    // Apply feedback terms
    gx = gx + Kp * ex + Ki * eInt[0];
    gy = gy + Kp * ey + Ki * eInt[1];
    gz = gz + Kp * ez + Ki * eInt[2];
    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx ¡V q3 * gy ¡V q4 * gz) * (0.5f * deltat);
    q2 = pa + (q1 * gx + pb * gz ¡V pc * gy) * (0.5f * deltat);
    q3 = pb + (q1 * gy ¡V pa * gz + pc * gx) * (0.5f * deltat);
    q4 = pc + (q1 * gz + pa * gy ¡V pb * gx) * (0.5f * deltat);
    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}


void AHRS_loop(void)
{
    uint8_t val;

    SensorI2C_readReg(MPU9250_ADDRESS, INT_STATUS, &val, 1);
    // If intPin goes high, all data registers have new data
    if (val & 0x01) {  // On interrupt, check if data ready interrupt
        readAccelData(accelCount);  // Read the x/y/z adc values
        getAres();

        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0] * aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
        ay = (float)accelCount[1] * aRes; // - accelBias[1];
        az = (float)accelCount[2] * aRes; // - accelBias[2];

        readGyroData(gyroCount);  // Read the x/y/z adc values
        getGres();

        // Calculate the gyro value into actual degrees per second
        gx = (float)gyroCount[0] * gRes;  // get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1] * gRes;
        gz = (float)gyroCount[2] * gRes;

        readMagData(magCount);  // Read the x/y/z adc values
        getMres();
        magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
        magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
        magbias[2] = +125.;  // User environmental x-axis correction in milliGauss

        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental corrections
        mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
        my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];
        mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];
    }

    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
    // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
    // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
    // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
    // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
    // This is ok by aircraft orientation standards!
    // Pass gyro rate as rad/s
    //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
    MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);


    if (!AHRS) {
        if(SerialDebug) {
            // Print acceleration values in milligs!
            System_printf("X-acceleration: %f mg\r\n", 1000*ax);
            System_printf("Y-acceleration: %f mg\r\n", 1000*ay);
            System_printf("Z-acceleration: %f mg\r\n", 1000*az);

            // Print gyro values in degree/sec
            System_printf("X-gyro rate: %f degrees/sec\r\n", gx);
            System_printf("Y-gyro rate: %f degrees/sec\r\n", gy);
            System_printf("Z-gyro rate: %f degrees/sec\r\n", gz);

            // Print mag values in degree/sec
            System_printf("X-mag field: %f mG\r\n", mx);
            System_printf("Y-mag field: %f mG\r\n", my);
            System_printf("Z-mag field: %f mG\r\n", mz);

            tempCount = readTempData();  // Read the adc values
            temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
            // Print temperature in degrees Centigrade
            System_printf("Temperature is %f degrees C\r\n"); // Print T values to tenths of s degree C
        }
    }
    else {
        // Serial print and/or display at 0.5 s rate independent of data rates
        if(SerialDebug) {
            System_printf("ax = %d mg\r\n", (int)1000*ax);
            System_printf("ay = %d mg\r\n", (int)1000*ay);
            System_printf("az = %d mg\r\n", (int)1000*az);
            System_printf("gx = %f deg/s\r\n", gx);
            System_printf("gy = %f deg/s\r\n", gy);
            System_printf("gz = %f deg/s\r\n", gz);
            System_printf("mx = %d mG\r\n", (int)mx);
            System_printf("my = %d mG\r\n", (int)my);
            System_printf("mz = %d mG\r\n", (int)mz);

            System_printf("q0 = %f\r\n", q[0]);
            System_printf("qx = %f\r\n", q[1]);
            System_printf("qy = %f\r\n", q[2]);
            System_printf("qz = %f\r\n", q[3]);
        }
        // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
        // In this coordinate system, the positive z-axis is down toward Earth.
        // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
        // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
        // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
        // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
        // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
        // applied in the correct order which for this configuration is yaw, pitch, and then roll.
        // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
        yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
        pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI;
        yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
        roll  *= 180.0f / PI;

        if(SerialDebug) {
            System_printf("Yaw, Pitch, Roll: %f, %f, %f\r\n", yaw, pitch, roll);
            System_printf("rate = %f Hz\r\n", (float)sumCount/sum);
        }


        // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and
        // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
        // The filter update rate is determined mostly by the mathematical steps in the respective algorithms,
        // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
        // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
        // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively.
        // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
        // This filter update rate should be fast enough to maintain accurate platform orientation for
        // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
        // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
        // The 3.3 V 8 MHz Pro Mini is doing pretty well!
    }
}
*/
/*
void AHRS_loop(void)
{
    uint8_t val;

    SensorI2C_readReg(MPU9250_ADDRESS, INT_STATUS, &val, 1);
    // If intPin goes high, all data registers have new data
    if (val & 0x01) {  // On interrupt, check if data ready interrupt
        //1. Read the accelerate x/y/z adc values
        readAccelData(accelCount);
        getAres();
        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0] * aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
        ay = (float)accelCount[1] * aRes; // - accelBias[1];
        az = (float)accelCount[2] * aRes; // - accelBias[2];

        //2. Read the gyro x/y/z adc values
        readGyroData(gyroCount);
        getGres();
        // Calculate the gyro value into actual degrees per second
        gx = (float)gyroCount[0] * gRes;  // get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1] * gRes;
        gz = (float)gyroCount[2] * gRes;

        //3. Read the mag x/y/z adc values
        readMagData(magCount);
        getMres();
        magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
        magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
        magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental corrections
        mx = (float)magCount[0] * mRes * magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
        my = (float)magCount[1] * mRes * magCalibration[1] - magbias[1];
        mz = (float)magCount[2] * mRes * magCalibration[2] - magbias[2];

        //4. Print acceleration values in milligs!
        //System_printf("X");

        System_printf("X-acceleration: %f mg\r\n", 1000 * ax);
        System_printf("Y-acceleration: %f mg\r\n", 1000 * ay);
        System_printf("Z-acceleration: %f mg\r\n", 1000 * az);

        // Print gyro values in degree/sec
        System_printf("X-gyro rate: %f degrees/sec\r\n", gx);
        System_printf("Y-gyro rate: %f degrees/sec\r\n", gy);
        System_printf("Z-gyro rate: %f degrees/sec\r\n", gz);

        // Print mag values in degree/sec
        System_printf("X-mag field: %f mG\r\n", mx);
        System_printf("Y-mag field: %f mG\r\n", my);
        System_printf("Z-mag field: %f mG\r\n", mz);

        tempCount = readTempData();  // Read the adc values
        temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
        // Print temperature in degrees Centigrade
        System_printf("Temperature is %f degrees C\r\n"); // Print T values to tenths of s degree C

    }
}
*/
