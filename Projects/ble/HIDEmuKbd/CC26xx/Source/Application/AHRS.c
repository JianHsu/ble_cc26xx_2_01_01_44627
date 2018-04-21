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
 *  @file       AHRS.c
 *
 *  @brief
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
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>

#include "math.h"

#include "util.h"
#include "Board.h"
#include "uart_printf.h"

#include "myi2c.h"
#include "mpu9250.h"

#include "AHRS.h"
/* -----------------------------------------------------------------------------
*  Constants
* ------------------------------------------------------------------------------
*/
#define Kp 2.0f         // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f       // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.5f * 100      // half the sample period
#define PI  M_PI
#define AHRS_TASK_STACK_SIZE       644
#define AHRS_TASK_PRIORITY         1

#define DELAY_MS(i)      (Task_sleep(((i) * 1000) / Clock_tickPeriod))
#define DELAY_US(i)      (Task_sleep(((i) * 1) / Clock_tickPeriod))
/* -----------------------------------------------------------------------------
*  Public Variables
* ------------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
*  Private Variables
* ------------------------------------------------------------------------------
*/
static Clock_Struct ChangeClock;
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
float magCalibrate[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float magCalibration[3] = {0, 0, 0};
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t accelCount[3], gyroCount[3], magCount[3];  // Stores the 16-bit signed accelerometer sensor output
float   SelfTest[6];    // holds results of gyro and accelerometer self test
volatile uint8_t clk_flag = 0;
uint8_t cnt_1s = 0;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;   // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;  // scaled integral error

float pitch, yaw, roll;

float deltat;
uint32_t lastUpdate;
Task_Struct AHRSTask;
Char AHRSTaskStack[AHRS_TASK_STACK_SIZE];

/* -----------------------------------------------------------------------------
*  Public Functions
* ------------------------------------------------------------------------------
*/
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void AHRS_taskFxn(UArg a0, UArg a1);

//===================================================================================================================
//======
//===================================================================================================================
void AHRS_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = AHRSTaskStack;
  taskParams.stackSize = AHRS_TASK_STACK_SIZE;
  taskParams.priority = AHRS_TASK_PRIORITY;

  Task_construct(&AHRSTask, AHRS_taskFxn, &taskParams, NULL);
}

void AHRS_taskFxn(UArg a0, UArg a1)
{
    /*
    I2C_init();
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C, &i2cParams);
    */
    SensorI2C_open();

    UART_Params uartParams;
    UART_Params_init(&uartParams);
    uartParams.baudRate = 115200;
    UartPrintf_init(UART_open(Board_UART, &uartParams));

    AHRS_init();

    for (;;)
    {
        //PINCC26XX_setOutputValue(Board_GLED, PINCC26XX_getOutputValue(Board_GLED) ^ 1);

        // 1. Read accel val
        readAccelData(accelCount);  // Read the x/y/z adc values
        ax = getAres(accelCount[0]);// - accelBias[0];  // get actual g value, this depends on scale being set
        ay = getAres(accelCount[1]);// - accelBias[1];
        az = getAres(accelCount[2]);// - accelBias[2];

        //2. Read gyro val
        readGyroData(gyroCount);  // Read the x/y/z adc values
        gx = getGres(gyroCount[0]);  // get actual gyro value, this depends on scale being set
        gy = getGres(gyroCount[1]);
        gz = getGres(gyroCount[2]);

        //3. Read mag val
        readMagData(magCount);  // Read the x/y/z adc values
        //magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
        //magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
        //magbias[2] = +125.;  // User environmental x-axis correction in milliGauss

        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental corrections
        mx = getMres(magCount[0]);// * magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
        my = getMres(magCount[1]);// * mRes*magCalibration[1] - magbias[1];
        mz = getMres(magCount[2]);// * mRes*magCalibration[2] - magbias[2];
        //System_printf("%d, %d, %d\r\n", magCount[0] ,magCount[1], magCount[2]);
        //System_printf("%f, %f, %f\r\n", mx ,my, mz);
        System_printf("%f,%f,%f,", ax, ay, az);
        System_printf("%f,%f,%f,", gx, gy, gz);
        System_printf("%f,%f,%f\n", mx ,my, mz);
        /*
        deltat = (Clock_getTicks() - lastUpdate) / 100000.0f;
        AHRSupdate(ax, ay, az, gx*PI/180.0, gy*PI/180.0, gz*PI/180.0, mx ,my, mz);
        //System_printf("%f, %f, %f, %f\r\n", q0, q1, q2, q3);
        lastUpdate = Clock_getTicks();
        yaw = atan2(2.0 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);
        pitch = -asin(2.0 * (q1*q3 - q0*q2));
        roll = atan2(2.0 * (q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3);

        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI;
        roll  *= 180.0f / PI;
        yaw += 180;
        System_printf("Yaw, Pitch, Roll: %f, %f, %f \r\n", yaw, pitch, roll);
        */
        DELAY_MS(10);
        //yaw   += 1.34;
        /* Declination at Potheri, Chennail ,India  Model Used:    IGRF12    Help
        Latitude:    12.823640¢X N
        Longitude:    80.043518¢X E
        Date    Declination
        2016-04-09    1.34¢X W  changing by  0.06¢X E per year (+ve for west )*/


        //PINCC26XX_setOutputValue(Board_GLED, PINCC26XX_getOutputValue(Board_GLED) ^ 1);

        /*
        I2C_Transaction i2cTransaction;
        txBuffer[0] = 0x43;
        i2cTransaction.slaveAddress = 0x68;
        i2cTransaction.writeBuf = txBuffer;
        i2cTransaction.writeCount = 1;
        i2cTransaction.readBuf = rxBuffer;
        i2cTransaction.readCount = 6;
        I2C_transfer(i2c, &i2cTransaction);


        */
    }
}

void AHRS_init(void){
    uint8_t val;

    SensorI2C_writeReg(MPU9250_ADDRESS, 0x6B, 0x80);
    System_printf("MPU9250 RESTART\r\n");
    DELAY_MS(100);

    SensorI2C_readReg(MPU9250_ADDRESS, 0x75, &val, 1);
    System_printf("MPU9250 WAI:%d\r\n", val);

    if (val == 0x71 || val == 0x73)
    {

        MPU9250SelfTest(SelfTest);
        System_printf("x-acceleration self test: %f\r\n", SelfTest[0]);
        System_printf("y-acceleration self test: %f\r\n", SelfTest[1]);
        System_printf("z-acceleration self test: %f\r\n", SelfTest[2]);
        System_printf("x-gyration self test: %f\r\n", SelfTest[3]);
        System_printf("y-gyration self test: %f\r\n", SelfTest[4]);
        System_printf("z-gyration self test: %f\r\n", SelfTest[5]);

        //calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
        System_printf("MPU9250 bias\r\n");
        System_printf(" x   y   z  \r\n");

        System_printf("%d, %d, %d mg\r\n",(int)(1000*accelBias[0]), (int)(1000*accelBias[1]), (int)(1000*accelBias[2]));
        System_printf("%f, %f, %f o/s\r\n", gyroBias[0], gyroBias[1], gyroBias[2]);

        DELAY_MS(1);

        initMPU9250();
    }

    SensorI2C_readReg(AK8963_ADDRESS, 0x00, &val, 1);
    System_printf("AK8963 WAI:%d\r\n", val);

    if (val == 0x48){
        // Get magnetometer calibration from AK8963 ROM
        initAK8963(magCalibration);

        System_printf("AK8963 initialized for active data mode....\r\n"); // Initialize device for active mode read of magnetometer
        System_printf("X-Axis sensitivity adjustment value: %f\r\n", magCalibration[0]);
        System_printf("Y-Axis sensitivity adjustment value: %f\r\n", magCalibration[1]);
        System_printf("Z-Axis sensitivity adjustment value: %f\r\n", magCalibration[2]);

    }
}

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;

    // auxiliary variables to reduce number of repeated operations
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    // normalise the measurements
    norm = sqrt(ax*ax + ay*ay + az*az);
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;
    norm = sqrt(mx*mx + my*my + mz*mz);
    mx = mx / norm;
    my = my / norm;
    mz = mz / norm;

    // compute reference direction of flux
    hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
    hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
    hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz;

    // estimated direction of gravity and flux (v and w)
    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
    wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
    wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);

    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    // integral error scaled integral gain
    exInt = exInt + ex*Ki;
    eyInt = eyInt + ey*Ki;
    ezInt = ezInt + ez*Ki;

    // adjusted gyroscope measurements
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;

    // integrate quaternion rate and normalise
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*0.5f * deltat;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*0.5f * deltat;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*0.5f * deltat;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*0.5f * deltat;

    // normalise quaternion
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
}
