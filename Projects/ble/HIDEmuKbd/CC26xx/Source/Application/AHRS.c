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
#include <ti/drivers/PWM.h>
#include <ti/drivers/ADC.h>

#include "math.h"

#include "util.h"
#include "Board.h"
#include "uart_printf.h"

#include "myi2c.h"
#include "mpu9250.h"
#include "hidemukbd.h"

#include "AHRS.h"
/* -----------------------------------------------------------------------------
*  Constants
* ------------------------------------------------------------------------------
*/
#define Kp                          2.0f         // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki                          0.005f       // integral gain governs rate of convergence of gyroscope biases
#define PI                          M_PI
#define AHRS_TASK_STACK_SIZE        644
#define AHRS_TASK_PRIORITY          1

#define ScreenX                     1920
#define ScreenY                     1080
#define Scale                       0.5

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

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
float magCalibrate[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float magCalibration[3] = {0, 0, 0};
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t accelCount[3], gyroCount[3], magCount[3];  // Stores the 16-bit signed accelerometer sensor output
float   SelfTest[6];    // holds results of gyro and accelerometer self test

uint32_t last, now;
float deltat;

float exInt = 0, eyInt = 0, ezInt = 0;  // scaled integral error

float quaternion[4] = {1, 0, 0, 0};   // quaternion elements representing the estimated orientation
float yaw, pitch, roll;         //當前姿態

float yaw_f, pitch_f, roll_f;   //前次姿態

Task_Struct AHRSTask;
Char AHRSTaskStack[AHRS_TASK_STACK_SIZE];
/* -----------------------------------------------------------------------------
*  Public Functions
* ------------------------------------------------------------------------------
*/
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void QtoEular(float q0, float q1, float q2, float q3);
void AHRS_taskFxn(UArg a0, UArg a1);
void Move_Mouse(float yaw, float pitch, float roll);
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
    uint32_t x, y;
    float z;

    PWM_Handle pwm_r, pwm_g, pwm_b;
    PWM_Params PWM_params;
    uint16_t   pwmPeriod = 5000;      // Period and duty in microseconds

    ADC_Handle   adc;
    ADC_Params   ADC_params;
    uint16_t adcValue0;

    /*PWM control RGB LED.
    PWM_Params_init(&PWM_params);
    PWM_params.dutyUnits = PWM_DUTY_US;
    PWM_params.dutyValue = 0;
    PWM_params.periodUnits = PWM_PERIOD_US;
    PWM_params.periodValue = pwmPeriod;
    pwm_r = PWM_open(Board_PWM0, &PWM_params);
    pwm_g = PWM_open(Board_PWM1, &PWM_params);
    pwm_b = PWM_open(Board_PWM2, &PWM_params);

    PWM_start(pwm_r);
    PWM_start(pwm_g);
    PWM_start(pwm_b);
    PWM_setDuty(pwm_r, 0);
    PWM_setDuty(pwm_g, 0);
    PWM_setDuty(pwm_b, 0);

    ADC_init();
    ADC_Params_init(&ADC_params);
    adc = ADC_open(Board_ADC0, &ADC_params);

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
        mx = getMres(magCount[1]);// * magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
        my = getMres(magCount[0]);// * magCalibration[1] - magbias[1];
        mz = -getMres(magCount[2]);// * magCalibration[2] - magbias[2]);

        //減去offset
        //MPU9255
/*
        mx -= 266.63f;
        my -= 202.67f;
        mz -= 176.14f;
*/
        //MPU9250(0)
        mx -= 88.46f;
        my -= -360.60f;
        mz -= 218.16f;

        //MPU9250(1)
//        mx -= 258.63f;
//        my -= 78.71f;
//        mz -= 306.62f;

        //計算deltat，單位(s)
        now = Clock_getTicks();
        deltat = (now - last) / 100000.0f;
        last = now;
/*
        send_cnt++;
        if (send_cnt & 0x01)
        {
            System_printf("%f,%f,%f,", ax, ay, az);
            System_printf("%f,%f,%f,", gx, gy, gz);
            System_printf("%f,%f,%f,", mx, my, mz);
            System_printf("%f\r\n", deltat * 2);
        }
*/

        //計算姿態
        x = Clock_getTicks();
        AHRSupdate(ax, ay, az, gx * PI / 180.0, gy * PI / 180.0, gz * PI / 180.0, mx, my, mz);
        System_printf("%f, %f, %f, %f", quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
        //轉為歐拉角
        y = Clock_getTicks();
        z = (y - x) / 100.0f;

        //QtoEular(quaternion[0], quaternion[1], -quaternion[2], -quaternion[3]);
        QtoEular(quaternion[0], -quaternion[1], quaternion[2], -quaternion[3]);
        //System_printf("\t%f, %f, %f\n", yaw, pitch, roll);
        //移動鼠標
//        Move_Mouse(yaw, pitch, roll);


//        ADC_convert(adc, &adcValue0);
//        System_printf("%d\n", adcValue0);
        DELAY_MS(500);
    }
}

void Move_Mouse(float yaw, float pitch, float roll)
{
    float dyaw = yaw - yaw_f;
    float dpitch = pitch - pitch_f;
    float dx = dyaw * ScreenX / 2 / 30 * Scale;
    float dy = -dpitch * ScreenY / 2 / 30 * Scale;
    uint8_t mouse_report[5] = {0};
    //處理交界部分
    if (dyaw > 180 || dyaw < -180) dx = 0;

    //送出封包
    mouse_report[1] = dx;
    mouse_report[2] = dy;
    HidEmuKbd_enqueueMsg(2, 0, mouse_report, 0);

    yaw_f = yaw;
    pitch_f = pitch;
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
        //MPU9250SelfTest(SelfTest);
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

        DELAY_MS(100);

        initMPU9250();
    }
    else{
        System_printf("Can't find MPU9250!!!\r\n");
        DELAY_MS(100);  //Wait uart send msg.
        while(1);
    }

    /*
     * MPU I2C Master Mode.
     */
    //啟用I2C Master Mode.
    System_printf("I2c Master Mode.\r\n");

    //讀取0x0c, 0x00, 1byte
    AK8963_read(0x00, &val, 1);
    System_printf("AK8963 WAI:%d\r\n", val);

    /*
     * MPU I2C Bypass Mode.
     */
/*
    SensorI2C_writeReg(MPU9250_ADDRESS, 0x6A, 0x00);
    System_printf("MPU9250 0x6A - 0x00\r\n");
    SensorI2C_writeReg(MPU9250_ADDRESS, 0x37, 0x02);
    System_printf("MPU9250 0x37 - 0x02\r\n");
    DELAY_MS(100);
    SensorI2C_readReg(AK8963_ADDRESS, 0x00, &val, 1);
    System_printf("AK8963 WAI:%d\r\n", val);
    DELAY_MS(100);  //Wait uart send msg.
    while(1);
*/
    if (val == 0x48){
        // Get magnetometer calibration from AK8963 ROM
        initAK8963(magCalibration);
        //magcalMPU9250(magbias, magCalibration);
        System_printf("AK8963 initialized for active data mode....\r\n"); // Initialize device for active mode read of magnetometer
        System_printf("AK8963 mag biases: %f, %f, %f\r\n", magbias[0], magbias[1], magbias[2]);
        System_printf("X-Axis sensitivity adjustment value: %f\r\n", magCalibration[0]);
        System_printf("Y-Axis sensitivity adjustment value: %f\r\n", magCalibration[1]);
        System_printf("Z-Axis sensitivity adjustment value: %f\r\n", magCalibration[2]);
    }
    else
    {
        System_printf("Can't find AK8963!!!\r\n");
        DELAY_MS(100);  //Wait uart send msg.
        while(1);
    }
}

void AHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    float q1 = quaternion[0], q2 = quaternion[1], q3 = quaternion[2], q4 = quaternion[3];   // short name local variable for readability
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
    if (norm == 0) return; // handle NaN
    norm = 1 / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0) return; // handle NaN
    norm = 1 / norm;        // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    hx = 2 * mx * (0.5f - q3q3 - q4q4) + 2 * my * (q2q3 - q1q4) + 2 * mz * (q2q4 + q1q3);
    hy = 2 * mx * (q2q3 + q1q4) + 2 * my * (0.5f - q2q2 - q4q4) + 2 * mz * (q3q4 - q1q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = 2 * mx * (q2q4 - q1q3) + 2 * my * (q3q4 + q1q2) + 2 * mz * (0.5f - q2q2 - q3q3);

    // Estimated direction of gravity and magnetic field
    vx = 2 * (q2q4 - q1q3);
    vy = 2 * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2 * bx * (0.5f - q3q3 - q4q4) + 2 * bz * (q2q4 - q1q3);
    wy = 2 * bx * (q2q3 - q1q4) + 2 * bz * (q1q2 + q3q4);
    wz = 2 * bx * (q1q3 + q2q4) + 2 * bz * (0.5f - q2q2 - q3q3);

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (Ki > 0)
    {
        exInt += ex;      // accumulate integral error
        eyInt += ey;
        ezInt += ez;
    }
    else
    {
        exInt = 0.0f;     // prevent integral wind up
        eyInt = 0.0f;
        ezInt = 0.0f;
    }

    // Apply feedback terms
    gx = gx + Kp * ex + Ki * exInt;
    gy = gy + Kp * ey + Ki * eyInt;
    gz = gz + Kp * ez + Ki * ezInt;

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    quaternion[0] = q1 * norm;
    quaternion[1] = q2 * norm;
    quaternion[2] = q3 * norm;
    quaternion[3] = q4 * norm;
}


void QtoEular(float q0, float q1, float q2, float q3) {
    yaw = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
    pitch = -asin(2.0f * (q1 * q3 - q0 * q2));
    roll = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    yaw *= 180 / PI;
    pitch *= 180 / PI;
    roll *= 180 / PI;
}

