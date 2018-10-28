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
#include "hiddev.h"

#include "AHRS.h"
/* -----------------------------------------------------------------------------
*  Constants
* ------------------------------------------------------------------------------
*/
/*For debug*/
#define Debug_Msg_Rawdata       0
#define Debug_Msg_Quaternion    0
#define Debug_Msg_Eular         0
#define Send_Mouse_Report       1
#define Send_Keyboard_Report    0
/*End of debug*/

#define Attitude_Index          0
#define Mouse_Index             2
#define Function_Index          3

/*Attitude Mode Detect Angle Value*/
//Normal(平穩) = 1, Up(上仰) = 2, Down(下俯) = 3, Right(右翻) = 4, Left(左翻) = 5
/*MODE 1 - Normal*/
#define NMode_pitch_pos         30
#define NMode_pitch_neg         -30
#define NMode_roll_pos          30
#define NMode_roll_neg          -30
/*MODE 2 - Up*/
#define UMode_pitch_pos         60
#define UMode_roll_pos          30
#define UMode_roll_neg          -30
/*MODE 3 - Down*/
#define DMode_pitch_neg         -60
#define DMode_roll_pos          30
#define DMode_roll_neg          -30
/*MODE 4 - Right*/
#define RMode_pitch_pos         30
#define RMode_pitch_neg         -30
#define RMode_roll_pos          60
/*MODE 5 - Left*/
#define LMode_pitch_pos         30
#define LMode_pitch_neg         -30
#define LMode_roll_neg          -60

#define Kp                          2.0f         // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki                          0.005f       // integral gain governs rate of convergence of gyroscope biases
#define PI                          M_PI
#define AHRS_TASK_STACK_SIZE        644
#define AHRS_TASK_PRIORITY          1

#define ScreenX                     1920
#define ScreenY                     1080
#define Scale                       1

#define DELAY_MS(i)      (Task_sleep(((i) * 1000) / Clock_tickPeriod))
#define DELAY_US(i)      (Task_sleep(((i) * 1) / Clock_tickPeriod))
/* -----------------------------------------------------------------------------
*  Public Variables
* ------------------------------------------------------------------------------
*/
#define MPU9250_Number              6
typedef struct MPU9250
{
    uint8_t id;
    uint8_t visable;
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    float offset_mx, offset_my, offset_mz;
    float quaternion[4];
    float exInt, eyInt, ezInt;
    uint32_t last, now;
    float deltat;
    float yaw, pitch, roll;
} MPU9250;

struct MPU9250 mpu9250[MPU9250_Number];
/* -----------------------------------------------------------------------------
*  Private Variables
* ------------------------------------------------------------------------------
*/
//Key btn's value.
extern uint8_t keysPressed;
//Read 9-axis raw data's buffer.
int16_t accelCount[3], gyroCount[3], magCount[3];

uint8_t attitude_mode = 0, attitude_mode_f = 0;
uint8_t mode_change, mode_action_lock;
float change_data_yaw;
uint8_t yaw_left, yaw_right;
//Save the previous attitude.(Send_Mouse)
float yaw_f, pitch_f, roll_f;   //前次姿態

Task_Struct AHRSTask;
Char AHRSTaskStack[AHRS_TASK_STACK_SIZE];

int8_t function_angle = 0;
/* -----------------------------------------------------------------------------
*  Public Functions
* ------------------------------------------------------------------------------
*/
void Detect_Sensor(void);
void Setting_Mag_Offset(void);
void Read_Data(uint8_t sel);
void Get_DeltaT(uint8_t sel);
void AHRSupdate(uint8_t sel, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void QtoEular(uint8_t sel, float q0, float q1, float q2, float q3);
void AHRS_taskFxn(UArg a0, UArg a1);
uint8_t Attitude_Detect(float yaw, float pitch, float roll);
void Attitude_Action(uint8_t action);
void Send_Mouse(float yaw, float pitch, float roll, uint8_t key);
void Send_Keyboard(uint8_t action);
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
    uint8_t i, keyvalue;
    float angle;
/*
    PWM_Handle pwm_r, pwm_g, pwm_b;
    PWM_Params PWM_params;
    uint16_t   pwmPeriod = 5000;      // Period and duty in microseconds
/*
    ADC_Handle   adc;
    ADC_Params   ADC_params;
    uint16_t adcValue0;
*/

//  PWM control RGB LED.
    /*
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
/*
    ADC_init();
    ADC_Params_init(&ADC_params);
    adc = ADC_open(Board_ADC0, &ADC_params);
*/
    SensorI2C_open();

    UART_Params uartParams;
    UART_Params_init(&uartParams);
    uartParams.baudRate = 115200;
    UartPrintf_init(UART_open(Board_UART, &uartParams));
    DELAY_MS(1000);

    /*
     *  FreeWrite start.
     */
    //1. Detect sensor status.
    Detect_Sensor();
    DELAY_MS(100);
    //2. Initialize sensor which is online.
    for (i = 0; i < MPU9250_Number; i++)
    {
        if (mpu9250[i].visable)
        {
            mpu9250[i].quaternion[0] = 1;
            mpu9250[i].quaternion[1] = 0;
            mpu9250[i].quaternion[2] = 0;
            mpu9250[i].quaternion[3] = 0;
            MPU9250_init(i);
        }
    }

    //3. Set the Magnetometer's offset.
    Setting_Mag_Offset();

    for (;;)
    {
        //取得每指姿態
        for (i = 0; i < MPU9250_Number; i++)
        {
            if (mpu9250[i].visable)
            {
                SensorI2C_Select(i);
                Read_Data(i);
                Get_DeltaT(i);

                //計算姿態
                AHRSupdate(i, mpu9250[i].ax, mpu9250[i].ay, mpu9250[i].az, mpu9250[i].gx * PI / 180.0, mpu9250[i].gy * PI / 180.0, mpu9250[i].gz * PI / 180.0, mpu9250[i].mx, mpu9250[i].my, mpu9250[i].mz);
#if Debug_Msg_Quaternion
                System_printf("[%d] : %f, %f, %f, %f", i, mpu9250[i].quaternion[0], mpu9250[i].quaternion[1], mpu9250[i].quaternion[2], mpu9250[i].quaternion[3]);
#endif
                //轉為歐拉角
                //QtoEular(quaternion[0], quaternion[1], -quaternion[2], -quaternion[3]);
                QtoEular(i, mpu9250[i].quaternion[0], -mpu9250[i].quaternion[1], mpu9250[i].quaternion[2], -mpu9250[i].quaternion[3]);
#if Debug_Msg_Eular

                //System_printf("[%d] : %f, %f, %f\n", i, mpu9250[i].yaw, mpu9250[i].pitch, mpu9250[i].roll);
                //System_printf("[%d] : %d, %d, %d\n", i, (int)mpu9250[i].yaw, (int)mpu9250[i].pitch, (int)mpu9250[i].roll);

#endif
            }
        }

        //手勢模式判斷
        attitude_mode = Attitude_Detect(mpu9250[Attitude_Index].yaw, mpu9250[Attitude_Index].pitch, mpu9250[Attitude_Index].roll);
        if ((attitude_mode != attitude_mode_f) && (attitude_mode != 0))
        {
            attitude_mode_f = attitude_mode;
            mode_change = TRUE;
            mode_action_lock = FALSE;
            //System_printf("Change mode: %d\r\n", attitude_mode_f);
        }
        else
        {
            mode_change = FALSE;
        }

        //手勢動作
        switch(attitude_mode)
        {
            //1. Normal Mode : 移動指標
            case Normal_Mode:
                if (mode_change)
                {
                    pitch_f = mpu9250[Mouse_Index].pitch;
                    yaw_f = mpu9250[Mouse_Index].yaw;
                }

                //移動鼠標
                Send_Mouse(mpu9250[Mouse_Index].yaw, mpu9250[Mouse_Index].pitch, mpu9250[Mouse_Index].roll, keysPressed);

                break;
            //2. Up Mode : 左右搖動，清除筆跡
            case Up_Mode:
                if (mode_change)
                {
                    yaw_left = FALSE;
                    yaw_right = FALSE;
                    change_data_yaw = mpu9250[Attitude_Index].yaw;
                }
                else
                {
                    //處理邊界(180, -180)
                    angle = mpu9250[Attitude_Index].yaw - change_data_yaw;
                    if (angle < -180) angle += 360;
                    if (angle > 180) angle -= 360;

                    if (angle < -15){
                        //System_printf("Clear_L.\r\n");
                        yaw_left = TRUE;
                    }
                    if (angle > 15){
                        //System_printf("Clear_R.\r\n");
                        yaw_right = TRUE;
                    }
                    if (yaw_left && yaw_right) {
                        if (!mode_action_lock)
                        {
                            //System_printf("Clear.\r\n");
                            Send_Keyboard(Key_Clear);
                        }

                        mode_action_lock = TRUE;
                    }
                }
                break;
            //3. Down Mode : 停止動作
            case Down_Mode:
                //Do nothing.
                break;
            //4. Right Mode : 左搖-上一頁，右搖-下一頁，按鍵-快切
            case Right_Mode:
                if (mode_change)
                {
                    change_data_yaw = mpu9250[Attitude_Index].yaw;
                }
                else
                {
                    //處理邊界(180, -180)
                    angle = mpu9250[Attitude_Index].yaw - change_data_yaw;
                    if (angle < -180) angle += 360;
                    if (angle > 180) angle -= 360;
                    if (angle > 15)
                    {
                        if (!mode_action_lock)
                        {
                            //System_printf("Next page.\r\n");
                            Send_Keyboard(Key_Next_Page);
                        }

                        if (keysPressed)
                            change_data_yaw = mpu9250[Attitude_Index].yaw;
                        else
                            mode_action_lock = TRUE;
                    }
                    if (angle < -15)
                    {
                        if (!mode_action_lock)
                        {
                            //System_printf("Previous page.\r\n");
                            Send_Keyboard(Key_Previous_Page);
                        }
                        if (keysPressed)
                            change_data_yaw = mpu9250[Attitude_Index].yaw;
                        else
                            mode_action_lock = TRUE;
                    }
                }
                break;
        }

        //ADC_convert(adc, &adcValue0);
        //System_printf("%d\n", adcValue0);


        //DELAY_MS(5);
    }
}

void Detect_Sensor(void)
{
    uint8_t i, res, val;

    System_printf("MPU9250 detect...\r\n");

    for (i = 0; i < MPU9250_Number; i++)
    {
        System_printf("MPU9250[%d] : ", i);
        mpu9250[i].id = i;
        mpu9250[i].visable = 0;
        res = SensorI2C_Select(i);
        DELAY_MS(10);
        if (res)
        {
            //Read WAI Reg.
            res = SensorI2C_readReg(MPU9250_ADDRESS, WHO_AM_I, &val, 1);
            if (res && (val == 0x71))
            {
                mpu9250[i].visable = 1;
                System_printf("1\r\n");
            }
            else
            {
                System_printf("0\r\n");
            }
        }
    }
    System_printf("\r\n");
}

void Setting_Mag_Offset(void)
{
    /*Index 0*/
    mpu9250[0].offset_mx = 21.0;
    mpu9250[0].offset_my = 134.95;
    mpu9250[0].offset_mz = 239.15;
    /*Index 1*/
    mpu9250[1].offset_mx = 0.0;
    mpu9250[1].offset_my = 0.0;
    mpu9250[1].offset_mz = 0.0;
    /*Index 2. (Version2 - 1)*/
    mpu9250[2].offset_mx = 100.46;
    mpu9250[2].offset_my = 84.72;
    mpu9250[2].offset_mz = 262.39;
    /*Index 3. (Version2 - 2)*/
    mpu9250[3].offset_mx = 371.84;
    mpu9250[3].offset_my = 82.47;
    mpu9250[3].offset_mz = 230.90;
    /*Index 4*/
    mpu9250[4].offset_mx = 0.0;
    mpu9250[4].offset_my = 0.0;
    mpu9250[4].offset_mz = 0.0;
    /*Index 5*/
    mpu9250[5].offset_mx = 0.0;
    mpu9250[5].offset_my = 0.0;
    mpu9250[5].offset_mz = 0.0;
    /*
    //(Version1 - 2)
    mpu9250[3].offset_mx = 380.84;
    mpu9250[3].offset_my = 90.71;
    mpu9250[3].offset_mz = 257.89;
    */
}

void Read_Data(uint8_t sel)
{

    // 1. Read accel val
    readAccelData(accelCount);
    mpu9250[sel].ax = getAres(accelCount[0]);
    mpu9250[sel].ay = getAres(accelCount[1]);
    mpu9250[sel].az = getAres(accelCount[2]);

    //2. Read gyro val
    readGyroData(gyroCount);
    mpu9250[sel].gx = getGres(gyroCount[0]);
    mpu9250[sel].gy = getGres(gyroCount[1]);
    mpu9250[sel].gz = getGres(gyroCount[2]);

    //3. Read mag val
    readMagData(magCount);
    mpu9250[sel].mx = getMres(magCount[1]);
    mpu9250[sel].my = getMres(magCount[0]);
    mpu9250[sel].mz = -getMres(magCount[2]);

    //4. Offset
    mpu9250[sel].mx -= mpu9250[sel].offset_mx;
    mpu9250[sel].my -= mpu9250[sel].offset_my;
    mpu9250[sel].mz -= mpu9250[sel].offset_mz;

#if Debug_Msg_Rawdata
    if(sel == 2)
    {
    System_printf("[%d] : %f,%f,%f,", sel, mpu9250[sel].ax, mpu9250[sel].ay, mpu9250[sel].az);
    System_printf("%f,%f,%f,", mpu9250[sel].gx, mpu9250[sel].gy, mpu9250[sel].gz);
    System_printf("%f,%f,%f,", mpu9250[sel].mx, mpu9250[sel].my, mpu9250[sel].mz);
    }
#endif
}
void Get_DeltaT(uint8_t sel)
{
    //計算deltat，單位(s)
    mpu9250[sel].now = Clock_getTicks();
    mpu9250[sel].deltat = (mpu9250[sel].now - mpu9250[sel].last) / 100000.0f;
    mpu9250[sel].last = mpu9250[sel].now;
#if Debug_Msg_Rawdata
    if(sel == 3)
    {
    System_printf("%f\r\n", mpu9250[sel].deltat);
    }

#endif
}

//Normal(平穩) = 1, Up(上仰) = 2, Down(下俯) = 3, Right(右翻) = 4, Left(左翻) = 5
uint8_t Attitude_Detect(float yaw, float pitch, float roll)
{
    if ((pitch > NMode_pitch_neg) && (pitch < NMode_pitch_pos) && (roll > NMode_roll_neg) && (roll < NMode_roll_pos)) return 1;
    else if ((pitch > UMode_pitch_pos) && (roll > UMode_roll_neg) && (roll < UMode_roll_pos)) return 2;
    else if ((pitch < DMode_pitch_neg) && (roll > DMode_roll_neg) && (roll < DMode_roll_pos)) return 3;
    else if ((roll > RMode_roll_pos) && (pitch > RMode_pitch_neg) && (pitch < RMode_pitch_pos)) return 4;
    else if ((roll < LMode_roll_neg) && (pitch > LMode_pitch_neg) && (pitch < LMode_pitch_pos)) return 5;
    else return 0;
}

void Send_Mouse(float yaw, float pitch, float roll, uint8_t key)
{
    uint8_t mouse_report[5];

    //System_printf("%f, %f\n", yaw, pitch);

    float dyaw = yaw - yaw_f;
    float dpitch = pitch - pitch_f;
    float dx = dyaw * ScreenX / 80 * Scale;
    float dy = -dpitch * ScreenY / 60 * Scale;

    //處理交界部分
    if (dyaw > 180 || dyaw < -180) dx = 0;

    //送出封包
    mouse_report[0] = key;
    if((dx > 5) || (dx < -5))
        mouse_report[1] = dx;
    else
        mouse_report[1] = 0;
    if((dy > 5) || (dy < -5))
        mouse_report[2] = dy;
    else
        mouse_report[2] = 0;
    mouse_report[3] = 0;
#if Send_Mouse_Report
    //if((dx > 3)||(dy > 3)||(dx < -3)||(dy < -3))
        HidEmuKbd_enqueueMsg(HIDEMUKBD_MOUSE_CHANGE_EVT, mouse_report, 0);
#endif

    yaw_f = yaw;
    pitch_f = pitch;
}

void Send_Keyboard(uint8_t action)
{
    uint8_t keyboard_report[8] = {0};
    uint8_t mouse_report[5] = {0};
#if Send_Keyboard_Report
    switch(action)
    {
        case Key_Next_Page:
            keyboard_report[3] = HID_KEYBOARD_RIGHT_ARROW;
            HidEmuKbd_enqueueMsg(HIDEMUKBD_KEY_CHANGE_EVT, mouse_report, keyboard_report);
            break;
        case Key_Previous_Page:
            keyboard_report[3] = HID_KEYBOARD_LEFT_ARROW;
            HidEmuKbd_enqueueMsg(HIDEMUKBD_KEY_CHANGE_EVT, mouse_report, keyboard_report);
            break;
        case Key_Clear:
            keyboard_report[3] = HID_KEYBOARD_E;
            HidEmuKbd_enqueueMsg(HIDEMUKBD_KEY_CHANGE_EVT, mouse_report, keyboard_report);
            break;
        case Key_Zoom_In:
            keyboard_report[3] = HID_KEYBOARD_LEFT_CTRL;
            mouse_report[3] = 1;    //wheel
            HidEmuKbd_enqueueMsg(HIDEMUKBD_KEY_MOUSE_CHANGE_EVT, mouse_report, keyboard_report);
            break;
        case Key_Zoom_Out:
            keyboard_report[3] = HID_KEYBOARD_LEFT_CTRL;
            mouse_report[3] = -1;   //wheel
            HidEmuKbd_enqueueMsg(HIDEMUKBD_KEY_MOUSE_CHANGE_EVT, mouse_report, keyboard_report);
            break;
    }
#endif
}

void MPU9250_init(uint8_t sel){

    uint8_t val;

    SensorI2C_Select(sel);

    SensorI2C_writeReg(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);
    System_printf("MPU9250[%d] RESTART\r\n", sel);
    DELAY_MS(100);

    SensorI2C_readReg(MPU9250_ADDRESS, WHO_AM_I, &val, 1);
    System_printf("MPU9250[%d] WAI:0x%x\r\n", sel, val);


    if (val == 0x71 || val == 0x73)
    {
        /*
        MPU9250SelfTest(SelfTest);
        System_printf("x-acceleration self test: %f\r\n", SelfTest[0]);
        System_printf("y-acceleration self test: %f\r\n", SelfTest[1]);
        System_printf("z-acceleration self test: %f\r\n", SelfTest[2]);
        System_printf("x-gyration self test: %f\r\n", SelfTest[3]);
        System_printf("y-gyration self test: %f\r\n", SelfTest[4]);
        System_printf("z-gyration self test: %f\r\n", SelfTest[5]);
        */
        /*
        calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
        System_printf("MPU9250 bias\r\n");
        System_printf(" x   y   z  \r\n");

        System_printf("%d, %d, %d mg\r\n",(int)(1000*accelBias[0]), (int)(1000*accelBias[1]), (int)(1000*accelBias[2]));
        System_printf("%f, %f, %f o/s\r\n", gyroBias[0], gyroBias[1], gyroBias[2]);
        */
        DELAY_MS(100);

        initMPU9250();
    }
    else{
        System_printf("Can't find MPU9250[%d]!!!\r\n", sel);
        DELAY_MS(100);  //Wait uart send msg.
        while(1);
    }

    /*
     * MPU I2C Master Mode.
     */

    //啟用I2C Master Mode.
    //System_printf("I2c Master Mode.\r\n");

    //讀取0x0c, 0x00, 1byte
    AK8963_read(0x00, &val, 1);
    System_printf("AK8963[%d] WAI:0x%x\r\n", sel, val);

    /*
     * MPU I2C Bypass Mode.
     */
/*
    SensorI2C_writeReg(MPU9250_ADDRESS, USER_CTRL, 0x00);
    System_printf("MPU9250 0x6A - 0x00\r\n");
    SensorI2C_writeReg(MPU9250_ADDRESS, INT_PIN_CFG, 0x02);
    System_printf("MPU9250 0x37 - 0x02\r\n");
    DELAY_MS(100);
    SensorI2C_readReg(AK8963_ADDRESS, MAG_WHO_AM_I, &val, 1);
    System_printf("AK8963 WAI:%d\r\n", val);
    DELAY_MS(100);  //Wait uart send msg.
*/

    if (val == 0x48){
        // Get magnetometer calibration from AK8963 ROM
        initAK8963();
        /*
        magcalMPU9250(magbias, magCalibration);
        System_printf("AK8963 initialized for active data mode....\r\n"); // Initialize device for active mode read of magnetometer
        System_printf("AK8963 mag biases: %f, %f, %f\r\n", magbias[0], magbias[1], magbias[2]);
        System_printf("X-Axis sensitivity adjustment value: %f\r\n", magCalibration[0]);
        System_printf("Y-Axis sensitivity adjustment value: %f\r\n", magCalibration[1]);
        System_printf("Z-Axis sensitivity adjustment value: %f\r\n", magCalibration[2]);
        */
    }
    else
    {
        System_printf("Can't find AK8963[%d]!!!\r\n", sel);
        DELAY_MS(100);  //Wait uart send msg.
        while(1);
    }
    System_printf("MPU9250[%d] initialized!!!\r\n", sel);
}

void AHRSupdate(uint8_t sel, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    float q1 = mpu9250[sel].quaternion[0], q2 = mpu9250[sel].quaternion[1], q3 = mpu9250[sel].quaternion[2], q4 = mpu9250[sel].quaternion[3];   // short name local variable for readability
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
        mpu9250[sel].exInt += ex;      // accumulate integral error
        mpu9250[sel].eyInt += ey;
        mpu9250[sel].ezInt += ez;
    }
    else
    {
        mpu9250[sel].exInt = 0.0f;     // prevent integral wind up
        mpu9250[sel].eyInt = 0.0f;
        mpu9250[sel].ezInt = 0.0f;
    }

    // Apply feedback terms
    gx = gx + Kp * ex + Ki * mpu9250[sel].exInt;
    gy = gy + Kp * ey + Ki * mpu9250[sel].eyInt;
    gz = gz + Kp * ez + Ki * mpu9250[sel].ezInt;

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * mpu9250[sel].deltat);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * mpu9250[sel].deltat);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * mpu9250[sel].deltat);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * mpu9250[sel].deltat);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    mpu9250[sel].quaternion[0] = q1 * norm;
    mpu9250[sel].quaternion[1] = q2 * norm;
    mpu9250[sel].quaternion[2] = q3 * norm;
    mpu9250[sel].quaternion[3] = q4 * norm;
}


void QtoEular(uint8_t sel, float q0, float q1, float q2, float q3) {
    mpu9250[sel].yaw = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
    mpu9250[sel].pitch = -asin(2.0f * (q1 * q3 - q0 * q2));
    mpu9250[sel].roll = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    mpu9250[sel].yaw *= 180 / PI;
    mpu9250[sel].pitch *= 180 / PI;
    mpu9250[sel].roll *= 180 / PI;
}

