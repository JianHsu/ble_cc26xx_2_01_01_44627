#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <Board.h>
#include <mpu9150.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include "gyro.h"
#include "util.h"

KALMAN_STRUCT *kalman;
I2C_Handle      i2c;
I2C_Params      i2cParams;
uint8_t         txBuffer[16];
uint8_t         rxBuffer[16];

/* Pin driver handles */
//static PIN_Handle ledPinHandle;

//static PIN_State ledPinState;

/*PIN_Config ledPinTable[] = {
    Board_PIN_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};*/

#define MSGSIZE 256

static Clock_Struct ChangeClock;
extern UART_Handle uart;

static void Update_Callback(UArg a0);
void i2cdummy(I2C_Handle handle, I2C_Transaction *transaction, bool transferStatus);
int Buffer2String( uint8_t *src, int srclen, uint8_t *dest );

void Gyro_init(void)
{
    kalman = malloc(sizeof(KALMAN_STRUCT));
    Kanman_Init(kalman);
    I2C_init();
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2cParams.transferMode = I2C_MODE_CALLBACK;
    i2cParams.transferCallbackFxn = i2cdummy;
    i2c = I2C_open(Board_I2C0, &i2cParams);
    if (i2c == NULL) {
        //Display_printf(display, 0, 0, "Error Initializing I2C\n");
        while (1);
    }
    else {
        //Display_printf(display, 0, 0, "I2C Initialized!\n");
    }
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);
    //GPIO_write(Board_GPIO_LED1, Board_GPIO_LED_OFF);
    Util_constructClock(&ChangeClock, Update_Callback, 500, 0, true, 0);
    //ledPinHandle = PIN_open(&ledPinState, ledPinTable);
}

void Kanman_Init(KALMAN_STRUCT * kalman)
{
    int i;
    //输出
    (*kalman).Angel = 0.0;  //最优估计的角度   是最终角度结果
    (*kalman).Gyro_x = 0.0; //最优估计角速度

    //固定参量
    (*kalman).Q_Angle = 0.001;      //{0.001,0.001,0.001};  //陀螺仪噪声协方差  0.001是经验值
    (*kalman).Q_Gyro = 0.003;       //{0.003,0.003,0.003};  //陀螺仪漂移噪声协方差    是mpu6500的经验值
    (*kalman).R_Angle = 0.5;        //{0.5,0.5,0.5};    //是加速度计噪声的协方差

    (*kalman).C_0 = 1;      //{1,1,1};  //H矩阵的一个观测参数 是常数

    //中间量
    (*kalman).Q_Bias = 0;       //{0,0,0};      //陀螺仪飘移预估值
    (*kalman).Angle_err = 0;    //{0,0,0};      //计算中间值 Angle 观测值-预估值

    (*kalman).PCt_0 = 0;            //{0,0,0},  //计算中间值
    (*kalman).PCt_1 = 0;            //{0,0,0},
    (*kalman).E     = 0;            //{0,0,0};
    (*kalman).t_0   = 0;            //{0,0,0},  //t:计算中间变量
    (*kalman).t_1   = 0;            //{0,0,0},

    (*kalman).K_0 = 0;          //{0,0,0},  //K:卡尔曼增益
    (*kalman).K_1 = 0;          //{0,0,0},

    for(i = 0;i < 4;i++)    //{0,0,0,0} //计算P矩阵的中间矩阵
    {
        (*kalman).Pdot[i] = 0;
    }

    (*kalman).PP[0][0] = 1;
    (*kalman).PP[0][1] = 0;
    (*kalman).PP[1][0] = 0;
    (*kalman).PP[1][1] = 1;
}


void Kanman_Filter(KALMAN_STRUCT * kalman,float Gyro,float Accel,unsigned int dt)    //Gyro陀螺仪的测量值  |  Accel加速度计的角度计  |  dt的时间考虑用小数 或 更小的分度表示
{
    float dt_f;
    //把dt这个单位是ms的u32型变量里的值转换为float型的以秒为单位的值
    dt_f = (float)dt;
    dt_f = dt_f / 1000;
    //x轴指向前，y轴指向左的坐标系  要算俯仰角
    //那么输入的应该是y轴的角速度（Gyro）和y轴的倾角加速度计估计值
    //坐标系情况大概是这样
    //角度测量模型方程 角度估计值=上一次最有角度+（角速度-上一次的最优零飘）*dt_f
    //就漂移来说，认为每次都是相同的Q_bias=Q_bias
    //估计角度
    (*kalman).Angel += (Gyro - (*kalman).Q_Bias) * dt_f;

    //计算估计模型的方差
    (*kalman).Pdot[0] = (*kalman).Q_Angle - (*kalman).PP[0][1] - (*kalman).PP[1][0];
    (*kalman).Pdot[1] = -(*kalman).PP[1][1];
    (*kalman).Pdot[2] = -(*kalman).PP[1][1];
    (*kalman).Pdot[3] = (*kalman).Q_Gyro;

    (*kalman).PP[0][0] += (*kalman).Pdot[0] * dt_f;
    (*kalman).PP[0][1] += (*kalman).Pdot[1] * dt_f;
    (*kalman).PP[1][0] += (*kalman).Pdot[2] * dt_f;
    (*kalman).PP[1][1] += (*kalman).Pdot[3] * dt_f;

    //计算卡尔曼增益
    (*kalman).PCt_0 = (*kalman).C_0 * (*kalman).PP[0][0];   //矩阵乘法的中间变量
    (*kalman).PCt_1 = (*kalman).C_0 * (*kalman).PP[0][1];   //C_0=1
    (*kalman).E = (*kalman).R_Angle + (*kalman).C_0 * (*kalman).PCt_0;  //分母
    (*kalman).K_0 = (*kalman).PCt_0 / (*kalman).E;  //卡尔曼增益，两个，一个是Angle的，一个是Q_bias的
    (*kalman).K_1 = (*kalman).PCt_1 / (*kalman).E;

    //计算最优角度、最优零飘
    (*kalman).Angle_err = Accel - (*kalman).Angel;
    (*kalman).Angel += (*kalman).K_0 * (*kalman).Angle_err; //计算最优的角度
    (*kalman).Q_Bias += (*kalman).K_1 * (*kalman).Angle_err;    //计算最优的零飘

    (*kalman).Gyro_x = Gyro -(*kalman).Q_Bias;  //计算得最优角速度

    //更新估计模型的方差
    (*kalman).t_0 = (*kalman).PCt_0;    //矩阵计算中间变量，相当于a
    (*kalman).t_1 = (*kalman).C_0 * (*kalman).PP[0][1]; //矩阵计算中间变量，相当于b

    (*kalman).PP[0][0] -= (*kalman).K_0 * (*kalman).t_0;
    (*kalman).PP[0][1] -= (*kalman).K_0 * (*kalman).t_1;
    (*kalman).PP[1][0] -= (*kalman).K_1 * (*kalman).t_0;
    (*kalman).PP[1][1] -= (*kalman).K_1 * (*kalman).t_1;
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

static void Update_Callback(UArg a0)
{
    //static uint8_t i = 0;
    //char a[20] = {0};
    //sprintf(a,"%d",i);
    char dis_buf[12] = {0};
    I2C_Transaction i2cTransaction;
    txBuffer[0] = 0x43;
    i2cTransaction.slaveAddress = MPU9150_DEFAULT_ADDRESS;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 6;
    I2C_transfer(i2c, &i2cTransaction);
    //i++;
    Buffer2String(rxBuffer, 6, dis_buf);
    UART_write(uart, dis_buf, sizeof(dis_buf));
    Util_startClock(&ChangeClock);
}

int Buffer2String( uint8_t *src, int srclen, uint8_t *dest )
{
    int i;
    for( i = 0; i < srclen; i++ ) {
        char tmp[3] = { 0 };
        sprintf( tmp, "%x", *( src + i ) & 0xff );
        if( strlen( tmp ) == 1 ) {
            strcat( ( char * )dest, "0" );
            strncat( ( char * )dest, tmp, 1 );
        } else if( strlen( tmp ) == 2 ) {
            strncat( ( char * )dest, tmp, 2 );
        } else {
            strcat( ( char * )dest, "00" );
        }
    }
    return i * 2;
}

void i2cdummy(I2C_Handle handle, I2C_Transaction *transaction, bool transferStatus)
{

}
